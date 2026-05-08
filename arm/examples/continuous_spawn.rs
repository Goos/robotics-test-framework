//! Continuous spawn-and-place scenario: blocks are scheduled to drop into
//! the search region one at a time at fixed intervals. The controller
//! sweeps, grasps, places, then loops — until N total blocks land in the
//! bin or the harness deadline fires.
//!
//! The state machine mirrors `SearchAndPlace` (sweep with peak-tracking →
//! descend → close → ascend → yaw → open). The differences:
//!   - On `OpenGripper` completion, instead of transitioning to `Done`,
//!     reset sweep state (idx=0, peak=0, contact_xy=None) and re-enter
//!     `Sweeping`. `Done` only fires once `placed_count >= target_count`.
//!   - If the sweep exhausts without contact, instead of terminating,
//!     wrap back to `Sweeping` with idx=0. The deadline catches a true
//!     failure; this means the controller waits patiently for the next
//!     block to spawn.
//!
//! Run a single demo (records to $TMPDIR/continuous_spawn.rrd if
//! --features viz-rerun):
//!   `cargo run --example continuous_spawn --features examples`
//!   `cargo run --example continuous_spawn --features viz-rerun`
//!
//! Run as a test (records to $TMPDIR/<test_name>.rrd if --features viz-rerun):
//!   `cargo test --example continuous_spawn --features examples`

use nalgebra::{Isometry3, Vector3};
use rtf_arm::{
    ik::ik_3r,
    ports::{
        EePoseReading, GripperCommand, JointEncoderReading, JointId, JointVelocityCommand,
        PressureReading,
    },
    spec::{ArmSpec, GripperSpec, JointSpec},
    test_helpers::{BIN_FIXTURE_ID, SEARCH_REGION_X, SEARCH_REGION_Y},
    world::ArmWorld,
    RateHz,
};
use rtf_core::{
    controller::{ControlError, Controller},
    goal::Goal,
    noise_source::NoiseSource,
    port::{PortReader, PortTx},
    score::Score,
    time::{Duration, Time},
};
#[cfg(test)]
use rtf_harness::Termination;
use rtf_harness::{run, RunConfig};
use rtf_sim::{
    faults::PcgNoiseSource,
    fixture::Fixture,
    object::{Object, ObjectId, ObjectState, SupportId},
    scene::Scene,
    shape::Shape,
};

// -- Controller ----------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CSState {
    Sweeping,
    DescendToContact,
    CloseGripper(u32),
    AscendWithBlock,
    YawToBin,
    OpenGripper(u32),
    Done,
}

const GRASP_Z: f32 = 0.55;
const PARK_Z: f32 = 0.85;
const CLOSE_HOLD_TICKS: u32 = 200;
const OPEN_HOLD_TICKS: u32 = 200;
/// Phase 3.4.5c: cumulative chain pitch the controller drives (J1+J2+J3)
/// to keep EE +x = world -z (fingers protruding straight down).
const TARGET_WRIST_PITCH: f32 = core::f32::consts::FRAC_PI_2;

#[allow(dead_code)]
pub struct ContinuousSearchAndPlace<R, P, Pr>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
    Pr: PortReader<PressureReading>,
{
    state: CSState,
    sweep_waypoints: Vec<(f32, f32, f32, f32)>,
    sweep_idx: usize,
    peak_pressure: f32,
    peak_xy: Option<(f32, f32)>,
    contact_xy: Option<(f32, f32)>,
    target_bin_xy: (f32, f32),
    pressure_threshold: f32,
    arm_shoulder_z: f32,
    l1: f32,
    l2: f32,
    /// Phase 3.4.5b: wrist link length, fed to ik_3r so J3 keeps EE +x =
    /// world -z (fingers pointing down).
    l3: f32,
    placed_count: u32,
    target_count: u32,
    encoder_rxs: Vec<R>,
    ee_pose_rx: P,
    pressure_rx: Pr,
    velocity_txs: Vec<PortTx<JointVelocityCommand>>,
    gripper_tx: PortTx<GripperCommand>,
    joint_tol: f32,
}

impl<R, P, Pr> ContinuousSearchAndPlace<R, P, Pr>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
    Pr: PortReader<PressureReading>,
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        encoder_rxs: Vec<R>,
        ee_pose_rx: P,
        pressure_rx: Pr,
        velocity_txs: Vec<PortTx<JointVelocityCommand>>,
        gripper_tx: PortTx<GripperCommand>,
        region_x: (f32, f32),
        region_y: (f32, f32),
        sweep_z: f32,
        stripe_dy: f32,
        target_bin_xy: (f32, f32),
        arm_shoulder_z: f32,
        l1: f32,
        l2: f32,
        l3: f32,
        target_count: u32,
    ) -> Self {
        let xy_waypoints = serpentine_waypoints(region_x, region_y, stripe_dy);
        let sweep_waypoints: Vec<(f32, f32, f32, f32)> = xy_waypoints
            .iter()
            .map(|&(x, y)| {
                let r = (x * x + y * y).sqrt();
                let (j1, j2, j3) =
                    ik_3r(r, sweep_z - arm_shoulder_z, TARGET_WRIST_PITCH, l1, l2, l3)
                        .expect("sweep waypoint unreachable — check region vs arm reach");
                let yaw = y.atan2(x);
                (yaw, j1, j2, j3)
            })
            .collect();
        Self {
            state: CSState::Sweeping,
            sweep_waypoints,
            sweep_idx: 0,
            peak_pressure: 0.0,
            peak_xy: None,
            contact_xy: None,
            target_bin_xy,
            pressure_threshold: 0.2,
            arm_shoulder_z,
            l1,
            l2,
            l3,
            placed_count: 0,
            target_count,
            encoder_rxs,
            ee_pose_rx,
            pressure_rx,
            velocity_txs,
            gripper_tx,
            joint_tol: 0.02,
        }
        // pressure_threshold remains 0.2 above; the second-pass refinement
        // happens via `min_commit_peak` checked at trigger time below.
    }

    /// Minimum peak pressure required to actually commit to descent.
    /// Lowered from the v1 0.5 to 0.2 in Step 1.13: under Rapier the arm
    /// physically pushes blocks during sweep so peaks degrade by the
    /// time we trigger; the wider 0.10 m gripper proximity (Step 1.12 in
    /// build_continuous_spawn_world) absorbs the corresponding xy error.
    const MIN_COMMIT_PEAK: f32 = 0.2;

    pub fn state(&self) -> CSState {
        self.state
    }

    pub fn placed_count(&self) -> u32 {
        self.placed_count
    }

    fn joint_qs(&self) -> Vec<f32> {
        self.encoder_rxs
            .iter()
            .map(|rx| rx.latest().map(|r| r.q).unwrap_or(0.0))
            .collect()
    }

    fn ee_xy(&self) -> Option<(f32, f32)> {
        let r = self.ee_pose_rx.latest()?;
        Some((r.pose.translation.x, r.pose.translation.y))
    }

    fn drive_joints_toward(&mut self, target: (f32, f32, f32, f32)) {
        let qs = self.joint_qs();
        for (i, t) in [
            (0usize, target.0),
            (1, target.1),
            (2, target.2),
            (3, target.3),
        ] {
            if i >= self.velocity_txs.len() {
                break;
            }
            let cur = qs.get(i).copied().unwrap_or(0.0);
            let q_dot = ((t - cur) * 4.0).clamp(-2.0, 2.0);
            self.velocity_txs[i].send(JointVelocityCommand {
                joint: JointId(i as u32),
                q_dot_target: q_dot,
            });
        }
    }

    fn halt_joints(&mut self) {
        for (i, tx) in self.velocity_txs.iter().enumerate() {
            tx.send(JointVelocityCommand {
                joint: JointId(i as u32),
                q_dot_target: 0.0,
            });
        }
    }

    fn joints_converged_to(&self, target: (f32, f32, f32, f32)) -> bool {
        let qs = self.joint_qs();
        let q0 = qs.first().copied().unwrap_or(0.0);
        let q1 = qs.get(1).copied().unwrap_or(0.0);
        let q2 = qs.get(2).copied().unwrap_or(0.0);
        let q3 = qs.get(3).copied().unwrap_or(0.0);
        (q0 - target.0).abs() < self.joint_tol
            && (q1 - target.1).abs() < self.joint_tol
            && (q2 - target.2).abs() < self.joint_tol
            && (q3 - target.3).abs() < self.joint_tol
    }

    fn ik_target_for(&self, xy: (f32, f32), z: f32) -> (f32, f32, f32, f32) {
        let (x, y) = xy;
        let r = (x * x + y * y).sqrt();
        let (j1, j2, j3) = ik_3r(
            r,
            z - self.arm_shoulder_z,
            TARGET_WRIST_PITCH,
            self.l1,
            self.l2,
            self.l3,
        )
        .expect("post-contact IK target unreachable — check geometry");
        (y.atan2(x), j1, j2, j3)
    }

    /// Reset the sweep state for the next cycle. Called on
    /// OpenGripper-completion (when not yet `Done`) and on
    /// sweep-exhausted-without-contact.
    fn reset_sweep(&mut self) {
        self.sweep_idx = 0;
        self.peak_pressure = 0.0;
        self.peak_xy = None;
        self.contact_xy = None;
    }
}

impl<R, P, Pr> Controller for ContinuousSearchAndPlace<R, P, Pr>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
    Pr: PortReader<PressureReading>,
{
    fn step(&mut self, _t: Time) -> Result<(), ControlError> {
        let pressure = self.pressure_rx.latest().map(|r| r.pressure).unwrap_or(0.0);

        match self.state {
            CSState::Sweeping => {
                if pressure > self.peak_pressure {
                    self.peak_pressure = pressure;
                    if let Some(xy) = self.ee_xy() {
                        self.peak_xy = Some(xy);
                    }
                }

                // Wait until the peak settles (pressure has fallen back
                // below threshold) AND the peak was strong enough to
                // trust as a real grasp candidate. Weak grazes (peak
                // 0.2-0.5) leave the recorded peak xy too far from the
                // block centre for the gripper's 5 cm proximity to bite.
                if self.peak_pressure >= Self::MIN_COMMIT_PEAK && pressure < self.pressure_threshold
                {
                    self.contact_xy = self.peak_xy;
                    self.state = CSState::DescendToContact;
                    self.halt_joints();
                    return Ok(());
                }

                let target = self.sweep_waypoints[self.sweep_idx];
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.sweep_idx += 1;
                    if self.sweep_idx >= self.sweep_waypoints.len() {
                        self.reset_sweep();
                    }
                }
            }
            CSState::DescendToContact => {
                let cxy = self.contact_xy.expect("contact_xy set on entry");
                let target = self.ik_target_for(cxy, GRASP_Z);
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.state = CSState::CloseGripper(0);
                }
            }
            CSState::CloseGripper(n) => {
                self.gripper_tx.send(GripperCommand {
                    target_separation: 0.012,
                });
                self.halt_joints();
                if n >= CLOSE_HOLD_TICKS {
                    // Verify a block was actually grasped before
                    // committing to the place sequence: if the EE
                    // pressure has fallen back to ~0 then the block
                    // wasn't there (likely pushed away during sweep).
                    // Skip the place and retry sweep. Threshold 0.5
                    // matches MIN_COMMIT_PEAK so any meaningful held
                    // contact qualifies.
                    if pressure < 0.5 {
                        self.gripper_tx.send(GripperCommand {
                            target_separation: 0.04,
                        });
                        self.reset_sweep();
                        self.state = CSState::Sweeping;
                    } else {
                        self.state = CSState::AscendWithBlock;
                    }
                } else {
                    self.state = CSState::CloseGripper(n + 1);
                }
            }
            CSState::AscendWithBlock => {
                let cxy = self.contact_xy.expect("contact_xy persists after grasp");
                let target = self.ik_target_for(cxy, PARK_Z);
                self.gripper_tx.send(GripperCommand {
                    target_separation: 0.012,
                });
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.state = CSState::YawToBin;
                }
            }
            CSState::YawToBin => {
                let target = self.ik_target_for(self.target_bin_xy, PARK_Z);
                self.gripper_tx.send(GripperCommand {
                    target_separation: 0.012,
                });
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.state = CSState::OpenGripper(0);
                }
            }
            CSState::OpenGripper(n) => {
                self.gripper_tx.send(GripperCommand {
                    target_separation: 0.04,
                });
                self.halt_joints();
                if n >= OPEN_HOLD_TICKS {
                    self.placed_count += 1;
                    self.state = if self.placed_count >= self.target_count {
                        CSState::Done
                    } else {
                        self.reset_sweep();
                        CSState::Sweeping
                    };
                } else {
                    self.state = CSState::OpenGripper(n + 1);
                }
            }
            CSState::Done => {
                self.halt_joints();
            }
        }
        Ok(())
    }
}

// -- Sweep waypoint helper (mirrors find_grasp_place's) -----------------

fn serpentine_waypoints(
    region_x: (f32, f32),
    region_y: (f32, f32),
    stripe_dy: f32,
) -> Vec<(f32, f32)> {
    let (x_min, x_max) = region_x;
    let (y_min, y_max) = region_y;
    let span = (y_max - y_min).max(0.0);
    let n_stripes = (span / stripe_dy).round() as usize + 1;
    let mut out = Vec::with_capacity(n_stripes * 2);
    for i in 0..n_stripes {
        let y = (y_min + (i as f32) * stripe_dy).min(y_max);
        let going_right = i % 2 == 0;
        if going_right {
            out.push((x_min, y));
            out.push((x_max, y));
        } else {
            out.push((x_max, y));
            out.push((x_min, y));
        }
    }
    out
}

// -- World setup --------------------------------------------------------

const BLOCK_HALF_HEIGHT: f32 = 0.025;
const TABLE_TOP_Z: f32 = 0.5;

/// Build the continuous-spawn world. Same arm + table + bin as the
/// pick-and-place / find-grasp-place worlds, but no initial blocks. A
/// `PcgNoiseSource` seeded with `seed` is used to draw `n_spawns` random
/// xy positions in the search region; spawn `i` is scheduled at sim time
/// `i * interval`.
fn build_continuous_spawn_world(seed: u64, n_spawns: u32, interval: Duration) -> ArmWorld {
    use core::f32::consts::PI;
    let mut src = PcgNoiseSource::from_seed(seed);
    let mut scene = Scene::with_ground(0);

    // Table fixture (id 0).
    scene.add_fixture(Fixture {
        id: 0,
        pose: Isometry3::translation(0.5, 0.0, 0.475),
        shape: Shape::Aabb {
            half_extents: Vector3::new(0.4, 0.4, 0.025),
        },
        is_support: true,
    });

    // Bin fixture.
    scene.add_fixture(Fixture {
        id: BIN_FIXTURE_ID,
        pose: Isometry3::translation(0.0, 0.6, 0.55),
        shape: Shape::Aabb {
            half_extents: Vector3::new(0.1, 0.1, 0.05),
        },
        is_support: true,
    });

    // Phase 3.4.5b: same Z-Y-Y-Y geometry as build_pick_and_place_world
    // (3 pitch + 1 wrist) so the controller can keep EE +x = world -z.
    let spec = ArmSpec {
        joints: vec![
            JointSpec::Revolute {
                axis: Vector3::z_axis(),
                limits: (-PI, PI),
            },
            JointSpec::Revolute {
                axis: Vector3::y_axis(),
                limits: (-PI, PI),
            },
            JointSpec::Revolute {
                axis: Vector3::y_axis(),
                limits: (-PI, PI),
            },
            JointSpec::Revolute {
                axis: Vector3::y_axis(),
                limits: (-PI, PI),
            }, // J3 wrist pitch (Phase 3.4.5b)
        ],
        link_offsets: vec![
            Isometry3::translation(0.0, 0.0, 0.8),
            Isometry3::translation(0.4, 0.0, 0.0),
            Isometry3::translation(0.4, 0.0, 0.0),
            Isometry3::translation(0.05, 0.0, 0.0), // wrist (Phase 3.4.5b)
        ],
        gripper: GripperSpec {
            // Wider than the pick-and-place threshold (0.05) for the same
            // reason build_search_world bumps it to 0.10 (Step 1.12): an
            // arm-link-pushed block lands a few cm from the EE descent
            // target, and the grasp logic needs the slack.
            proximity_threshold: 0.10,
            max_grasp_size: 0.1,
        },
    };

    let mut world = ArmWorld::new(scene, spec, /* gravity */ true);

    let block_z = TABLE_TOP_Z + BLOCK_HALF_HEIGHT;
    for i in 0..n_spawns {
        let x = SEARCH_REGION_X.0 + src.uniform_unit() * (SEARCH_REGION_X.1 - SEARCH_REGION_X.0);
        let y = SEARCH_REGION_Y.0 + src.uniform_unit() * (SEARCH_REGION_Y.1 - SEARCH_REGION_Y.0);
        let template = Object {
            id: ObjectId(0), // overwritten by world.schedule_spawn
            pose: Isometry3::translation(x, y, block_z),
            shape: Shape::Aabb {
                half_extents: Vector3::new(0.025, 0.025, BLOCK_HALF_HEIGHT),
            },
            mass: 0.1,
            graspable: true,
            state: ObjectState::Settled {
                on: SupportId::Fixture(0),
            },
            friction: 2.0,
            lin_vel: Vector3::zeros(),
        };
        let at = Time::from_nanos((i as i64) * interval.as_nanos());
        world.schedule_spawn(at, template);
    }
    world
}

// -- Goal ---------------------------------------------------------------

/// Goal for the continuous-spawn scenario: count Settled Objects whose
/// xy lies inside the bin's xy footprint at-or-above the bin top z.
/// Complete when the count reaches `target_count`. Score is
/// `count / target_count`, clamped to [0, 1).
///
/// Pre-Rapier (v1) walked the SupportId chain to count stacked blocks
/// transitively. Rapier doesn't track support chains (Settled.on is
/// SupportId::Unknown), so the check is now pose-based: stacked blocks
/// just register as "their xy is inside the bin's xy footprint".
struct NObjectsInBin {
    bin: u32,
    target_count: u32,
}

impl NObjectsInBin {
    /// Bin xy center, xy half-extents, and top z. Returns None if no
    /// such fixture or its shape is non-Aabb.
    fn bin_footprint(&self, world: &ArmWorld) -> Option<((f32, f32), nalgebra::Vector3<f32>, f32)> {
        let (_, fix) = world.scene.fixtures().find(|(id, _)| **id == self.bin)?;
        let half = match fix.shape {
            rtf_sim::shape::Shape::Aabb { half_extents } => half_extents,
            _ => return None,
        };
        let center = (fix.pose.translation.x, fix.pose.translation.y);
        let top_z = fix.pose.translation.z + half.z;
        Some((center, half, top_z))
    }

    /// True iff `obj` is Settled and its xy is inside the bin footprint
    /// AND its z is at-or-above the bin top (within a 5 cm slop so a
    /// stack-of-blocks all count).
    fn pose_in_bin(
        &self,
        obj: &rtf_sim::object::Object,
        center: (f32, f32),
        half: nalgebra::Vector3<f32>,
        top_z: f32,
    ) -> bool {
        if !matches!(obj.state, ObjectState::Settled { .. }) {
            return false;
        }
        let dx = (obj.pose.translation.x - center.0).abs();
        let dy = (obj.pose.translation.y - center.1).abs();
        dx <= half.x && dy <= half.y && obj.pose.translation.z >= top_z - 0.05
    }

    fn count_in_bin(&self, world: &ArmWorld) -> u32 {
        let Some((center, half, top_z)) = self.bin_footprint(world) else {
            return 0;
        };
        world
            .scene
            .objects()
            .filter(|(_, o)| self.pose_in_bin(o, center, half, top_z))
            .count() as u32
    }
}

impl Goal<ArmWorld> for NObjectsInBin {
    fn is_complete(&self, world: &ArmWorld) -> bool {
        self.count_in_bin(world) >= self.target_count
    }

    fn evaluate(&self, world: &ArmWorld) -> Score {
        let count = self.count_in_bin(world) as f64;
        let target = self.target_count as f64;
        // Clamp slightly under 1.0 in the partial-progress case so
        // is_complete remains the authoritative GoalComplete signal.
        Score::new((count / target).clamp(0.0, 0.99))
    }
}

// -- Runner -------------------------------------------------------------

const N_SPAWNS: u32 = 3;
const SPAWN_INTERVAL_SECS: i64 = 15;
// Bumped from 60 → 90 s after Step 1.13: with Rapier physics each
// pick-place cycle takes longer (the arm pushes blocks during sweep,
// adding lift+approach overhead) and the third block needs settle time
// after release before NObjectsInBin counts it.
const DEADLINE_SECS: i64 = 90;

fn run_continuous_spawn(seed: u64, rrd_name: &str) -> rtf_harness::RunResult {
    run_continuous_spawn_with(seed, rrd_name, /* debug_overlay */ false)
}

fn run_continuous_spawn_with(
    seed: u64,
    rrd_name: &str,
    debug_overlay: bool,
) -> rtf_harness::RunResult {
    let _ = rrd_name; // used only when viz-rerun is on
    let mut world =
        build_continuous_spawn_world(seed, N_SPAWNS, Duration::from_secs(SPAWN_INTERVAL_SECS));
    if debug_overlay {
        world.enable_debug_overlay(true);
    }
    let ports = world.attach_standard_arm_ports();
    let ee_pose_rx = world.attach_ee_pose_sensor(RateHz::new(100));
    let pressure_rx = world.attach_pressure_sensor(RateHz::new(1000), 0.03);

    let controller = ContinuousSearchAndPlace::new(
        ports.encoder_rxs,
        ee_pose_rx,
        pressure_rx,
        ports.velocity_txs,
        ports.gripper_tx,
        SEARCH_REGION_X,
        SEARCH_REGION_Y,
        /* sweep_z */ 0.57,
        // Tighter than find_grasp_place's 0.05 — finer sampling improves
        // peak-xy resolution so post-trigger contact_xy stays inside the
        // gripper proximity threshold. 0.03 keeps worst-case offset
        // under ~3 cm while still covering the search region quickly.
        /* stripe_dy */
        0.03,
        /* target_bin_xy */ (0.0, 0.6),
        /* arm_shoulder_z */ 0.8,
        /* l1 */ 0.4,
        /* l2 */ 0.4,
        /* l3 (wrist link, Phase 3.4.5b) */ 0.05,
        N_SPAWNS,
    );
    let goal = NObjectsInBin {
        bin: BIN_FIXTURE_ID,
        target_count: N_SPAWNS,
    };
    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(DEADLINE_SECS))
        .with_seed(seed);

    #[cfg(feature = "viz-rerun")]
    {
        if std::env::var("RTF_DEBUG_OVERLAY").as_deref() == Ok("1") {
            world.enable_debug_overlay(true);
        }
        match rtf_viz::maybe_recorder_for(rrd_name) {
            Some(rec) => run(world, controller, goal, cfg.with_recorder(rec)),
            None => run(world, controller, goal, cfg),
        }
    }
    #[cfg(not(feature = "viz-rerun"))]
    {
        run(world, controller, goal, cfg)
    }
}

fn main() {
    let seed = 42_u64;
    let res = run_continuous_spawn(seed, "continuous_spawn");
    println!(
        "continuous_spawn(seed={seed}): terminated_by={:?}, final_time={:?}, score={}",
        res.terminated_by, res.final_time, res.score.value,
    );
}

// -- Tests --------------------------------------------------------------

#[test]
fn continuous_spawn_seed_1() {
    let seed = 1_u64;
    let res = run_continuous_spawn(seed, "continuous_spawn_seed_1");
    eprintln!(
        "seed={seed}: terminated_by={:?}, final_time={:?}, score={}",
        res.terminated_by, res.final_time, res.score.value,
    );
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "seed {seed} did not converge in {DEADLINE_SECS}s; terminated_by={:?}, score={}",
        res.terminated_by,
        res.score.value,
    );
    assert!(res.score.value >= 0.99);
}

#[test]
fn continuous_spawn_seed_42() {
    let seed = 42_u64;
    let res = run_continuous_spawn(seed, "continuous_spawn_seed_42");
    eprintln!(
        "seed={seed}: terminated_by={:?}, final_time={:?}, score={}",
        res.terminated_by, res.final_time, res.score.value,
    );
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "seed {seed} did not converge in {DEADLINE_SECS}s; terminated_by={:?}, score={}",
        res.terminated_by,
        res.score.value,
    );
    assert!(res.score.value >= 0.99);
}

#[test]
fn continuous_spawn_seed_1337() {
    let seed = 1337_u64;
    let res = run_continuous_spawn(seed, "continuous_spawn_seed_1337");
    eprintln!(
        "seed={seed}: terminated_by={:?}, final_time={:?}, score={}",
        res.terminated_by, res.final_time, res.score.value,
    );
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "seed {seed} did not converge in {DEADLINE_SECS}s; terminated_by={:?}, score={}",
        res.terminated_by,
        res.score.value,
    );
    assert!(res.score.value >= 0.99);
}

/// Sanity-check: the Rapier debug overlay doesn't break continuous-spawn.
#[test]
fn continuous_spawn_seed_42_with_debug_overlay() {
    let seed = 42_u64;
    let res = run_continuous_spawn_with(seed, "continuous_spawn_seed_42_overlay", true);
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "seed {seed} (overlay on) did not converge; terminated_by={:?}, score={}",
        res.terminated_by,
        res.score.value,
    );
    assert!(res.score.value >= 0.99);
}

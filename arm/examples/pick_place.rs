//! Headline acceptance scenario for v1: a Z-Y-Y arm yaws over a block,
//! descends, grasps, ascends, swings to a bin, and releases.
//!
//! Run terminal-only:
//!   `cargo run --example pick_place --features examples`
//!
//! Run as a test:
//!   `cargo test --example pick_place --features examples`
//!
//! Save a rerun .rrd for visual inspection (requires the `viz-rerun` feature):
//!   `cargo test --example pick_place --features viz-rerun \
//!      pick_place_save_rrd -- --nocapture`
//! Then view with `rerun <printed-path>` (install once via
//! `cargo install rerun-cli --version 0.21`).

use rtf_arm::{
    goals::PlaceInBin,
    ik::ik_2r,
    ports::{EePoseReading, GripperCommand, JointEncoderReading, JointId, JointVelocityCommand},
    test_helpers::{bin_id, block_id, build_pick_and_place_world},
    RateHz,
};
use rtf_core::{
    controller::{ControlError, Controller},
    port::{PortReader, PortTx},
    time::{Duration, Time},
};
#[cfg(test)]
use rtf_harness::Termination;
use rtf_harness::{run, RunConfig};

// -- Controller ----------------------------------------------------------

/// Pick-and-place state-machine controller for a Z-Y-Y arm. Yaws J0 over the
/// block, descends J1+J2 via 2R IK, closes the gripper, ascends, swings J0
/// over the bin, and releases. Pre-computes IK targets at construction —
/// unreachable geometry panics in `new()` (programmer error in fixture).
pub struct PickPlace<R, P>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
{
    state: PickPlaceState,
    target_block_xy: (f32, f32),
    target_bin_xy: (f32, f32),
    encoder_rxs: Vec<R>,
    ee_pose_rx: P,
    velocity_txs: Vec<PortTx<JointVelocityCommand>>,
    gripper_tx: PortTx<GripperCommand>,
    /// Per-joint convergence threshold (radians). All three joints must be
    /// within this of their respective targets to advance state.
    joint_tol: f32,
    /// IK solutions (J1, J2) for the three keyframe poses.
    ik_above_block: (f32, f32),
    ik_at_block: (f32, f32),
    ik_above_bin: (f32, f32),
    close_hold_ticks: u32,
    open_hold_ticks: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickPlaceState {
    /// J0 swings to align with block xy; J1, J2 driven to 0 (extended).
    ApproachYaw,
    /// J1, J2 driven to `ik_at_block` so EE drops onto the block.
    DescendOverBlock,
    /// Gripper closes; all joints held still.
    CloseGripper(u32),
    /// J1, J2 driven back to `ik_above_block`, lifting the grasped block.
    AscendWithBlock,
    /// J0 swings to align with bin xy; J1, J2 driven to `ik_above_bin`.
    YawToBin,
    /// Gripper opens; all joints held still.
    OpenGripper(u32),
    Done,
}

impl<R, P> PickPlace<R, P>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
{
    /// Construct a PickPlace controller. Pre-computes IK for three keyframe
    /// EE poses (above_block, at_block, above_bin) where above_* targets sit
    /// at z=0.85 and at_block sits 0.025 m above the block top. Panics if
    /// any IK solve fails — the test fixture is supposed to be reachable.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        encoder_rxs: Vec<R>,
        ee_pose_rx: P,
        velocity_txs: Vec<PortTx<JointVelocityCommand>>,
        gripper_tx: PortTx<GripperCommand>,
        target_block_xy: (f32, f32),
        target_bin_xy: (f32, f32),
        arm_shoulder_z: f32,
        l1: f32,
        l2: f32,
    ) -> Self {
        let block_r =
            (target_block_xy.0 * target_block_xy.0 + target_block_xy.1 * target_block_xy.1).sqrt();
        let bin_r = (target_bin_xy.0 * target_bin_xy.0 + target_bin_xy.1 * target_bin_xy.1).sqrt();
        // Block top at z=0.55 (block center 0.525 + half-extent 0.025).
        // Park EE 0.85m up between maneuvers — safely above the bin top
        // (z=0.6) so the released block can fall into the bin.
        let block_grasp_z = 0.55_f32;
        let park_z = 0.85_f32;
        let ik_at_block = ik_2r(block_r, block_grasp_z - arm_shoulder_z, l1, l2)
            .expect("at_block target unreachable — check arm geometry vs block xy/z");
        let ik_above_block = ik_2r(block_r, park_z - arm_shoulder_z, l1, l2)
            .expect("above_block target unreachable — check arm geometry vs block xy");
        let ik_above_bin = ik_2r(bin_r, park_z - arm_shoulder_z, l1, l2)
            .expect("above_bin target unreachable — check arm geometry vs bin xy");
        Self {
            state: PickPlaceState::ApproachYaw,
            target_block_xy,
            target_bin_xy,
            encoder_rxs,
            ee_pose_rx,
            velocity_txs,
            gripper_tx,
            joint_tol: 0.02,
            ik_above_block,
            ik_at_block,
            ik_above_bin,
            close_hold_ticks: 200,
            open_hold_ticks: 200,
        }
    }

    pub fn state(&self) -> PickPlaceState {
        self.state
    }

    /// Read all joint encoders into a vector; entries default to 0.0 if a
    /// publisher hasn't fired yet (only relevant in tests).
    fn joint_qs(&self) -> Vec<f32> {
        self.encoder_rxs
            .iter()
            .map(|rx| rx.latest().map(|r| r.q).unwrap_or(0.0))
            .collect()
    }

    /// Yaw-error from current EE xy heading to a target xy heading, wrapped
    /// to `[-π, π]`. None if EE pose hasn't published yet.
    fn yaw_error(&self, target: (f32, f32)) -> Option<f32> {
        use core::f32::consts::PI;
        let r = self.ee_pose_rx.latest()?;
        let ee_xy = (r.pose.translation.x, r.pose.translation.y);
        let target_heading = target.1.atan2(target.0);
        let ee_heading = ee_xy.1.atan2(ee_xy.0);
        let error = target_heading - ee_heading;
        Some(((error + PI).rem_euclid(2.0 * PI)) - PI)
    }

    /// Send a per-joint velocity command computed via P-control on the
    /// position error toward `targets`. Joint 0's "target" is interpreted
    /// as a yaw error (radians, already wrapped); joints 1+ as absolute
    /// position targets.
    fn drive_joints(&mut self, j0_yaw_err: f32, j1_target: f32, j2_target: f32) {
        let qs = self.joint_qs();
        // J0: position-equivalent error is the yaw_err itself.
        let q_dot_0 = (j0_yaw_err * 4.0).clamp(-2.0, 2.0);
        self.velocity_txs[0].send(JointVelocityCommand {
            joint: JointId(0),
            q_dot_target: q_dot_0,
        });
        for (i, target) in [(1usize, j1_target), (2usize, j2_target)] {
            if i >= self.velocity_txs.len() {
                break;
            }
            let cur = qs.get(i).copied().unwrap_or(0.0);
            let q_dot = ((target - cur) * 4.0).clamp(-2.0, 2.0);
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

    /// True iff `|q[1] - j1_target| < tol AND |q[2] - j2_target| < tol`.
    fn pitch_converged(&self, j1_target: f32, j2_target: f32) -> bool {
        let qs = self.joint_qs();
        let j1_ok = qs
            .get(1)
            .is_some_and(|q| (q - j1_target).abs() < self.joint_tol);
        let j2_ok = qs
            .get(2)
            .is_some_and(|q| (q - j2_target).abs() < self.joint_tol);
        j1_ok && j2_ok
    }
}

impl<R, P> Controller for PickPlace<R, P>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
{
    fn step(&mut self, _t: Time) -> Result<(), ControlError> {
        let Some(yaw_err_block) = self.yaw_error(self.target_block_xy) else {
            return Ok(());
        };
        let yaw_err_bin = self.yaw_error(self.target_bin_xy).unwrap_or(0.0);
        match self.state {
            PickPlaceState::ApproachYaw => {
                self.drive_joints(yaw_err_block, 0.0, 0.0);
                if yaw_err_block.abs() < self.joint_tol && self.pitch_converged(0.0, 0.0) {
                    self.state = PickPlaceState::DescendOverBlock;
                }
            }
            PickPlaceState::DescendOverBlock => {
                self.drive_joints(yaw_err_block, self.ik_at_block.0, self.ik_at_block.1);
                if self.pitch_converged(self.ik_at_block.0, self.ik_at_block.1) {
                    self.state = PickPlaceState::CloseGripper(0);
                }
            }
            PickPlaceState::CloseGripper(n) => {
                self.gripper_tx.send(GripperCommand { close: true });
                self.halt_joints();
                self.state = if n >= self.close_hold_ticks {
                    PickPlaceState::AscendWithBlock
                } else {
                    PickPlaceState::CloseGripper(n + 1)
                };
            }
            PickPlaceState::AscendWithBlock => {
                self.gripper_tx.send(GripperCommand { close: true });
                self.drive_joints(yaw_err_block, self.ik_above_block.0, self.ik_above_block.1);
                if self.pitch_converged(self.ik_above_block.0, self.ik_above_block.1) {
                    self.state = PickPlaceState::YawToBin;
                }
            }
            PickPlaceState::YawToBin => {
                self.gripper_tx.send(GripperCommand { close: true });
                self.drive_joints(yaw_err_bin, self.ik_above_bin.0, self.ik_above_bin.1);
                if yaw_err_bin.abs() < self.joint_tol
                    && self.pitch_converged(self.ik_above_bin.0, self.ik_above_bin.1)
                {
                    self.state = PickPlaceState::OpenGripper(0);
                }
            }
            PickPlaceState::OpenGripper(n) => {
                self.gripper_tx.send(GripperCommand { close: false });
                self.halt_joints();
                self.state = if n >= self.open_hold_ticks {
                    PickPlaceState::Done
                } else {
                    PickPlaceState::OpenGripper(n + 1)
                };
            }
            PickPlaceState::Done => {}
        }
        Ok(())
    }
}

// -- Runner --------------------------------------------------------------

fn run_pick_place_default() -> rtf_harness::RunResult {
    let mut world = build_pick_and_place_world();
    let ports = world.attach_standard_arm_ports();
    let ee_pose_rx = world.attach_ee_pose_sensor(RateHz::new(100));
    let block = block_id(&world);
    let bin = bin_id(&world);

    let controller = PickPlace::new(
        ports.encoder_rxs,
        ee_pose_rx,
        ports.velocity_txs,
        ports.gripper_tx,
        /* target_block_xy */ (0.6, 0.0),
        /* target_bin_xy */ (0.0, 0.6),
        /* arm_shoulder_z */ 0.8,
        /* l1 */ 0.4,
        /* l2 */ 0.4,
    );
    let goal = PlaceInBin::new(block, bin);

    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(15))
        .with_seed(42);

    run(world, controller, goal, cfg)
}

fn main() {
    let res = run_pick_place_default();
    println!(
        "PickPlace: terminated_by={:?}, final_time={:?}, score={}",
        res.terminated_by, res.final_time, res.score.value,
    );
}

// -- Tests ---------------------------------------------------------------

#[test]
fn state_machine_picks_block_and_drops_in_bin() {
    let res = run_pick_place_default();
    eprintln!(
        "e2e PickPlace: terminated_by={:?}, final_time={:?}, score={}",
        res.terminated_by, res.final_time, res.score.value,
    );
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "did not converge in 15s; final score={}, terminated_by={:?}",
        res.score.value,
        res.terminated_by,
    );
    assert!(res.score.value > 0.9);
}

#[cfg(feature = "viz-rerun")]
#[test]
fn state_machine_picks_block_and_drops_in_bin_with_rerun() {
    use rtf_viz::RerunRecorder;

    let rec = RerunRecorder::in_memory("pick_place").unwrap();

    let mut world = build_pick_and_place_world();
    let ports = world.attach_standard_arm_ports();
    let ee_pose_rx = world.attach_ee_pose_sensor(RateHz::new(100));
    let block = block_id(&world);
    let bin = bin_id(&world);

    let controller = PickPlace::new(
        ports.encoder_rxs,
        ee_pose_rx,
        ports.velocity_txs,
        ports.gripper_tx,
        (0.6, 0.0),
        (0.0, 0.6),
        0.8,
        0.4,
        0.4,
    );
    let goal = PlaceInBin::new(block, bin);

    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(15))
        .with_seed(42)
        .with_recorder(rec);

    let res = run(world, controller, goal, cfg);
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "did not converge with rerun recorder; final score = {}",
        res.score.value,
    );
}

#[cfg(feature = "viz-rerun")]
#[test]
fn pick_place_save_rrd() {
    use rtf_viz::RerunRecorder;

    let path = std::env::temp_dir().join("pick_place.rrd");
    let _ = std::fs::remove_file(&path);
    let rec = RerunRecorder::save_to_file(&path, "pick_place").unwrap();

    let mut world = build_pick_and_place_world();
    let ports = world.attach_standard_arm_ports();
    let ee_pose_rx = world.attach_ee_pose_sensor(RateHz::new(100));
    let block = block_id(&world);
    let bin = bin_id(&world);

    let controller = PickPlace::new(
        ports.encoder_rxs,
        ee_pose_rx,
        ports.velocity_txs,
        ports.gripper_tx,
        (0.6, 0.0),
        (0.0, 0.6),
        0.8,
        0.4,
        0.4,
    );
    let goal = PlaceInBin::new(block, bin);

    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(15))
        .with_seed(42)
        .with_recorder(rec);

    let res = run(world, controller, goal, cfg);
    eprintln!("Saved rrd file to: {:?}", path);
    eprintln!("View with: rerun {:?}", path);
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "did not converge while saving rrd; final score = {}",
        res.score.value,
    );
}

#[cfg(test)]
mod pick_place_tests {
    use super::*;
    use rtf_core::port::{port, PortRx, PortTx};

    /// Test rig: 3 encoder pubs, 1 EE pose pub, 3 velocity-cmd subs, 1
    /// gripper-cmd sub, plus the controller. Block at (0.6, 0), bin at
    /// (0, 0.6); shoulder z=0.8, l1=l2=0.4 (matches the test fixture).
    struct Rig {
        c: PickPlace<PortRx<JointEncoderReading>, PortRx<EePoseReading>>,
        enc_txs: Vec<PortTx<JointEncoderReading>>,
        ee_tx: PortTx<EePoseReading>,
        _vel_rxs: Vec<PortRx<JointVelocityCommand>>,
        _g_rx: PortRx<GripperCommand>,
    }

    fn make_rig() -> Rig {
        let mut enc_rxs = Vec::new();
        let mut enc_txs = Vec::new();
        for _ in 0..3 {
            let (tx, rx) = port::<JointEncoderReading>();
            enc_rxs.push(rx);
            enc_txs.push(tx);
        }
        let (ee_tx, ee_rx) = port::<EePoseReading>();
        let mut vel_txs = Vec::new();
        let mut vel_rxs = Vec::new();
        for _ in 0..3 {
            let (tx, rx) = port::<JointVelocityCommand>();
            vel_txs.push(tx);
            vel_rxs.push(rx);
        }
        let (g_tx, g_rx) = port::<GripperCommand>();
        let c = PickPlace::new(
            enc_rxs,
            ee_rx,
            vel_txs,
            g_tx,
            (0.6, 0.0),
            (0.0, 0.6),
            0.8,
            0.4,
            0.4,
        );
        Rig {
            c,
            enc_txs,
            ee_tx,
            _vel_rxs: vel_rxs,
            _g_rx: g_rx,
        }
    }

    fn publish_encoders(rig: &Rig, qs: [f32; 3]) {
        for (i, q) in qs.iter().enumerate() {
            rig.enc_txs[i].send(JointEncoderReading {
                joint: JointId(i as u32),
                q: *q,
                q_dot: 0.0,
                sampled_at: Time::ZERO,
            });
        }
    }

    fn publish_ee_xy(rig: &Rig, xy: (f32, f32)) {
        rig.ee_tx.send(EePoseReading {
            pose: nalgebra::Isometry3::translation(xy.0, xy.1, 0.85),
            sampled_at: Time::ZERO,
        });
    }

    #[test]
    fn approach_yaw_advances_when_yaw_and_pitches_converged() {
        let mut rig = make_rig();
        publish_ee_xy(&rig, (0.6, 0.0));
        publish_encoders(&rig, [0.0, 0.0, 0.0]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), PickPlaceState::DescendOverBlock));
    }

    #[test]
    fn approach_yaw_stays_when_yaw_off() {
        let mut rig = make_rig();
        publish_ee_xy(&rig, (0.0, 0.6));
        publish_encoders(&rig, [0.0, 0.0, 0.0]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), PickPlaceState::ApproachYaw));
    }

    #[test]
    fn descend_over_block_transitions_to_close_when_pitches_converged() {
        let mut rig = make_rig();
        publish_ee_xy(&rig, (0.6, 0.0));
        publish_encoders(&rig, [0.0, 0.0, 0.0]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), PickPlaceState::DescendOverBlock));
        let (j1, j2) = rig.c.ik_at_block;
        publish_encoders(&rig, [0.0, j1, j2]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), PickPlaceState::CloseGripper(_)));
    }

    #[test]
    fn close_gripper_transitions_to_ascend_after_hold_ticks() {
        let mut rig = make_rig();
        publish_ee_xy(&rig, (0.6, 0.0));
        publish_encoders(&rig, [0.0, 0.0, 0.0]);
        rig.c.step(Time::ZERO).unwrap();
        let (j1, j2) = rig.c.ik_at_block;
        publish_encoders(&rig, [0.0, j1, j2]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), PickPlaceState::CloseGripper(_)));
        for _ in 0..210 {
            rig.c.step(Time::ZERO).unwrap();
        }
        assert!(matches!(rig.c.state(), PickPlaceState::AscendWithBlock));
    }

    #[test]
    fn ascend_transitions_to_yaw_to_bin_when_pitches_converged() {
        let mut rig = make_rig();
        publish_ee_xy(&rig, (0.6, 0.0));
        publish_encoders(&rig, [0.0, 0.0, 0.0]);
        rig.c.step(Time::ZERO).unwrap();
        let (j1_at, j2_at) = rig.c.ik_at_block;
        publish_encoders(&rig, [0.0, j1_at, j2_at]);
        rig.c.step(Time::ZERO).unwrap();
        for _ in 0..210 {
            rig.c.step(Time::ZERO).unwrap();
        }
        assert!(matches!(rig.c.state(), PickPlaceState::AscendWithBlock));
        let (j1_up, j2_up) = rig.c.ik_above_block;
        publish_encoders(&rig, [0.0, j1_up, j2_up]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), PickPlaceState::YawToBin));
    }

    #[test]
    fn yaw_to_bin_transitions_to_open_when_yaw_and_pitches_converged() {
        let mut rig = make_rig();
        publish_ee_xy(&rig, (0.6, 0.0));
        publish_encoders(&rig, [0.0, 0.0, 0.0]);
        rig.c.step(Time::ZERO).unwrap();
        let (j1_at, j2_at) = rig.c.ik_at_block;
        publish_encoders(&rig, [0.0, j1_at, j2_at]);
        rig.c.step(Time::ZERO).unwrap();
        for _ in 0..210 {
            rig.c.step(Time::ZERO).unwrap();
        }
        let (j1_up, j2_up) = rig.c.ik_above_block;
        publish_encoders(&rig, [0.0, j1_up, j2_up]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), PickPlaceState::YawToBin));
        publish_ee_xy(&rig, (0.0, 0.6));
        let (j1_bin, j2_bin) = rig.c.ik_above_bin;
        publish_encoders(&rig, [core::f32::consts::FRAC_PI_2, j1_bin, j2_bin]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), PickPlaceState::OpenGripper(_)));
    }
}

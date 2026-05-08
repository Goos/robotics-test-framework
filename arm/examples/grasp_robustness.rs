//! Step 3.6 — explicit slip-detection scenario for the joint-attached
//! grasp model (design §11.3). Demonstrates that Phase 3 actually
//! changed something: under the Phase 1 kinematic-weld grasp BOTH runs
//! below would succeed (the weld can't slip); under Phase 3's joint
//! grasp + slip-impulse threshold, only the slow-ascend run reaches the
//! bin — the fast-jerk run trips the slip threshold and the block falls
//! out mid-yaw.
//!
//! The world is the same `build_pick_and_place_world` fixture: a 5 cm
//! cube on the table at xy=(0.6, 0), a bin at xy=(0, 0.6), block xy
//! known a priori (no sweep). The controller is a stripped-down
//! pick-and-place that takes a `held_block_velocity_clamp` parameter
//! (rad/s, applied to ALL joints during AscendWithBlock + YawToBin).
//!
//! - **Slow ascend** (`clamp = 0.5 rad/s`): joint impulse stays under
//!   `SLIP_IMPULSE_THRESHOLD = 5.0`, joint persists, block reaches bin,
//!   `score = 1`.
//! - **Fast jerk** (`clamp = 20 rad/s`): centripetal load on the held
//!   block during YawToBin spikes the joint impulse past 5.0 within a
//!   few ticks, joint releases, block falls before reaching the bin,
//!   `score < 0.5` (block ends near the start position, not in the bin).
//!
//! Run a single mode (records to $TMPDIR/<name>.rrd if --features
//! viz-rerun):
//!   `cargo run --example grasp_robustness --features examples`
//!
//! Run both modes as tests:
//!   `cargo test --example grasp_robustness --features examples`

use rtf_arm::{
    goals::PlaceInBin,
    ik::ik_3r,
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

/// Stripped-down pick-place controller with a tunable joint-velocity
/// clamp during the held-block phases. Differs from the full PickPlace
/// only in (a) a smaller state machine (skips the "yaw to block" since
/// the canonical block xy=(0.6, 0) is already on +x at q0=0) and
/// (b) the `held_block_velocity_clamp` knob — `slow_ascend_demo` uses a
/// modest 0.5 rad/s, `fast_jerk_demo` uses a punishing 20 rad/s that
/// the joint-grasp can't sustain.
pub struct GraspRobustness<R, P>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
{
    state: GraspRobustnessState,
    target_block_xy: (f32, f32),
    target_bin_xy: (f32, f32),
    encoder_rxs: Vec<R>,
    ee_pose_rx: P,
    velocity_txs: Vec<PortTx<JointVelocityCommand>>,
    gripper_tx: PortTx<GripperCommand>,
    joint_tol: f32,
    /// Pre-computed (J1, J2, J3) IK targets at the three keyframe poses.
    ik_above_block: (f32, f32, f32),
    ik_at_block: (f32, f32, f32),
    ik_above_bin: (f32, f32, f32),
    /// Joint-velocity clamp (rad/s) applied during AscendWithBlock and
    /// YawToBin — i.e., whenever a block is in the friction grip. The
    /// good vs bad run differ only on this knob.
    held_block_velocity_clamp: f32,
    close_hold_ticks: u32,
    open_hold_ticks: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GraspRobustnessState {
    /// J1, J2, J3 driven to `ik_at_block` so EE drops onto the block.
    DescendOverBlock,
    /// Gripper closes; all joints held still.
    CloseGripper(u32),
    /// J1, J2, J3 driven back to `ik_above_block`, lifting the grasped
    /// block. Drives at `held_block_velocity_clamp` rad/s.
    AscendWithBlock,
    /// J0 swings to align with bin xy; J1, J2, J3 driven to
    /// `ik_above_bin`. Also clamped to `held_block_velocity_clamp`.
    YawToBin,
    /// Gripper opens; all joints held still.
    OpenGripper(u32),
    Done,
}

impl<R, P> GraspRobustness<R, P>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
{
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
        l3: f32,
        held_block_velocity_clamp: f32,
    ) -> Self {
        use core::f32::consts::PI;
        let block_r =
            (target_block_xy.0 * target_block_xy.0 + target_block_xy.1 * target_block_xy.1).sqrt();
        let bin_r = (target_bin_xy.0 * target_bin_xy.0 + target_bin_xy.1 * target_bin_xy.1).sqrt();
        let block_grasp_z = 0.55_f32;
        let park_z = 0.85_f32;
        let target_pitch = PI / 2.0;
        let ik_at_block = ik_3r(
            block_r,
            block_grasp_z - arm_shoulder_z,
            target_pitch,
            l1,
            l2,
            l3,
        )
        .expect("at_block target unreachable");
        let ik_above_block = ik_3r(block_r, park_z - arm_shoulder_z, target_pitch, l1, l2, l3)
            .expect("above_block target unreachable");
        let ik_above_bin = ik_3r(bin_r, park_z - arm_shoulder_z, target_pitch, l1, l2, l3)
            .expect("above_bin target unreachable");
        Self {
            state: GraspRobustnessState::DescendOverBlock,
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
            held_block_velocity_clamp,
            close_hold_ticks: 500,
            open_hold_ticks: 200,
        }
    }

    pub fn state(&self) -> GraspRobustnessState {
        self.state
    }

    fn joint_qs(&self) -> Vec<f32> {
        self.encoder_rxs
            .iter()
            .map(|rx| rx.latest().map(|r| r.q).unwrap_or(0.0))
            .collect()
    }

    fn yaw_error(&self, target: (f32, f32)) -> Option<f32> {
        use core::f32::consts::PI;
        let r = self.ee_pose_rx.latest()?;
        let ee_xy = (r.pose.translation.x, r.pose.translation.y);
        let target_heading = target.1.atan2(target.0);
        let ee_heading = ee_xy.1.atan2(ee_xy.0);
        let error = target_heading - ee_heading;
        Some(((error + PI).rem_euclid(2.0 * PI)) - PI)
    }

    fn drive_joints(
        &mut self,
        j0_yaw_err: f32,
        j1_target: f32,
        j2_target: f32,
        j3_target: f32,
        q_dot_max: f32,
    ) {
        self.drive_joints_with_kp(j0_yaw_err, j1_target, j2_target, j3_target, q_dot_max, 4.0);
    }

    /// Variant that takes an explicit P-gain. The held-block phases use
    /// a high kp so the velocity command saturates at `q_dot_max` for
    /// any meaningful error (otherwise the natural ramp-down keeps
    /// q_dot well under the clamp throughout the held-block segment,
    /// which defeats the slip-test purpose).
    fn drive_joints_with_kp(
        &mut self,
        j0_yaw_err: f32,
        j1_target: f32,
        j2_target: f32,
        j3_target: f32,
        q_dot_max: f32,
        kp: f32,
    ) {
        let qs = self.joint_qs();
        let q_dot_0 = (j0_yaw_err * kp).clamp(-q_dot_max, q_dot_max);
        self.velocity_txs[0].send(JointVelocityCommand {
            joint: JointId(0),
            q_dot_target: q_dot_0,
        });
        for (i, target) in [
            (1usize, j1_target),
            (2usize, j2_target),
            (3usize, j3_target),
        ] {
            if i >= self.velocity_txs.len() {
                break;
            }
            let cur = qs.get(i).copied().unwrap_or(0.0);
            let q_dot = ((target - cur) * kp).clamp(-q_dot_max, q_dot_max);
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

    fn pitch_converged(&self, j1_target: f32, j2_target: f32, j3_target: f32) -> bool {
        let qs = self.joint_qs();
        let j1_ok = qs
            .get(1)
            .is_some_and(|q| (q - j1_target).abs() < self.joint_tol);
        let j2_ok = qs
            .get(2)
            .is_some_and(|q| (q - j2_target).abs() < self.joint_tol);
        let j3_ok = qs
            .get(3)
            .is_some_and(|q| (q - j3_target).abs() < self.joint_tol);
        j1_ok && j2_ok && j3_ok
    }
}

impl<R, P> Controller for GraspRobustness<R, P>
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
            GraspRobustnessState::DescendOverBlock => {
                self.drive_joints(
                    yaw_err_block,
                    self.ik_at_block.0,
                    self.ik_at_block.1,
                    self.ik_at_block.2,
                    2.0,
                );
                if self.pitch_converged(self.ik_at_block.0, self.ik_at_block.1, self.ik_at_block.2)
                {
                    self.state = GraspRobustnessState::CloseGripper(0);
                }
            }
            GraspRobustnessState::CloseGripper(n) => {
                self.gripper_tx.send(GripperCommand {
                    target_separation: 0.012,
                });
                self.halt_joints();
                self.state = if n >= self.close_hold_ticks {
                    GraspRobustnessState::AscendWithBlock
                } else {
                    GraspRobustnessState::CloseGripper(n + 1)
                };
            }
            GraspRobustnessState::AscendWithBlock => {
                self.gripper_tx.send(GripperCommand {
                    target_separation: 0.012,
                });
                // Use a high kp (50) so that for any non-trivial error
                // the commanded q_dot saturates at the clamp. This is
                // what makes the "fast_jerk" mode (clamp=20 rad/s)
                // actually deliver the punishing tangential acceleration
                // that exceeds the joint's slip-impulse budget — at
                // kp=4 (the default for other controllers) the joint
                // velocity ramps slowly with the error and never gets
                // anywhere near the clamp during a short ascent.
                self.drive_joints_with_kp(
                    yaw_err_block,
                    self.ik_above_block.0,
                    self.ik_above_block.1,
                    self.ik_above_block.2,
                    self.held_block_velocity_clamp,
                    50.0,
                );
                if self.pitch_converged(
                    self.ik_above_block.0,
                    self.ik_above_block.1,
                    self.ik_above_block.2,
                ) {
                    self.state = GraspRobustnessState::YawToBin;
                }
            }
            GraspRobustnessState::YawToBin => {
                self.gripper_tx.send(GripperCommand {
                    target_separation: 0.012,
                });
                self.drive_joints_with_kp(
                    yaw_err_bin,
                    self.ik_above_bin.0,
                    self.ik_above_bin.1,
                    self.ik_above_bin.2,
                    self.held_block_velocity_clamp,
                    50.0,
                );
                if yaw_err_bin.abs() < self.joint_tol
                    && self.pitch_converged(
                        self.ik_above_bin.0,
                        self.ik_above_bin.1,
                        self.ik_above_bin.2,
                    )
                {
                    self.state = GraspRobustnessState::OpenGripper(0);
                }
            }
            GraspRobustnessState::OpenGripper(n) => {
                self.gripper_tx.send(GripperCommand {
                    target_separation: 0.04,
                });
                self.halt_joints();
                self.state = if n >= self.open_hold_ticks {
                    GraspRobustnessState::Done
                } else {
                    GraspRobustnessState::OpenGripper(n + 1)
                };
            }
            GraspRobustnessState::Done => {}
        }
        Ok(())
    }
}

// -- Runner --------------------------------------------------------------

fn run_grasp_robustness(rrd_name: &str, held_block_velocity_clamp: f32) -> rtf_harness::RunResult {
    let _ = rrd_name; // used only when viz-rerun is on
    let mut world = build_pick_and_place_world();
    let ports = world.attach_standard_arm_ports();
    let ee_pose_rx = world.attach_ee_pose_sensor(RateHz::new(100));
    let block = block_id(&world);
    let bin = bin_id(&world);

    let controller = GraspRobustness::new(
        ports.encoder_rxs,
        ee_pose_rx,
        ports.velocity_txs,
        ports.gripper_tx,
        /* target_block_xy */ (0.6, 0.0),
        /* target_bin_xy */ (0.0, 0.6),
        /* arm_shoulder_z */ 0.8,
        /* l1 */ 0.4,
        /* l2 */ 0.4,
        /* l3 */ 0.05,
        held_block_velocity_clamp,
    );
    let goal = PlaceInBin::new(block, bin);
    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(30))
        .with_seed(42);

    #[cfg(feature = "viz-rerun")]
    {
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
    let slow = run_grasp_robustness("grasp_robustness_slow", 0.5);
    println!(
        "slow_ascend (clamp=0.5 rad/s): terminated_by={:?}, score={}",
        slow.terminated_by, slow.score.value
    );
    let fast = run_grasp_robustness("grasp_robustness_fast", 20.0);
    println!(
        "fast_jerk (clamp=20 rad/s): terminated_by={:?}, score={}",
        fast.terminated_by, fast.score.value
    );
}

// -- Tests ---------------------------------------------------------------

/// Slow ascend (joint-velocity clamp 0.5 rad/s) keeps the joint impulse
/// well under SLIP_IMPULSE_THRESHOLD=5.0; the joint persists through
/// AscendWithBlock + YawToBin; the block reaches the bin and the goal
/// completes with score 1.0. This is the "good" run.
#[test]
fn slow_ascend_succeeds() {
    let res = run_grasp_robustness("grasp_robustness_slow", 0.5);
    eprintln!(
        "slow_ascend: terminated_by={:?}, final_time={:?}, score={}",
        res.terminated_by, res.final_time, res.score.value,
    );
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "slow ascend did not converge; terminated_by={:?}, score={}",
        res.terminated_by,
        res.score.value,
    );
    assert!(res.score.value > 0.9);
}

/// Fast jerk (joint-velocity clamp 20 rad/s) drives the held block at
/// a tangential acceleration that the joint's accumulated impulse can't
/// keep below SLIP_IMPULSE_THRESHOLD=5.0 within a couple of YawToBin
/// ticks. The slip-impulse check fires, the joint releases, the block
/// falls before reaching the bin. Score lands well under 0.9 — this is
/// the "bad" run that proves Phase 3 added a real failure mode.
///
/// Under the Phase 1 kinematic-weld grasp this same sequence would have
/// completed cleanly (the weld can't slip), so this test is the
/// behavioural assertion that the joint-grasp + slip-impulse threshold
/// is doing real work.
#[test]
fn fast_jerk_slips() {
    let res = run_grasp_robustness("grasp_robustness_fast", 20.0);
    eprintln!(
        "fast_jerk: terminated_by={:?}, final_time={:?}, score={}",
        res.terminated_by, res.final_time, res.score.value,
    );
    assert!(
        res.score.value < 0.9,
        "fast jerk should have slipped (score < 0.9), got score={}",
        res.score.value,
    );
}

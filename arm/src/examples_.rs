//! Example controllers and (later) test helpers, gated behind the `examples`
//! feature or `cfg(test)`. Trailing underscore avoids confusion with cargo's
//! `examples/` directory convention — this is a regular library module, not
//! an example binary.

use rtf_core::{
    controller::{ControlError, Controller},
    port::{PortReader, PortTx},
    time::Time,
};

use crate::ports::{
    EePoseReading, GripperCommand, JointEncoderReading, JointId, JointVelocityCommand,
};

/// Per-joint PD velocity controller (design v2 §6, Phase 5 happy path).
/// Generic over `R: PortReader<JointEncoderReading>` so it accepts either raw
/// `PortRx<_>` (Phase 5) or fault-wrapped readers (Phase 9.6) without changes.
///
/// One sensor port and one actuator port per joint, indexed by `i: usize`
/// into both `Vec`s — keeps lookup O(1) and avoids any HashMap dependency.
pub struct PdJointController<R: PortReader<JointEncoderReading>> {
    target_q: Vec<f32>,
    kp: f32,
    kd: f32,
    encoder_rxs: Vec<R>,
    velocity_txs: Vec<PortTx<JointVelocityCommand>>,
}

impl<R: PortReader<JointEncoderReading>> PdJointController<R> {
    pub fn new(
        target_q: Vec<f32>,
        encoder_rxs: Vec<R>,
        velocity_txs: Vec<PortTx<JointVelocityCommand>>,
    ) -> Self {
        Self {
            target_q,
            kp: 4.0,
            kd: 1.5,
            encoder_rxs,
            velocity_txs,
        }
    }
}

impl<R: PortReader<JointEncoderReading>> Controller for PdJointController<R> {
    fn step(&mut self, _t: Time) -> Result<(), ControlError> {
        for (i, rx) in self.encoder_rxs.iter().enumerate() {
            let Some(reading) = rx.latest() else { continue };
            let err = self.target_q[i] - reading.q;
            let q_dot_target = self.kp * err - self.kd * reading.q_dot;
            self.velocity_txs[i].send(JointVelocityCommand {
                joint: JointId(i as u32),
                q_dot_target,
            });
        }
        Ok(())
    }
}

/// Open-loop pick-and-place state-machine controller (Phase 7 §7.4a). Drives
/// the EE toward a block xy with a P-controller on heading (joint 0), closes
/// the gripper, drives to a bin xy, opens the gripper. Joints 1, 2 are held
/// still — joint 0 alone suffices to swing the planar chain to any xy
/// reachable within link-sum radius.
pub struct PickPlace<R, P>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
{
    state: PickPlaceState,
    target_block_xy: (f32, f32),
    target_bin_xy: (f32, f32),
    /// Joint encoders are accepted in the constructor for parity with
    /// `StandardArmPorts` and future joint-aware control extensions; v1
    /// state-machine drives off EE pose alone.
    #[allow(dead_code)]
    encoder_rxs: Vec<R>,
    ee_pose_rx: P,
    velocity_txs: Vec<PortTx<JointVelocityCommand>>,
    gripper_tx: PortTx<GripperCommand>,
    proximity_threshold: f32,
    close_hold_ticks: u32,
    open_hold_ticks: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickPlaceState {
    Approach,
    CloseGripper(u32),
    MoveToBin,
    OpenGripper(u32),
    Done,
}

impl<R, P> PickPlace<R, P>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
{
    pub fn new(
        encoder_rxs: Vec<R>,
        ee_pose_rx: P,
        velocity_txs: Vec<PortTx<JointVelocityCommand>>,
        gripper_tx: PortTx<GripperCommand>,
        target_block_xy: (f32, f32),
        target_bin_xy: (f32, f32),
    ) -> Self {
        Self {
            state: PickPlaceState::Approach,
            target_block_xy,
            target_bin_xy,
            encoder_rxs,
            ee_pose_rx,
            velocity_txs,
            gripper_tx,
            proximity_threshold: 0.05,
            close_hold_ticks: 50,
            open_hold_ticks: 50,
        }
    }

    pub fn state(&self) -> PickPlaceState { self.state }

    fn drive_toward_xy(&mut self, ee_xy: (f32, f32), target: (f32, f32)) {
        // Rotate joint 0 to align the chain heading with the target direction.
        // Wrap heading error to [-π, π] so the shortest rotation wins.
        use core::f32::consts::PI;
        let target_heading = target.1.atan2(target.0);
        let ee_heading = ee_xy.1.atan2(ee_xy.0);
        let error = target_heading - ee_heading;
        let wrapped = ((error + PI).rem_euclid(2.0 * PI)) - PI;
        let q_dot_target = (wrapped * 4.0).clamp(-1.0, 1.0);
        self.velocity_txs[0].send(JointVelocityCommand {
            joint: JointId(0),
            q_dot_target,
        });
        for (i, tx) in self.velocity_txs.iter().enumerate().skip(1) {
            tx.send(JointVelocityCommand {
                joint: JointId(i as u32),
                q_dot_target: 0.0,
            });
        }
    }

    fn ee_xy(&self) -> Option<(f32, f32)> {
        let r = self.ee_pose_rx.latest()?;
        Some((r.pose.translation.x, r.pose.translation.y))
    }

    fn halt_joints(&mut self) {
        for (i, tx) in self.velocity_txs.iter().enumerate() {
            tx.send(JointVelocityCommand {
                joint: JointId(i as u32),
                q_dot_target: 0.0,
            });
        }
    }
}

impl<R, P> Controller for PickPlace<R, P>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
{
    fn step(&mut self, _t: Time) -> Result<(), ControlError> {
        let Some(ee_xy) = self.ee_xy() else { return Ok(()); };
        match self.state {
            PickPlaceState::Approach => {
                self.drive_toward_xy(ee_xy, self.target_block_xy);
                let dx = ee_xy.0 - self.target_block_xy.0;
                let dy = ee_xy.1 - self.target_block_xy.1;
                if (dx * dx + dy * dy).sqrt() < self.proximity_threshold {
                    self.state = PickPlaceState::CloseGripper(0);
                }
            }
            PickPlaceState::CloseGripper(n) => {
                self.gripper_tx.send(GripperCommand { close: true });
                self.halt_joints();
                self.state = if n >= self.close_hold_ticks {
                    PickPlaceState::MoveToBin
                } else {
                    PickPlaceState::CloseGripper(n + 1)
                };
            }
            PickPlaceState::MoveToBin => {
                self.gripper_tx.send(GripperCommand { close: true });
                self.drive_toward_xy(ee_xy, self.target_bin_xy);
                let dx = ee_xy.0 - self.target_bin_xy.0;
                let dy = ee_xy.1 - self.target_bin_xy.1;
                if (dx * dx + dy * dy).sqrt() < self.proximity_threshold {
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

#[cfg(test)]
mod tests {
    use super::*;
    use rtf_core::port::port;

    #[test]
    fn pd_emits_velocity_command_proportional_to_error() {
        let (enc_tx, enc_rx) = port::<JointEncoderReading>();
        let (vel_tx, mut vel_rx) = port::<JointVelocityCommand>();
        let mut c = PdJointController::new(vec![1.0], vec![enc_rx], vec![vel_tx]);
        enc_tx.send(JointEncoderReading {
            joint: JointId(0),
            q: 0.0,
            q_dot: 0.0,
            sampled_at: Time::ZERO,
        });
        c.step(Time::ZERO).unwrap();
        let cmd = vel_rx.take().unwrap();
        assert!(cmd.q_dot_target > 0.0);
    }
}

#[cfg(test)]
mod pick_place_tests {
    use super::*;
    use rtf_core::port::port;

    #[allow(clippy::type_complexity)]
    fn make_controller(
        ee_xy: (f32, f32),
    ) -> (
        PickPlace<rtf_core::port::PortRx<JointEncoderReading>, rtf_core::port::PortRx<EePoseReading>>,
        rtf_core::port::PortRx<JointVelocityCommand>,
        rtf_core::port::PortRx<GripperCommand>,
    ) {
        let (_enc_tx, enc_rx) = port::<JointEncoderReading>();
        let (ee_tx, ee_rx) = port::<EePoseReading>();
        let (vel_tx, vel_rx) = port::<JointVelocityCommand>();
        let (g_tx, g_rx) = port::<GripperCommand>();
        ee_tx.send(EePoseReading {
            pose: nalgebra::Isometry3::translation(ee_xy.0, ee_xy.1, 0.0),
            sampled_at: Time::ZERO,
        });
        let c = PickPlace::new(
            vec![enc_rx],
            ee_rx,
            vec![vel_tx],
            g_tx,
            (0.5, 0.0),
            (0.0, 0.5),
        );
        (c, vel_rx, g_rx)
    }

    #[test]
    fn approach_transitions_to_close_when_within_threshold() {
        let (mut c, _vel_rx, _g_rx) = make_controller((0.5, 0.0));
        c.step(Time::ZERO).unwrap();
        assert!(matches!(c.state(), PickPlaceState::CloseGripper(_)));
    }

    #[test]
    fn approach_stays_when_outside_threshold() {
        let (mut c, _vel_rx, _g_rx) = make_controller((1.0, 0.0));
        c.step(Time::ZERO).unwrap();
        assert!(matches!(c.state(), PickPlaceState::Approach));
    }

    #[test]
    fn close_gripper_transitions_to_move_after_hold_ticks() {
        let (mut c, _vel_rx, _g_rx) = make_controller((0.5, 0.0));
        c.step(Time::ZERO).unwrap();
        assert!(matches!(c.state(), PickPlaceState::CloseGripper(_)));
        for _ in 0..60 {
            c.step(Time::ZERO).unwrap();
        }
        assert!(matches!(c.state(), PickPlaceState::MoveToBin));
    }
}

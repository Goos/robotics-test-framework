//! Example controllers and (later) test helpers, gated behind the `examples`
//! feature or `cfg(test)`. Trailing underscore avoids confusion with cargo's
//! `examples/` directory convention — this is a regular library module, not
//! an example binary.

use rtf_core::{
    controller::{ControlError, Controller},
    port::{PortReader, PortTx},
    time::Time,
};

use crate::ports::{JointEncoderReading, JointId, JointVelocityCommand};

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

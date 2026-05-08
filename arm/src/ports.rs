use nalgebra::Isometry3;
use rtf_core::{
    noise_source::NoiseSource,
    sensor_reading::{Noise, SensorReading},
    time::Time,
};

/// Stable identifier for a joint within an arm (design v2 §5.6).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct JointId(pub u32);

/// Per-joint encoder reading (design v2 §5.6, §6): position + velocity with
/// a sample timestamp. Implements `SensorReading` (design §9.4).
#[derive(Clone, Debug)]
pub struct JointEncoderReading {
    pub joint: JointId,
    pub q: f32,
    pub q_dot: f32,
    pub sampled_at: Time,
}

impl SensorReading for JointEncoderReading {
    fn sampled_at(&self) -> Time {
        self.sampled_at
    }
}

impl Noise for JointEncoderReading {
    fn apply_noise(&mut self, source: &mut dyn NoiseSource, stddev: f32) {
        self.q += stddev * source.standard_normal();
        self.q_dot += stddev * source.standard_normal();
    }
}

/// End-effector pose reading (design v2 §5.6, §6): world-frame pose with a
/// sample timestamp. Implements `SensorReading` (design §9.4).
#[derive(Clone, Debug)]
pub struct EePoseReading {
    pub pose: Isometry3<f32>,
    pub sampled_at: Time,
}

impl SensorReading for EePoseReading {
    fn sampled_at(&self) -> Time {
        self.sampled_at
    }
}

/// Per-joint velocity-command actuator input (design v2 §5.6).
#[derive(Copy, Clone, Debug)]
pub struct JointVelocityCommand {
    pub joint: JointId,
    pub q_dot_target: f32,
}

/// Gripper actuator input (rapier-integration Phase 3.2 — replaces the
/// v1 `close: bool`): the requested lateral finger separation in metres.
/// The world drives `arm.state.gripper_separation` toward this value at
/// a fixed rate (0.5 m/s). Conventional "open" target is 0.04 m,
/// "close" is 0.012 m.
#[derive(Copy, Clone, Debug)]
pub struct GripperCommand {
    pub target_separation: f32,
}

/// EE-mounted scalar pressure sensor reading (find-grasp-place design §2).
/// `pressure` is the proximity-falloff signal `(eps - d) / eps` for the
/// nearest sim entity, clamped to >= 0; values >= 1.0 mean the EE is at or
/// inside the surface. Future-proofed as a scalar so noise / multi-finger
/// extensions don't break the API.
#[derive(Clone, Debug)]
pub struct PressureReading {
    pub pressure: f32,
    pub sampled_at: Time,
}

impl SensorReading for PressureReading {
    fn sampled_at(&self) -> Time {
        self.sampled_at
    }
}

/// Per-joint contact torque reading (rapier-integration design §7.1) — the
/// scalar torque about the joint's rotation axis induced by external
/// contacts on links distal to this joint. Zero in free motion; non-zero
/// when downstream link colliders are in contact with dynamic Rapier
/// bodies. Sign follows the joint's positive-rotation convention.
#[derive(Clone, Debug)]
pub struct JointTorqueReading {
    pub joint: JointId,
    pub tau: f32,
    pub sampled_at: Time,
}

impl SensorReading for JointTorqueReading {
    fn sampled_at(&self) -> Time {
        self.sampled_at
    }
}

/// Latest single-arm-link external contact in world coordinates: the
/// impulse-magnitude-weighted centroid of all link-vs-dynamic-body contact
/// points in the most recent physics step, plus the sum of impulses (whose
/// direction approximates the contact normal pointing into the contacted
/// body). Both fields are `None` when no link is in contact this tick.
/// Useful for controllers that need to know *where* a touch occurred
/// (e.g. find-by-touch) rather than just whether a torque exists at a
/// joint, and which side the touched body sits on relative to the link.
///
/// Only meaningful with the `physics-rapier` feature on (publishers always
/// emit `None` otherwise).
#[derive(Clone, Debug)]
pub struct ArmContactReading {
    pub point_world: Option<nalgebra::Point3<f32>>,
    pub impulse_world: Option<nalgebra::Vector3<f32>>,
    pub sampled_at: Time,
}

impl SensorReading for ArmContactReading {
    fn sampled_at(&self) -> Time {
        self.sampled_at
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rtf_core::{
        noise_source::NoiseSource,
        sensor_reading::{Noise, SensorReading},
        time::Time,
    };

    /// Deterministic NoiseSource that returns a fixed sequence — keeps the
    /// apply_noise unit test independent of any concrete RNG choice.
    struct FixedSource(std::vec::IntoIter<f32>);
    impl NoiseSource for FixedSource {
        fn standard_normal(&mut self) -> f32 {
            self.0.next().expect("ran out of fixed draws")
        }
        fn uniform_unit(&mut self) -> f32 {
            self.0.next().expect("ran out of fixed draws")
        }
    }

    #[test]
    fn joint_encoder_reading_exposes_sample_time() {
        let r = JointEncoderReading {
            joint: JointId(0),
            q: 0.5,
            q_dot: 0.1,
            sampled_at: Time::from_millis(7),
        };
        assert_eq!(r.sampled_at(), Time::from_millis(7));
    }

    #[test]
    fn apply_noise_perturbs_q_and_q_dot_via_noise_source() {
        let draws = vec![1.5_f32, -2.0_f32];
        let mut source = FixedSource(draws.into_iter());
        let mut r = JointEncoderReading {
            joint: JointId(0),
            q: 1.0,
            q_dot: 0.0,
            sampled_at: Time::from_nanos(0),
        };
        r.apply_noise(&mut source as &mut dyn NoiseSource, 0.01);
        // q gets the first draw (1.5), q_dot gets the second (-2.0); both
        // scaled by stddev=0.01.
        assert!((r.q - 1.015).abs() < 1e-6, "q={}", r.q);
        assert!((r.q_dot - (-0.02)).abs() < 1e-6, "q_dot={}", r.q_dot);
    }
}

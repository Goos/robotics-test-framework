use nalgebra::Isometry3;
use rand_distr::{Distribution, StandardNormal};
use rtf_core::{sensor_reading::{Noise, SensorReading}, time::Time};

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
    fn sampled_at(&self) -> Time { self.sampled_at }
}

impl Noise for JointEncoderReading {
    fn apply_noise(&mut self, rng: &mut rand_pcg::Pcg64, stddev: f32) {
        let n: f32 = StandardNormal.sample(rng);
        self.q += stddev * n;
        let n: f32 = StandardNormal.sample(rng);
        self.q_dot += stddev * n;
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
    fn sampled_at(&self) -> Time { self.sampled_at }
}

/// Per-joint velocity-command actuator input (design v2 §5.6).
#[derive(Copy, Clone, Debug)]
pub struct JointVelocityCommand {
    pub joint: JointId,
    pub q_dot_target: f32,
}

/// Gripper actuator input (design v2 §5.6): `close = true` requests grasp.
#[derive(Copy, Clone, Debug)]
pub struct GripperCommand {
    pub close: bool,
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;
    use rand_pcg::Pcg64;
    use rtf_core::{sensor_reading::{Noise, SensorReading}, time::Time};
    #[test]
    fn joint_encoder_reading_exposes_sample_time() {
        let r = JointEncoderReading {
            joint: JointId(0), q: 0.5, q_dot: 0.1, sampled_at: Time::from_millis(7),
        };
        assert_eq!(r.sampled_at(), Time::from_millis(7));
    }

    #[test]
    fn apply_noise_perturbs_q_and_q_dot_with_known_seed() {
        // Seed 7 (per plan v2 §9.5) is known to produce non-trivial draws on
        // both q and q_dot — both should drift from their starting values.
        let mut rng = Pcg64::seed_from_u64(7);
        let mut r = JointEncoderReading {
            joint: JointId(0), q: 1.0, q_dot: 0.0, sampled_at: Time::from_nanos(0),
        };
        r.apply_noise(&mut rng, 0.01);
        assert!(
            (r.q - 1.0).abs() > 1e-9 || r.q_dot.abs() > 1e-9,
            "expected at least one of q,q_dot perturbed; got q={}, q_dot={}",
            r.q, r.q_dot,
        );
        assert!((r.q - 1.0).abs() < 5.0 * 0.01, "q drift larger than 5 sigma");
        assert!(r.q_dot.abs() < 5.0 * 0.01, "q_dot drift larger than 5 sigma");
    }
}

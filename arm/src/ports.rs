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
    fn sampled_at(&self) -> Time { self.sampled_at }
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
    }

    #[test]
    fn joint_encoder_reading_exposes_sample_time() {
        let r = JointEncoderReading {
            joint: JointId(0), q: 0.5, q_dot: 0.1, sampled_at: Time::from_millis(7),
        };
        assert_eq!(r.sampled_at(), Time::from_millis(7));
    }

    #[test]
    fn apply_noise_perturbs_q_and_q_dot_via_noise_source() {
        let draws = vec![1.5_f32, -2.0_f32];
        let mut source = FixedSource(draws.into_iter());
        let mut r = JointEncoderReading {
            joint: JointId(0), q: 1.0, q_dot: 0.0, sampled_at: Time::from_nanos(0),
        };
        r.apply_noise(&mut source as &mut dyn NoiseSource, 0.01);
        // q gets the first draw (1.5), q_dot gets the second (-2.0); both
        // scaled by stddev=0.01.
        assert!((r.q - 1.015).abs() < 1e-6, "q={}", r.q);
        assert!((r.q_dot - (-0.02)).abs() < 1e-6, "q_dot={}", r.q_dot);
    }
}

use nalgebra::Isometry3;
use rtf_core::{sensor_reading::SensorReading, time::Time};

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
    use rtf_core::{sensor_reading::SensorReading, time::Time};
    #[test]
    fn joint_encoder_reading_exposes_sample_time() {
        let r = JointEncoderReading {
            joint: JointId(0), q: 0.5, q_dot: 0.1, sampled_at: Time::from_millis(7),
        };
        assert_eq!(r.sampled_at(), Time::from_millis(7));
    }
}

use nalgebra::{Isometry3, Unit, Vector3};

/// Per-joint kinematic spec (design v2 §5.3).
#[derive(Clone, Debug)]
pub enum JointSpec {
    Revolute {
        axis: Unit<Vector3<f32>>,
        limits: (f32, f32),
    },
    Prismatic {
        axis: Unit<Vector3<f32>>,
        limits: (f32, f32),
    },
}

/// End-effector spec (design v2 §5.3).
#[derive(Copy, Clone, Debug)]
pub struct GripperSpec {
    pub proximity_threshold: f32,
    pub max_grasp_size: f32,
}

/// Full arm kinematic spec (design v2 §5.3): ordered joint chain with
/// per-link offsets and a gripper at the tip.
#[derive(Clone, Debug)]
pub struct ArmSpec {
    pub joints: Vec<JointSpec>,
    pub link_offsets: Vec<Isometry3<f32>>,
    pub gripper: GripperSpec,
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::{FRAC_PI_2, PI};
    use nalgebra::{Isometry3, Vector3};
    #[test]
    fn arm_spec_with_two_revolute_joints() {
        let spec = ArmSpec {
            joints: vec![
                JointSpec::Revolute {
                    axis: Vector3::z_axis(),
                    limits: (-PI, PI),
                },
                JointSpec::Revolute {
                    axis: Vector3::y_axis(),
                    limits: (-FRAC_PI_2, FRAC_PI_2),
                },
            ],
            link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.1); 2],
            gripper: GripperSpec {
                proximity_threshold: 0.02,
                max_grasp_size: 0.05,
            },
        };
        assert_eq!(spec.joints.len(), 2);
    }
}

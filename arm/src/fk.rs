use nalgebra::{Isometry3, Translation3, UnitQuaternion};

use crate::spec::{ArmSpec, JointSpec};

/// Single-joint forward kinematic transform: maps joint coordinate `q` to the
/// rigid-body transform contributed by that joint (design v2 §5.3).
pub fn joint_transform(spec: &JointSpec, q: f32) -> Isometry3<f32> {
    match spec {
        JointSpec::Revolute { axis, .. } => {
            let rot = UnitQuaternion::from_axis_angle(axis, q);
            Isometry3::from_parts(Translation3::identity(), rot)
        }
        JointSpec::Prismatic { axis, .. } => {
            let trans = Translation3::from(axis.into_inner() * q);
            Isometry3::from_parts(trans, UnitQuaternion::identity())
        }
    }
}

/// Multi-joint forward kinematics (design v2 §5.3): composes per-joint
/// transforms and per-link offsets in chain order. For joint i with
/// coordinate q[i] and link offset L_i, the chain is
/// `T_0 = I, T_{i+1} = T_i · J_i(q_i) · L_i`. Returns the end-effector pose.
pub fn forward_kinematics(spec: &ArmSpec, q: &[f32]) -> Isometry3<f32> {
    spec.joints.iter().enumerate().fold(Isometry3::identity(), |acc, (i, joint)| {
        acc * joint_transform(joint, q[i]) * spec.link_offsets[i]
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::FRAC_PI_2;
    use nalgebra::{Isometry3, Vector3};
    use crate::spec::JointSpec;

    #[test]
    fn revolute_z_rotates_offset_by_q() {
        let joint = JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-3.2, 3.2) };
        let offset = Isometry3::translation(1.0, 0.0, 0.0);
        let pose = joint_transform(&joint, FRAC_PI_2) * offset;
        assert!((pose.translation.x - 0.0).abs() < 1e-5);
        assert!((pose.translation.y - 1.0).abs() < 1e-5);
    }

    #[test]
    fn prismatic_x_translates_by_q() {
        let joint = JointSpec::Prismatic { axis: Vector3::x_axis(), limits: (0.0, 1.0) };
        let pose = joint_transform(&joint, 0.5);
        assert!((pose.translation.x - 0.5).abs() < 1e-9);
    }

    #[test]
    fn three_joint_chain_at_zero_q_yields_sum_of_offsets() {
        use crate::spec::{ArmSpec, GripperSpec, JointSpec};
        use nalgebra::{Isometry3, Vector3};
        let spec = ArmSpec {
            joints: vec![JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-3.2, 3.2) }; 3],
            link_offsets: vec![Isometry3::translation(0.5, 0.0, 0.0); 3],
            gripper: GripperSpec { proximity_threshold: 0.01, max_grasp_size: 0.05 },
        };
        let q = vec![0.0; 3];
        let ee = forward_kinematics(&spec, &q);
        assert!((ee.translation.x - 1.5).abs() < 1e-5);
        assert!(ee.translation.y.abs() < 1e-5);
    }

    #[test]
    fn first_joint_pi_over_2_swings_chain_to_plus_y() {
        use crate::spec::{ArmSpec, GripperSpec, JointSpec};
        use nalgebra::{Isometry3, Vector3};
        let spec = ArmSpec {
            joints: vec![JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-3.2, 3.2) }; 3],
            link_offsets: vec![Isometry3::translation(0.5, 0.0, 0.0); 3],
            gripper: GripperSpec { proximity_threshold: 0.01, max_grasp_size: 0.05 },
        };
        // Only the first joint rotated 90°: the entire chain swings to +y axis.
        let q = vec![FRAC_PI_2, 0.0, 0.0];
        let ee = forward_kinematics(&spec, &q);
        assert!(ee.translation.x.abs() < 1e-4, "x should be ~0, got {}", ee.translation.x);
        assert!((ee.translation.y - 1.5).abs() < 1e-4, "y should be ~1.5, got {}", ee.translation.y);
    }
}

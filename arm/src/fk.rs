use nalgebra::{Isometry3, Translation3, UnitQuaternion};

use crate::spec::JointSpec;

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
}

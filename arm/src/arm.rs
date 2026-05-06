use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use rtf_sim::entity::EntityId;
use rtf_sim::primitive::{Color, Primitive};
use rtf_sim::visualizable::Visualizable;

use crate::fk::joint_transform;
use crate::spec::ArmSpec;
use crate::state::ArmState;

/// Per-arm runtime aggregate (design v2 §5.3): immutable spec, per-tick
/// mutable state, and a stable numeric id used to namespace visualization
/// entity ids across multiple arms.
#[derive(Clone, Debug)]
pub struct Arm {
    pub spec: ArmSpec,
    pub state: ArmState,
    pub id: u32,
}

const LINK_RADIUS: f32 = 0.02;
/// Half-extents of each finger pillar (small thin box, 4 cm long along EE +z).
const FINGER_HALF_EXTENTS: Vector3<f32> = Vector3::new(0.01, 0.01, 0.04);
/// Lateral separation of finger centers along EE +y when the gripper is open.
const FINGER_OPEN_SEPARATION: f32 = 0.04;
/// Lateral separation of finger centers along EE +y when the gripper is closed.
const FINGER_CLOSED_SEPARATION: f32 = 0.012;
/// Forward offset of the fingers along EE +z so they protrude past the EE
/// origin (matches `FINGER_HALF_EXTENTS.z` so fingers extend from EE surface).
const FINGER_FORWARD_OFFSET: f32 = 0.04;

impl Visualizable for Arm {
    fn append_primitives(&self, out: &mut Vec<(EntityId, Primitive)>) {
        let mut acc = Isometry3::identity();
        for (i, joint) in self.spec.joints.iter().enumerate() {
            acc *= joint_transform(joint, self.state.q[i]);
            let link_offset = self.spec.link_offsets[i];
            let link_vec = link_offset.translation.vector;
            let half_height = (link_vec.norm() / 2.0).max(0.001);
            // Orient the capsule so its local +Z axis aligns with the link
            // direction. Without this rotation the capsule "stick" would be
            // drawn perpendicular to the actual link (since `Capsule`'s axis
            // is local +Z but the link points wherever the offset translates).
            let z_to_link = if link_vec.norm() > 0.0 {
                UnitQuaternion::rotation_between(&Vector3::z(), &link_vec.normalize())
                    .unwrap_or_else(UnitQuaternion::identity)
            } else {
                UnitQuaternion::identity()
            };
            let mid = acc * Isometry3::from_parts(Translation3::from(link_vec / 2.0), z_to_link);
            out.push((
                EntityId::Arm {
                    arm_id: self.id,
                    slot: i as u32,
                },
                Primitive::Capsule {
                    pose: mid,
                    half_height,
                    radius: LINK_RADIUS,
                    color: Color::WHITE,
                },
            ));
            acc *= link_offset;
        }
        // Articulated gripper: two finger pillars whose lateral separation
        // is the visual proxy for the gripper open/close state. Both fingers
        // protrude forward of the EE origin along its local +z.
        let separation = if self.state.gripper_closed {
            FINGER_CLOSED_SEPARATION
        } else {
            FINGER_OPEN_SEPARATION
        };
        for (slot, sign) in [(998, 1.0_f32), (999, -1.0_f32)] {
            let finger_pose = acc
                * Isometry3::from_parts(
                    Translation3::new(0.0, sign * separation, FINGER_FORWARD_OFFSET),
                    UnitQuaternion::identity(),
                );
            out.push((
                EntityId::Arm {
                    arm_id: self.id,
                    slot,
                },
                Primitive::Box {
                    pose: finger_pose,
                    half_extents: FINGER_HALF_EXTENTS,
                    color: Color::RED,
                },
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spec::{ArmSpec, GripperSpec, JointSpec};
    use crate::state::ArmState;
    use nalgebra::{Isometry3, Vector3};
    use rtf_sim::primitive::Primitive;
    use rtf_sim::visualizable::Visualizable;

    #[test]
    fn appends_n_capsules_and_two_finger_boxes() {
        let spec = ArmSpec {
            joints: vec![
                JointSpec::Revolute {
                    axis: Vector3::z_axis(),
                    limits: (-3.2, 3.2)
                };
                3
            ],
            link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.1); 3],
            gripper: GripperSpec {
                proximity_threshold: 0.02,
                max_grasp_size: 0.05,
            },
        };
        let state = ArmState::zeros(3);
        let arm = Arm { spec, state, id: 0 };
        let mut out = Vec::new();
        arm.append_primitives(&mut out);
        let n_capsules = out
            .iter()
            .filter(|(_, p)| matches!(p, Primitive::Capsule { .. }))
            .count();
        let n_boxes = out
            .iter()
            .filter(|(_, p)| matches!(p, Primitive::Box { .. }))
            .count();
        assert_eq!(n_capsules, 3);
        assert_eq!(n_boxes, 2);
    }

    #[test]
    fn all_emitted_ids_are_arm_namespaced() {
        let spec = ArmSpec {
            joints: vec![
                JointSpec::Revolute {
                    axis: Vector3::z_axis(),
                    limits: (-3.2, 3.2)
                };
                2
            ],
            link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.1); 2],
            gripper: GripperSpec {
                proximity_threshold: 0.02,
                max_grasp_size: 0.05,
            },
        };
        let state = ArmState::zeros(2);
        let arm = Arm { spec, state, id: 0 };
        let mut out = Vec::new();
        arm.append_primitives(&mut out);
        assert!(
            out.iter()
                .all(|(id, _)| matches!(id, EntityId::Arm { arm_id: 0, .. })),
            "expected every emitted id to be EntityId::Arm; got {:?}",
            out.iter().map(|(id, _)| *id).collect::<Vec<_>>(),
        );
    }
}

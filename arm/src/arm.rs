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

/// Per-link FK output: world-frame midpoint pose of the link's capsule,
/// plus its half-height. The capsule's local +Z points along the link
/// direction (matches `Arm::append_primitives`'s rotation logic).
///
/// Used by the Rapier integration (Step 1.5+) to keep arm-link kinematic
/// body poses synced with FK each tick.
#[derive(Clone, Copy, Debug)]
pub struct LinkPose {
    pub slot: u32,
    pub pose: Isometry3<f32>,
    pub half_height: f32,
}

impl Arm {
    /// Compute the per-link capsule midpoint pose + half-height for every
    /// link in the kinematic chain. Used by the Rapier integration to
    /// keep arm-link kinematic body poses synced with FK each tick (and
    /// also by `Visualizable::append_primitives` for rendering).
    pub fn link_poses(&self) -> Vec<LinkPose> {
        let mut out = Vec::with_capacity(self.spec.joints.len());
        let mut acc = Isometry3::identity();
        for (i, joint) in self.spec.joints.iter().enumerate() {
            acc *= joint_transform(joint, self.state.q[i]);
            let link_offset = self.spec.link_offsets[i];
            let link_vec = link_offset.translation.vector;
            let half_height = (link_vec.norm() / 2.0).max(0.001);
            let z_to_link = if link_vec.norm() > 0.0 {
                UnitQuaternion::rotation_between(&Vector3::z(), &link_vec.normalize())
                    .unwrap_or_else(UnitQuaternion::identity)
            } else {
                UnitQuaternion::identity()
            };
            let mid = acc * Isometry3::from_parts(Translation3::from(link_vec / 2.0), z_to_link);
            out.push(LinkPose {
                slot: i as u32,
                pose: mid,
                half_height,
            });
            acc *= link_offset;
        }
        out
    }
}

pub const LINK_RADIUS: f32 = 0.02;
/// Half-extents of each finger pillar (small thin box, 4 cm long along EE +z).
pub const FINGER_HALF_EXTENTS: Vector3<f32> = Vector3::new(0.01, 0.01, 0.04);
/// Lateral separation of finger centers along EE +y when the gripper is open.
pub const FINGER_OPEN_SEPARATION: f32 = 0.04;
/// Lateral separation of finger centers along EE +y when the gripper is closed.
pub const FINGER_CLOSED_SEPARATION: f32 = 0.012;
/// Forward offset of the fingers along EE +z so they protrude past the EE
/// origin (matches `FINGER_HALF_EXTENTS.z` so fingers extend from EE surface).
pub const FINGER_FORWARD_OFFSET: f32 = 0.04;
/// Slot ids assigned to the two finger primitives within `EntityId::Arm`.
/// 998/999 are kept high so they don't collide with link slots (which are
/// indexed 0..n_joints) and stay stable across rerun runs.
pub const FINGER_SLOT_PLUS: u32 = 998;
pub const FINGER_SLOT_MINUS: u32 = 999;

/// Compute the world-space pose of one finger given the EE pose, finger
/// slot (+y or -y side), and current lateral separation. Shared by the
/// visualization (`append_primitives`) and the per-tick physics-pose
/// update so both layers see exactly the same finger geometry.
pub fn finger_pose(ee_pose: Isometry3<f32>, slot: u32, separation: f32) -> Isometry3<f32> {
    let sign = if slot == FINGER_SLOT_PLUS {
        1.0_f32
    } else {
        -1.0
    };
    ee_pose
        * Isometry3::from_parts(
            Translation3::new(0.0, sign * separation, FINGER_FORWARD_OFFSET),
            UnitQuaternion::identity(),
        )
}

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
        // is the live `gripper_separation` (0.012 closed → 0.04 open after
        // Phase 3.2). Both fingers protrude forward of the EE origin along
        // its local +z.
        let separation = self.state.gripper_separation;
        for slot in [FINGER_SLOT_PLUS, FINGER_SLOT_MINUS] {
            out.push((
                EntityId::Arm {
                    arm_id: self.id,
                    slot,
                },
                Primitive::Box {
                    pose: finger_pose(acc, slot, separation),
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

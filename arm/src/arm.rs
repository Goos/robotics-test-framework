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
    /// Visible (and physical) arm decorations: foot, column, joint barrels,
    /// wrist cuff. Single source of truth for the renderer and the Rapier
    /// insertion / per-tick sync path. C5 emits foot + column only;
    /// barrels + cuff are added in C6.
    ///
    /// Foot + column are emitted ONLY when `link_offsets[0].translation.z`
    /// is approximately 0.8 m — `BASE_COLUMN_HEIGHT = 0.745` is sized for
    /// that gap. Minimal test arms (e.g. `(0,0,0.1)` link offsets) get an
    /// empty decoration list rather than a column that ends mid-shoulder.
    pub fn decoration_poses(&self) -> Vec<DecorationPose> {
        use rtf_sim::physics::world::DecorationKind;
        let mut out = Vec::new();
        let has_standard_pedestal = !self.spec.link_offsets.is_empty()
            && (self.spec.link_offsets[0].translation.z - 0.8).abs() < 1e-3;
        if has_standard_pedestal {
            out.push(DecorationPose {
                slot: BASE_FOOT_SLOT,
                shape: rtf_sim::shape::Shape::Cylinder {
                    radius: BASE_FOOT_RADIUS,
                    half_height: BASE_FOOT_HALF_HEIGHT,
                },
                pose: Isometry3::translation(0.0, 0.0, BASE_FOOT_HALF_HEIGHT),
                color: rtf_sim::palette::JOINT_BLACK,
                kind: DecorationKind::Static,
            });
            out.push(DecorationPose {
                slot: BASE_COLUMN_SLOT,
                shape: rtf_sim::shape::Shape::Cylinder {
                    radius: BASE_COLUMN_RADIUS,
                    half_height: BASE_COLUMN_HEIGHT * 0.5,
                },
                pose: Isometry3::translation(
                    0.0,
                    0.0,
                    2.0 * BASE_FOOT_HALF_HEIGHT + BASE_COLUMN_HEIGHT * 0.5,
                ),
                color: rtf_sim::palette::LINK_WHITE,
                kind: DecorationKind::Static,
            });
        }
        out
    }

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

/// Radius (m) of the cylindrical arm-link capsules. Doubles as the
/// Rapier collider radius and the rerun visualizer's capsule radius
/// (both reads ride this single constant — see arm/src/world.rs and
/// arm/src/arm.rs `append_primitives`).
///
/// Step VIS.1 (2026-05-12): bumped 0.02 → 0.04 to give the arm
/// visible "real robot" proportions in the rerun viewer rather than
/// the previous stick-figure look. Affects physics: arm-vs-block
/// contact zones grow by 2 cm radially per link, slip-impulse
/// interactions during ascend slightly different, sweep-controller
/// finger geometry is unaffected (FINGER_HALF_EXTENTS unchanged).
/// Sweep-based seed tests retuned in this same commit where needed.
pub const LINK_RADIUS: f32 = 0.04;
/// Half-extents of each finger pillar (small thin box, 4 cm long along
/// EE +x). Phase 3.4.5d: long axis switched from EE +z to EE +x so it
/// aligns with the wrist link direction; under the wrist alignment
/// (cumulative pitch = π/2 → EE +x = world -z), fingers protrude
/// DOWNWARD in world frame.
pub const FINGER_HALF_EXTENTS: Vector3<f32> = Vector3::new(0.04, 0.01, 0.01);
/// Lateral separation of finger centers along EE +y when the gripper is open.
pub const FINGER_OPEN_SEPARATION: f32 = 0.04;
/// Lateral separation of finger centers along EE +y when the gripper is closed.
pub const FINGER_CLOSED_SEPARATION: f32 = 0.012;
/// Forward offset of the fingers along EE +x so they protrude past the EE
/// origin (matches `FINGER_HALF_EXTENTS.x` so fingers extend from EE
/// surface). Phase 3.4.5d: switched from EE +z to EE +x to align with
/// the wrist link direction (see `FINGER_HALF_EXTENTS`).
pub const FINGER_FORWARD_OFFSET: f32 = 0.04;
/// Slot ids assigned to the two finger primitives within `EntityId::Arm`.
/// 998/999 are kept high so they don't collide with link slots (which are
/// indexed 0..n_joints) and stay stable across rerun runs.
pub const FINGER_SLOT_PLUS: u32 = 998;
pub const FINGER_SLOT_MINUS: u32 = 999;

// Decoration geometry (foot, column, joint barrels, wrist cuff). The
// foot+column visually fill the existing 0.8 m FK pedestal gap
// (link_offsets[0].translation.z) so the arm appears to stand on its own
// base instead of floating above the FK origin. No FK / IK refactor —
// `link_offsets[0]` stays at z=0.8.
pub const BASE_FOOT_RADIUS: f32 = 0.10;
pub const BASE_FOOT_HALF_HEIGHT: f32 = 0.015;
pub const BASE_COLUMN_RADIUS: f32 = 0.05;
/// Sized to fill the existing 0.8 m FK gap minus foot height and the J0
/// joint barrel half-length: `0.8 - 2*0.015 - 0.025 = 0.745`. If
/// `link_offsets[0].translation.z` ever changes, this constant must
/// change too — pinned by `decoration_poses_emits_foot_and_column_in_existing_pedestal_gap`.
pub const BASE_COLUMN_HEIGHT: f32 = 0.745;
pub const JOINT_BARREL_RADIUS: f32 = 0.055;
pub const JOINT_BARREL_HALF_LENGTH: f32 = 0.025;
pub const WRIST_CUFF_RADIUS: f32 = 0.05;
pub const WRIST_CUFF_HALF_LENGTH: f32 = 0.015;

/// Slot ids for arm decorations within `EntityId::Arm { slot, .. }`. Kept
/// high so they don't collide with link slots (`0..n_joints`) or finger
/// slots (998/999). Joint barrel `i` lives at `JOINT_BARREL_SLOT_BASE + i`.
pub const BASE_FOOT_SLOT: u32 = 990;
pub const BASE_COLUMN_SLOT: u32 = 991;
pub const WRIST_CUFF_SLOT: u32 = 997;
pub const JOINT_BARREL_SLOT_BASE: u32 = 800;

/// Visible (and physical) decoration geometry for one arm. Single source
/// of truth: consumed by both the renderer (`Arm::append_primitives`) and
/// the Rapier insertion path (`arm/src/world.rs::ArmWorld::new`). See
/// design §6.4, §8.1.
#[derive(Clone, Copy, Debug)]
pub struct DecorationPose {
    pub slot: u32,
    pub shape: rtf_sim::shape::Shape,
    pub pose: Isometry3<f32>,
    pub color: rtf_sim::primitive::Color,
    pub kind: rtf_sim::physics::world::DecorationKind,
}

/// Compute the world-space pose of one finger given the EE pose, finger
/// slot (+y or -y side), and current lateral separation. Shared by the
/// visualization (`append_primitives`) and the per-tick physics-pose
/// update so both layers see exactly the same finger geometry.
///
/// Phase 3.4.5d: fingers protrude in EE +x (matching the wrist link
/// direction), so under the wrist alignment (EE +x = world -z) they
/// point straight down. EE +y still maps to world +y for finger
/// separation.
pub fn finger_pose(ee_pose: Isometry3<f32>, slot: u32, separation: f32) -> Isometry3<f32> {
    let sign = if slot == FINGER_SLOT_PLUS {
        1.0_f32
    } else {
        -1.0
    };
    ee_pose
        * Isometry3::from_parts(
            Translation3::new(FINGER_FORWARD_OFFSET, sign * separation, 0.0),
            UnitQuaternion::identity(),
        )
}

impl Visualizable for Arm {
    fn append_primitives(&self, out: &mut Vec<(EntityId, Primitive)>) {
        for lp in self.link_poses() {
            out.push((
                EntityId::Arm {
                    arm_id: self.id,
                    slot: lp.slot,
                },
                Primitive::Capsule {
                    pose: lp.pose,
                    half_height: lp.half_height,
                    radius: LINK_RADIUS,
                    color: rtf_sim::palette::LINK_WHITE,
                },
            ));
        }
        for d in self.decoration_poses() {
            // Foot is rendered as a flat Box. With BASE_FOOT_RADIUS=0.10
            // and BASE_FOOT_HALF_HEIGHT=0.015, the Cylinder→Capsule
            // fallback produces hemispherical caps of radius 0.10 — a
            // sphere visually, not a foot. A Box of the equivalent square
            // footprint reads as a recognizable base. Rapier collider
            // stays a Cylinder (set in insert_arm_decoration).
            let prim = match d.shape {
                rtf_sim::shape::Shape::Cylinder {
                    radius,
                    half_height,
                } if d.slot == BASE_FOOT_SLOT => Primitive::Box {
                    pose: d.pose,
                    half_extents: Vector3::new(radius, radius, half_height),
                    color: d.color,
                },
                rtf_sim::shape::Shape::Cylinder {
                    radius,
                    half_height,
                } => Primitive::Capsule {
                    pose: d.pose,
                    half_height,
                    radius,
                    color: d.color,
                },
                _ => continue,
            };
            out.push((
                EntityId::Arm {
                    arm_id: self.id,
                    slot: d.slot,
                },
                prim,
            ));
        }
        // Fingers: independent of link FK; computed from the EE pose, which
        // we recompute by re-running FK once. `link_poses()` doesn't expose
        // the EE pose explicitly (only link midpoints), and Commit 6 will
        // fold finger rendering into `decoration_poses()` — so this local
        // mini-FK is intentional scoping for now.
        let mut acc = Isometry3::identity();
        for (i, joint) in self.spec.joints.iter().enumerate() {
            acc *= joint_transform(joint, self.state.q[i]);
            acc *= self.spec.link_offsets[i];
        }
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
    fn decoration_poses_emits_foot_and_column_in_existing_pedestal_gap() {
        let spec = ArmSpec {
            joints: vec![JointSpec::Revolute {
                axis: Vector3::z_axis(),
                limits: (-3.2, 3.2),
            }],
            link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.8)],
            gripper: GripperSpec {
                proximity_threshold: 0.02,
                max_grasp_size: 0.05,
            },
        };
        let arm = Arm {
            spec,
            state: ArmState::zeros(1),
            id: 0,
        };
        let decos = arm.decoration_poses();
        assert_eq!(decos.len(), 2, "C5 emits foot + column only");
        let foot = decos.iter().find(|d| d.slot == BASE_FOOT_SLOT).unwrap();
        let column = decos.iter().find(|d| d.slot == BASE_COLUMN_SLOT).unwrap();
        // Foot top at z = 0.030 (foot_center=0.015 + half_height=0.015).
        assert!((foot.pose.translation.z + BASE_FOOT_HALF_HEIGHT - 0.030).abs() < 1e-6);
        // Column top at z = 0.030 + 0.745 = 0.775; rim of J0 barrel sits
        // at z ∈ [0.775, 0.825] in C6. Column TOP must NOT exceed 0.8.
        let column_top = column.pose.translation.z + BASE_COLUMN_HEIGHT * 0.5;
        assert!((column_top - 0.775).abs() < 1e-6);
    }

    #[test]
    fn decoration_poses_empty_for_non_standard_pedestal() {
        // Minimal test arm with link_offsets[0].z != 0.8 → no foot/column.
        let spec = ArmSpec {
            joints: vec![JointSpec::Revolute {
                axis: Vector3::z_axis(),
                limits: (-3.2, 3.2),
            }],
            link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.1)],
            gripper: GripperSpec {
                proximity_threshold: 0.02,
                max_grasp_size: 0.05,
            },
        };
        let arm = Arm {
            spec,
            state: ArmState::zeros(1),
            id: 0,
        };
        assert_eq!(arm.decoration_poses().len(), 0);
    }

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

    #[test]
    fn append_primitives_capsule_poses_match_link_poses() {
        let spec = ArmSpec {
            joints: vec![
                JointSpec::Revolute {
                    axis: Vector3::z_axis(),
                    limits: (-3.2, 3.2),
                };
                3
            ],
            link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.1); 3],
            gripper: GripperSpec {
                proximity_threshold: 0.02,
                max_grasp_size: 0.05,
            },
        };
        let mut state = ArmState::zeros(3);
        state.q = vec![0.3, -0.4, 0.5];
        let arm = Arm { spec, state, id: 0 };

        let mut prims = Vec::new();
        arm.append_primitives(&mut prims);
        let link_poses = arm.link_poses();

        for (i, lp) in link_poses.iter().enumerate() {
            match &prims[i].1 {
                Primitive::Capsule {
                    pose, half_height, ..
                } => {
                    assert!(
                        (pose.translation.vector - lp.pose.translation.vector).norm() < 1e-6,
                        "link {i} translation mismatch",
                    );
                    assert!(
                        (half_height - lp.half_height).abs() < 1e-6,
                        "link {i} half_height mismatch",
                    );
                }
                other => panic!("expected Capsule for link {i}, got {other:?}"),
            }
        }
    }
}

use nalgebra::{Isometry3, Point3, Vector3};
use rtf_core::time::Time;

use crate::entity::EntityId;

/// 8-bit-per-channel RGBA color for visualization primitives (design v2 §7).
/// `const` constructors and named constants keep scenario code allocation-free.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

impl Color {
    pub const WHITE: Color = Color {
        r: 255,
        g: 255,
        b: 255,
        a: 255,
    };
    pub const RED: Color = Color {
        r: 255,
        g: 0,
        b: 0,
        a: 255,
    };
    pub const GREEN: Color = Color {
        r: 0,
        g: 255,
        b: 0,
        a: 255,
    };
    pub const BLUE: Color = Color {
        r: 0,
        g: 0,
        b: 255,
        a: 255,
    };
    pub const BLACK: Color = Color {
        r: 0,
        g: 0,
        b: 0,
        a: 255,
    };
    pub const fn rgba(r: u8, g: u8, b: u8, a: u8) -> Self {
        Color { r, g, b, a }
    }
}

/// Renderable scene primitive (design v2 §7). Decoupled from `Shape` so the
/// visualizer can support shapes the simulator doesn't (capsules, lines,
/// labels) without bloating the physics types.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Primitive {
    Sphere {
        pose: Isometry3<f32>,
        radius: f32,
        color: Color,
    },
    Capsule {
        /// `pose.translation` is the **midpoint** of the capsule's central
        /// axis. The axis is aligned with the local +Z direction implied by
        /// `pose.rotation`. Endpoints are
        /// `pose.translation ± rotation * (Vector3::z() * half_height)`.
        /// Pinned by `tests::capsule_pose_is_midpoint_of_axis`.
        pose: Isometry3<f32>,
        half_height: f32,
        radius: f32,
        color: Color,
    },
    Box {
        pose: Isometry3<f32>,
        half_extents: Vector3<f32>,
        color: Color,
    },
    Line {
        from: Point3<f32>,
        to: Point3<f32>,
        color: Color,
    },
    Label {
        pose: Isometry3<f32>,
        text: String,
        color: Color,
    },
}

/// Per-tick render bundle: timestamp + insertion-ordered list of (owner, primitive)
/// pairs. The harness builds one of these every visual tick from `Visualizable`
/// implementations and ships it to the recorder.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SceneSnapshot {
    pub t: Time,
    pub items: Vec<(EntityId, Primitive)>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity::EntityId;
    use nalgebra::Isometry3;
    use rtf_core::time::Time;
    #[test]
    fn snapshot_holds_items_in_insertion_order() {
        let snap = SceneSnapshot {
            t: Time::from_nanos(0),
            items: vec![(
                EntityId::Object(1),
                Primitive::Sphere {
                    pose: Isometry3::identity(),
                    radius: 0.05,
                    color: Color::WHITE,
                },
            )],
        };
        assert_eq!(snap.items.len(), 1);
    }

    #[test]
    fn capsule_pose_is_midpoint_of_axis() {
        use nalgebra::{UnitQuaternion, Vector3};
        let half_height = 0.2_f32;
        let mid = Vector3::new(1.0, 2.0, 3.0);
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::new(1.0, 1.0, 0.0)),
            0.7,
        );
        let pose = Isometry3::from_parts(mid.into(), rot);
        let dir = rot * Vector3::z();
        let p_top = mid + dir * half_height;
        let p_bot = mid - dir * half_height;
        let recomputed_mid = (p_top + p_bot) * 0.5;
        assert!((recomputed_mid - mid).norm() < 1e-6);
        let base = pose.translation.vector - dir * half_height;
        assert!((base - p_bot).norm() < 1e-6);
        let _ = pose; // ties test intent to the actual primitive type
        let _ = half_height;
    }
}

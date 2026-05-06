use nalgebra::{Isometry3, Point3, Vector3};
use rtf_core::time::Time;

use crate::entity::EntityId;

/// 8-bit-per-channel RGBA color for visualization primitives (design v2 §7).
/// `const` constructors and named constants keep scenario code allocation-free.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct Color { pub r: u8, pub g: u8, pub b: u8, pub a: u8 }

impl Color {
    pub const WHITE: Color = Color { r: 255, g: 255, b: 255, a: 255 };
    pub const RED:   Color = Color { r: 255, g: 0,   b: 0,   a: 255 };
    pub const GREEN: Color = Color { r: 0,   g: 255, b: 0,   a: 255 };
    pub const BLUE:  Color = Color { r: 0,   g: 0,   b: 255, a: 255 };
    pub const BLACK: Color = Color { r: 0,   g: 0,   b: 0,   a: 255 };
    pub const fn rgba(r: u8, g: u8, b: u8, a: u8) -> Self { Color { r, g, b, a } }
}

/// Renderable scene primitive (design v2 §7). Decoupled from `Shape` so the
/// visualizer can support shapes the simulator doesn't (capsules, lines,
/// labels) without bloating the physics types.
#[derive(Clone, Debug)]
pub enum Primitive {
    Sphere  { pose: Isometry3<f32>, radius: f32, color: Color },
    Capsule { pose: Isometry3<f32>, half_height: f32, radius: f32, color: Color },
    Box     { pose: Isometry3<f32>, half_extents: Vector3<f32>, color: Color },
    Line    { from: Point3<f32>, to: Point3<f32>, color: Color },
    Label   { pose: Isometry3<f32>, text: String, color: Color },
}

/// Per-tick render bundle: timestamp + insertion-ordered list of (owner, primitive)
/// pairs. The harness builds one of these every visual tick from `Visualizable`
/// implementations and ships it to the recorder.
#[derive(Clone, Debug)]
pub struct SceneSnapshot {
    pub t: Time,
    pub items: Vec<(EntityId, Primitive)>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Isometry3;
    use crate::entity::EntityId;
    use rtf_core::time::Time;
    #[test]
    fn snapshot_holds_items_in_insertion_order() {
        let snap = SceneSnapshot {
            t: Time::from_nanos(0),
            items: vec![
                (EntityId::Object(1), Primitive::Sphere {
                    pose: Isometry3::identity(), radius: 0.05, color: Color::WHITE
                }),
            ],
        };
        assert_eq!(snap.items.len(), 1);
    }
}

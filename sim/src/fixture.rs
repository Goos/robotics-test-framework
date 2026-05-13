use nalgebra::{Isometry3, Vector2, Vector3};

use crate::{
    entity::EntityId,
    primitive::{Color, Primitive},
    shape::Shape,
    visualizable::Visualizable,
};

/// Immovable simulation entity (design v2 §5.2). Tables, shelves, and other
/// world geometry. `is_support` lets gravity-fall (§5.5) decide whether this
/// fixture's top surface is something objects can rest on.
///
/// v1 keeps `id` as raw `u32` rather than a `FixtureId` newtype because fixtures
/// are referenced from a single home (the `Scene::fixtures` map) and via
/// `SupportId::Fixture(u32)`; promote to a newtype if a second referent appears.
#[derive(Clone, Debug)]
pub struct Fixture {
    pub id: u32,
    pub pose: Isometry3<f32>,
    pub shape: Shape,
    pub is_support: bool,
    pub color: Color,
}

impl Visualizable for Fixture {
    fn append_primitives(&self, out: &mut Vec<(EntityId, Primitive)>) {
        let prim = match &self.shape {
            Shape::Sphere { radius } => Primitive::Sphere {
                pose: self.pose,
                radius: *radius,
                color: self.color,
            },
            Shape::Aabb { half_extents } => Primitive::Box {
                pose: self.pose,
                half_extents: *half_extents,
                color: self.color,
            },
            Shape::Cylinder {
                radius,
                half_height,
            } => Primitive::Capsule {
                pose: self.pose,
                half_height: *half_height,
                radius: *radius,
                color: self.color,
            },
        };
        out.push((EntityId::Fixture(self.id), prim));
    }
}

/// Decompose a bin spec into the (pose, half_extents) of its 5 box pieces:
/// `[floor, +x wall, -x wall, +y wall, -y wall]`.
///
/// `rim_center.translation.z` is the **top z** of the rim — matches today's
/// slab-top z so the place-target z invariant holds (design §7.3). The floor
/// sits `inner_depth` below the rim; walls extend from floor top to rim top.
/// Caller wraps each tuple as a `Fixture` with its own id and color.
pub fn bin_decomposition(
    rim_center: Isometry3<f32>,
    inner_half_extents_xy: Vector2<f32>,
    inner_depth: f32,
    wall_thickness: f32,
) -> [(Isometry3<f32>, Vector3<f32>); 5] {
    let rim_z = rim_center.translation.z;
    let floor_top_z = rim_z - inner_depth;
    let floor_half_thickness = wall_thickness * 0.5;
    let floor_center_z = floor_top_z - floor_half_thickness;
    let wall_center_z = (rim_z + floor_top_z) * 0.5;
    let wall_half_height = inner_depth * 0.5;
    let inner_x = inner_half_extents_xy.x;
    let inner_y = inner_half_extents_xy.y;
    let outer_y = inner_y + wall_thickness;

    let center_xy = rim_center.translation.vector.xy();
    let mk = |dx: f32, dy: f32, dz: f32| {
        let mut p = rim_center;
        p.translation.x = center_xy.x + dx;
        p.translation.y = center_xy.y + dy;
        p.translation.z = dz;
        p
    };
    [
        (
            mk(0.0, 0.0, floor_center_z),
            Vector3::new(inner_x, inner_y, floor_half_thickness),
        ),
        (
            mk(inner_x + wall_thickness * 0.5, 0.0, wall_center_z),
            Vector3::new(wall_thickness * 0.5, outer_y, wall_half_height),
        ),
        (
            mk(-(inner_x + wall_thickness * 0.5), 0.0, wall_center_z),
            Vector3::new(wall_thickness * 0.5, outer_y, wall_half_height),
        ),
        (
            mk(0.0, inner_y + wall_thickness * 0.5, wall_center_z),
            Vector3::new(inner_x, wall_thickness * 0.5, wall_half_height),
        ),
        (
            mk(0.0, -(inner_y + wall_thickness * 0.5), wall_center_z),
            Vector3::new(inner_x, wall_thickness * 0.5, wall_half_height),
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitive::Primitive;
    use crate::shape::Shape;
    use crate::visualizable::Visualizable;
    use nalgebra::{Isometry3, Vector3};

    #[test]
    fn fixture_appends_one_primitive_matching_shape() {
        let f = Fixture {
            id: 0,
            pose: Isometry3::translation(0.0, 0.0, 0.05),
            shape: Shape::Aabb {
                half_extents: Vector3::new(0.5, 0.5, 0.05),
            },
            is_support: true,
            color: Color::WHITE,
        };
        let mut out = Vec::new();
        f.append_primitives(&mut out);
        assert_eq!(out.len(), 1);
        assert!(matches!(out[0].1, Primitive::Box { .. }));
    }
}

#[cfg(test)]
mod bin_tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion};

    #[test]
    fn bin_decomposition_floor_top_is_inner_depth_below_rim() {
        let pose = Isometry3::from_parts(
            Translation3::new(0.5, -0.3, 0.6),
            UnitQuaternion::identity(),
        );
        let parts = bin_decomposition(pose, Vector2::new(0.10, 0.10), 0.08, 0.01);
        let (floor_pose, floor_he) = &parts[0];
        let floor_top_z = floor_pose.translation.z + floor_he.z;
        assert!((floor_top_z - (0.6 - 0.08)).abs() < 1e-6);
    }

    #[test]
    fn bin_decomposition_walls_top_at_rim() {
        let pose =
            Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.6), UnitQuaternion::identity());
        let parts = bin_decomposition(pose, Vector2::new(0.10, 0.10), 0.08, 0.01);
        for (wall_pose, wall_he) in &parts[1..] {
            let top_z = wall_pose.translation.z + wall_he.z;
            assert!(
                (top_z - 0.6).abs() < 1e-6,
                "wall top expected at rim z=0.6, got {top_z}"
            );
        }
    }
}

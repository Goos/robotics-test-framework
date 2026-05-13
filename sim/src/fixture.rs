use nalgebra::Isometry3;

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

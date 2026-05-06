use nalgebra::{Isometry3, Vector3};

use crate::{
    entity::EntityId,
    primitive::{Color, Primitive},
    shape::Shape,
    visualizable::Visualizable,
};

/// Stable identifier for a movable simulation object (design v2 §5.2).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ObjectId(pub u32);

/// Reference to an arm holding an object. Opaque numeric handle; semantics
/// come from the arm crate (Phase 4+).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct ArmRef(pub u32);

/// What a `Settled` object is resting on. Distinguishes fixtures (immovable)
/// from other objects (stacking) without conflating numeric ids across kinds.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum SupportId {
    Fixture(u32),
    Object(u32),
}

/// Lifecycle state of a sim object (design v2 §5.2): `Free` (subject to
/// gravity), `Grasped` (welded to an arm), or `Settled` (resting on a
/// support, not subject to gravity).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ObjectState {
    Free,
    Grasped { by: ArmRef },
    Settled { on: SupportId },
}

/// Movable simulation entity with pose, geometry, mass, and lifecycle state.
#[derive(Clone, Debug)]
pub struct Object {
    pub id: ObjectId,
    pub pose: Isometry3<f32>,
    pub shape: Shape,
    pub mass: f32,
    pub graspable: bool,
    /// Lifecycle state — see `ObjectState`.
    pub state: ObjectState,
    /// Linear velocity in world frame (m/s). v1 gravity-fall (design §5.5) writes
    /// only `lin_vel.z`; lateral components must remain zero.
    pub lin_vel: Vector3<f32>,
}

impl Object {
    pub fn new(
        id: ObjectId,
        pose: Isometry3<f32>,
        shape: Shape,
        mass: f32,
        graspable: bool,
    ) -> Self {
        Self {
            id,
            pose,
            shape,
            mass,
            graspable,
            state: ObjectState::Free,
            lin_vel: Vector3::zeros(),
        }
    }
}

impl Visualizable for Object {
    fn append_primitives(&self, out: &mut Vec<(EntityId, Primitive)>) {
        let prim = match &self.shape {
            Shape::Sphere { radius } => Primitive::Sphere {
                pose: self.pose,
                radius: *radius,
                color: Color::WHITE,
            },
            Shape::Aabb { half_extents } => Primitive::Box {
                pose: self.pose,
                half_extents: *half_extents,
                color: Color::WHITE,
            },
            Shape::Cylinder {
                radius,
                half_height,
            } => Primitive::Capsule {
                pose: self.pose,
                half_height: *half_height,
                radius: *radius,
                color: Color::WHITE,
            },
        };
        out.push((EntityId::Object(self.id.0), prim));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Isometry3;
    #[test]
    fn free_object_constructed() {
        let o = Object::new(
            ObjectId(1),
            Isometry3::translation(0.0, 0.0, 0.05),
            Shape::Sphere { radius: 0.05 },
            /* mass */ 0.10,
            /* graspable */ true,
        );
        assert_eq!(o.state, ObjectState::Free);
        assert!(o.graspable);
    }

    #[test]
    fn object_appends_one_primitive_matching_shape() {
        use super::*;
        use crate::primitive::Primitive;
        use crate::visualizable::Visualizable;
        use nalgebra::Isometry3;
        let o = Object::new(
            ObjectId(1),
            Isometry3::identity(),
            Shape::Sphere { radius: 0.05 },
            0.1,
            true,
        );
        let mut out = Vec::new();
        o.append_primitives(&mut out);
        assert_eq!(out.len(), 1);
        assert!(matches!(out[0].1, Primitive::Sphere { .. }));
    }
}

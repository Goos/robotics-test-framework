use nalgebra::{Isometry3, Vector3};

use crate::shape::Shape;

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
    pub fn new(id: ObjectId, pose: Isometry3<f32>, shape: Shape, mass: f32, graspable: bool) -> Self {
        Self {
            id, pose, shape, mass, graspable,
            state: ObjectState::Free,
            lin_vel: Vector3::zeros(),
        }
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
}

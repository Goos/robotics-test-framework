use nalgebra::Isometry3;

use crate::shape::Shape;

/// Immovable simulation entity (design v2 §5.2). Tables, shelves, and other
/// world geometry. `is_support` lets gravity-fall (§5.5) decide whether this
/// fixture's top surface is something objects can rest on.
#[derive(Clone, Debug)]
pub struct Fixture {
    pub id: u32,
    pub pose: Isometry3<f32>,
    pub shape: Shape,
    pub is_support: bool,
}

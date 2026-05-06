use nalgebra::Isometry3;

use crate::shape::Shape;

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
}

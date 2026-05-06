//! rtf_sim — populated starting in Phase 1.

pub mod entity;
pub mod fixture;
pub mod object;
pub mod primitive;
pub mod scene;
pub mod shape;
pub mod visualizable;

pub use entity::EntityId;
pub use fixture::Fixture;
pub use object::{ArmRef, Object, ObjectId, ObjectState, SupportId};
pub use primitive::{Color, Primitive, SceneSnapshot};
pub use scene::Scene;
pub use shape::Shape;
pub use visualizable::Visualizable;

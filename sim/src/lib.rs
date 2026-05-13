//! rtf_sim — populated starting in Phase 1.

pub mod entity;
pub mod faults;
pub mod fixture;
pub mod object;
pub mod palette;
pub mod physics;
pub mod primitive;
pub mod rate_scheduler;
pub mod recorder;
pub mod runnable_world;
pub mod scene;
pub mod shape;
pub mod sim_clock;
pub mod visualizable;

pub use entity::EntityId;
pub use fixture::Fixture;
pub use object::{ArmRef, Object, ObjectId, ObjectState, SupportId};
pub use primitive::{Color, Primitive, SceneSnapshot};
pub use rate_scheduler::RateScheduler;
#[cfg(feature = "controller_events")]
pub use recorder::ControllerEvent;
pub use recorder::{NullRecorder, Recorder};
pub use runnable_world::{RunnableWorld, SimError};
pub use scene::Scene;
pub use shape::Shape;
pub use sim_clock::SimClock;
pub use visualizable::Visualizable;

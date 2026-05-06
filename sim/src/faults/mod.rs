//! Fault-injection wrappers (design v2 §13). Each wrapper takes any
//! `R: PortReader<T>` and itself impls `PortReader<T>`, so they compose
//! freely. Submodules land one per Phase 9 step.

pub mod delay;
pub mod drop_messages;
pub mod stale_data;

pub use delay::Delay;
pub use drop_messages::DropMessages;
pub use stale_data::StaleData;

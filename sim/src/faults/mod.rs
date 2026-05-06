//! Fault-injection wrappers (design v2 §13). Each wrapper takes any
//! `R: PortReader<T>` and itself impls `PortReader<T>`, so they compose
//! freely. Submodules land one per Phase 9 step.

pub mod delay;
pub mod drop_messages;
pub mod gaussian_noise;
pub mod pcg_noise_source;
pub mod stale_data;

pub use delay::Delay;
pub use drop_messages::DropMessages;
pub use gaussian_noise::GaussianNoise;
pub use pcg_noise_source::PcgNoiseSource;
pub use stale_data::StaleData;

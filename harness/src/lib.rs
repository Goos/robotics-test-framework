//! rtf_harness — populated starting in Phase 1.

pub mod run;
pub mod types;

pub use run::run;
pub use types::{RunConfig, RunResult, Termination};

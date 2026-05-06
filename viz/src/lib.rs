//! rtf_viz — visualizer adapters for the rtf framework.
//!
//! Per design v2 §7: this crate sees only `rtf_sim::primitive::Primitive`,
//! never robot-kind crates. Default build has no external deps; `--features
//! rerun` pulls in the rerun SDK for live + offline visualization.

pub mod file_recorder;

#[cfg(feature = "rerun")]
pub mod rerun_recorder;

pub use file_recorder::FileRecorder;

#[cfg(feature = "rerun")]
pub use rerun_recorder::RerunRecorder;

#[cfg(test)]
mod tests {
    #[test]
    fn viz_compiles_without_rerun_feature() {
        // No-op test; passes when default-feature build compiles.
    }
}

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

/// Construct a `RerunRecorder` writing to `$TMPDIR/<test_name>.rrd`. Used
/// by example scenarios so each `#[test]` can opt into recording with one
/// line, conditional on the `rerun` feature being on.
///
/// Removes any pre-existing rrd at the path first so re-runs don't append
/// stale data. Prints the resolved path via `eprintln!` so test output
/// surfaces it. Returns `None` if the rerun SDK refused to construct the
/// recorder (e.g. file-system error) — callers fall back to no recorder.
///
/// Lives behind `feature = "rerun"` because it names `RerunRecorder` in
/// its return type. Call sites use `#[cfg(feature = "viz-rerun")]` (the
/// arm-crate feature that pulls in `rtf_viz/rerun`) to gate the call.
#[cfg(feature = "rerun")]
pub fn maybe_recorder_for(test_name: &str) -> Option<RerunRecorder> {
    let path = std::env::temp_dir().join(format!("{test_name}.rrd"));
    let _ = std::fs::remove_file(&path);
    eprintln!("rrd will be saved to: {:?}", path);
    RerunRecorder::save_to_file(&path, test_name).ok()
}

#[cfg(test)]
mod tests {
    #[test]
    fn viz_compiles_without_rerun_feature() {
        // No-op test; passes when default-feature build compiles.
    }
}

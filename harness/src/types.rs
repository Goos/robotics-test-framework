use rtf_core::{
    controller::ControlError,
    score::Score,
    time::{Duration, Time},
};
use rtf_sim::recorder::{NullRecorder, Recorder};

/// Per-run configuration for the harness driver (design v2 §6). The
/// `Recorder` type parameter defaults to `NullRecorder` so callers that
/// don't need recording get the cheap no-op without any extra typing.
pub struct RunConfig<R: Recorder = NullRecorder> {
    pub tick_rate_hz: u32,
    pub deadline: Duration,
    pub seed: u64,
    pub recorder: R,
}

/// What the harness emits when a run finishes (design v2 §6).
pub struct RunResult {
    pub score: Score,
    pub final_time: Time,
    pub terminated_by: Termination,
}

/// Reason the harness loop ended this run (design v2 §6).
#[derive(Debug, Clone)]
pub enum Termination {
    GoalComplete,
    Deadline,
    ControllerError(ControlError),
}

impl Default for RunConfig<NullRecorder> {
    fn default() -> Self {
        Self {
            tick_rate_hz: 1000,
            deadline: Duration::from_secs(10),
            seed: 0,
            recorder: NullRecorder,
        }
    }
}

impl<R: Recorder> RunConfig<R> {
    pub fn with_tick_rate(mut self, hz: u32) -> Self {
        self.tick_rate_hz = hz;
        self
    }
    pub fn with_deadline(mut self, dt: Duration) -> Self {
        self.deadline = dt;
        self
    }
    pub fn with_seed(mut self, seed: u64) -> Self {
        self.seed = seed;
        self
    }

    /// Replace the recorder, returning a `RunConfig<NewR>`. Type-changing so
    /// callers can swap the default `NullRecorder` for a real implementation
    /// without losing any other configuration.
    pub fn with_recorder<NewR: Recorder>(self, recorder: NewR) -> RunConfig<NewR> {
        RunConfig {
            tick_rate_hz: self.tick_rate_hz,
            deadline: self.deadline,
            seed: self.seed,
            recorder,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rtf_core::time::Duration;
    use rtf_sim::recorder::NullRecorder;
    #[test]
    fn run_config_defaults() {
        let cfg: RunConfig<NullRecorder> = RunConfig::default();
        assert_eq!(cfg.tick_rate_hz, 1000);
        assert_eq!(cfg.deadline, Duration::from_secs(10));
    }
}

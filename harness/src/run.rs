use rtf_core::{controller::Controller, goal::Goal};
use rtf_sim::{recorder::Recorder, runnable_world::RunnableWorld};

use crate::types::{RunConfig, RunResult, Termination};

/// Drive a `RunnableWorld` to completion. Returns a `RunResult` summarizing
/// the run's score, final time, and termination reason (design v2 §6).
///
/// Step 4.2 skeleton: this returns immediately with a `Deadline` termination
/// and a goal-evaluated score from the initial world state. The tick loop
/// (controller step interleaved between `publish_sensors` and
/// `consume_actuators_and_integrate`) lands in Step 4.3.
pub fn run<W, C, G, R>(
    world: W,
    _controller: C,
    goal: G,
    _cfg: RunConfig<R>,
) -> RunResult
where
    W: RunnableWorld,
    C: Controller,
    G: Goal<W>,
    R: Recorder,
{
    RunResult {
        score: goal.evaluate(&world),
        final_time: world.time(),
        terminated_by: Termination::Deadline,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rtf_core::{
        controller::{ControlError, Controller},
        goal::Goal,
        score::Score,
        time::{Duration, Time},
        world_view::WorldView,
    };
    use rtf_sim::{primitive::SceneSnapshot, runnable_world::RunnableWorld};

    use crate::types::{RunConfig, Termination};

    struct W { t: Time }
    impl WorldView for W {}
    impl RunnableWorld for W {
        fn publish_sensors(&mut self) {}
        fn consume_actuators_and_integrate(&mut self, dt: Duration) { self.t = self.t + dt; }
        fn snapshot(&self) -> SceneSnapshot { SceneSnapshot { t: self.t, items: vec![] } }
        fn time(&self) -> Time { self.t }
    }
    struct Noop;
    impl Controller for Noop {
        fn step(&mut self, _: Time) -> Result<(), ControlError> { Ok(()) }
    }
    struct AlwaysHalf;
    impl Goal<W> for AlwaysHalf {
        fn evaluate(&self, _: &W) -> Score { Score::new(0.5) }
    }

    #[test]
    fn zero_deadline_terminates_immediately_with_evaluated_score() {
        let cfg = RunConfig::default().with_deadline(Duration::from_nanos(0));
        let res = run(W { t: Time::ZERO }, Noop, AlwaysHalf, cfg);
        assert!(matches!(res.terminated_by, Termination::Deadline));
        assert_eq!(res.score.value, 0.5);
    }
}

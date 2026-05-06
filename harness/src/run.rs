use rtf_core::{
    controller::{ControlErrorKind, Controller},
    goal::Goal,
    time::{Duration, Time},
};
use rtf_sim::{recorder::Recorder, runnable_world::RunnableWorld};

use crate::types::{RunConfig, RunResult, Termination};

/// Drive a `RunnableWorld` to completion (design v2 §6). The tick loop is
/// `publish_sensors → recorder.record(snapshot) → goal.tick → controller.step
/// → consume_actuators_and_integrate(dt)`. Termination order each iteration:
/// (1) deadline reached, (2) goal complete, (3) controller error.
///
/// Single-threaded (design v2 §10.3); no global state.
pub fn run<W, C, G, R>(
    mut world: W,
    mut controller: C,
    mut goal: G,
    mut cfg: RunConfig<R>,
) -> RunResult
where
    W: RunnableWorld,
    C: Controller,
    G: Goal<W>,
    R: Recorder,
{
    let dt_ns = 1_000_000_000_i64 / cfg.tick_rate_hz as i64;
    let dt = Duration::from_nanos(dt_ns);
    let deadline_time = Time::from_nanos(cfg.deadline.as_nanos());

    let terminated_by = loop {
        if world.time() >= deadline_time { break Termination::Deadline; }
        if goal.is_complete(&world) { break Termination::GoalComplete; }

        world.publish_sensors();
        cfg.recorder.record(&world.snapshot());
        goal.tick(world.time(), &world);

        match controller.step(world.time()) {
            Ok(()) => {}
            Err(e) if e.kind == ControlErrorKind::Recoverable => {
                // Transient — keep ticking. (Phase 9: optionally surface via
                // `recorder.record_event(...)` when `controller_events` is on.)
            }
            Err(e) => break Termination::ControllerError(e),
        }

        world.consume_actuators_and_integrate(dt);
    };

    RunResult {
        score: goal.evaluate(&world),
        final_time: world.time(),
        terminated_by,
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

    #[test]
    fn n_ticks_advance_world_time_by_n_dt() {
        let cfg = RunConfig::default()
            .with_deadline(Duration::from_millis(5))
            .with_tick_rate(1000);
        let res = run(W { t: Time::ZERO }, Noop, AlwaysHalf, cfg);
        assert_eq!(res.final_time, Time::from_millis(5));
    }

    #[test]
    fn run_terminates_on_goal_complete_before_deadline() {
        struct DoneAt(Time);
        impl Goal<W> for DoneAt {
            fn is_complete(&self, w: &W) -> bool { w.time() >= self.0 }
            fn evaluate(&self, _: &W) -> Score { Score::new(1.0) }
        }
        let cfg = RunConfig::default().with_deadline(Duration::from_millis(10));
        let res = run(W { t: Time::ZERO }, Noop, DoneAt(Time::from_millis(3)), cfg);
        assert!(matches!(res.terminated_by, Termination::GoalComplete));
        assert!(res.final_time >= Time::from_millis(3));
    }
}

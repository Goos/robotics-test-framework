use std::borrow::Cow;

use rtf_core::{
    time::{Duration, Time},
    world_view::WorldView,
};

use crate::primitive::SceneSnapshot;

/// Errors that can arise during a sim tick. Carries a `Cow<'static, str>` so
/// callers can pass either a borrowed `&'static str` literal (zero-alloc) or
/// an owned `String` for context-rich messages — same shape as `ControlError`.
#[derive(Debug, Clone)]
pub struct SimError {
    pub detail: Cow<'static, str>,
}

/// A simulated world that can be ticked, snapshotted, and queried.
///
/// All four core methods are required; `tick` is provided as a default that
/// calls `publish_sensors` then `consume_actuators_and_integrate(dt)` —
/// `harness::run` uses these directly so it can interleave a controller step
/// between them. Direct callers (no controller) get `tick` as a convenience.
pub trait RunnableWorld: WorldView {
    fn publish_sensors(&mut self);
    fn consume_actuators_and_integrate(&mut self, dt: Duration);
    fn snapshot(&self) -> SceneSnapshot;
    fn time(&self) -> Time;

    fn tick(&mut self, dt: Duration) -> Result<(), SimError> {
        self.publish_sensors();
        self.consume_actuators_and_integrate(dt);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitive::SceneSnapshot;
    use rtf_core::{
        time::{Duration, Time},
        world_view::WorldView,
    };

    struct EmptyWorld {
        t: Time,
    }
    impl WorldView for EmptyWorld {}
    impl RunnableWorld for EmptyWorld {
        fn publish_sensors(&mut self) { /* no sensors */
        }
        fn consume_actuators_and_integrate(&mut self, dt: Duration) {
            self.t = self.t + dt;
        }
        fn snapshot(&self) -> SceneSnapshot {
            SceneSnapshot {
                t: self.t,
                items: vec![],
            }
        }
        fn time(&self) -> Time {
            self.t
        }
    }

    #[test]
    fn default_tick_advances_time_via_consume() {
        let mut w = EmptyWorld { t: Time::ZERO };
        w.tick(Duration::from_millis(1)).unwrap();
        assert_eq!(w.time(), Time::from_millis(1));
    }
}

use core::cell::Cell;

use rtf_core::{clock::Clock, time::{Duration, Time}};

/// Sim-driven clock backed by `Cell<Time>` (single-threaded; safe).
/// Use `advance(dt)` to step time; `harness::run` will own this and tick it
/// at the configured rate.
pub struct SimClock {
    now: Cell<Time>,
}

impl SimClock {
    pub fn new() -> Self {
        Self { now: Cell::new(Time::ZERO) }
    }

    pub fn advance(&self, dt: Duration) {
        self.now.set(self.now.get() + dt);
    }
}

impl Default for SimClock {
    fn default() -> Self { Self::new() }
}

impl Clock for SimClock {
    fn now(&self) -> Time {
        self.now.get()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rtf_core::clock::Clock;
    use rtf_core::time::{Time, Duration};
    #[test]
    fn sim_clock_advances_via_advance() {
        let c = SimClock::new();
        assert_eq!(c.now(), Time::ZERO);
        c.advance(Duration::from_millis(3));
        assert_eq!(c.now(), Time::from_millis(3));
    }
}

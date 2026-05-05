use core::cell::Cell;

use crate::time::{Duration, Time};

/// Source of monotonic simulated time. Inject into anything that needs `now()`
/// — never call `std::time::Instant::now()` in core/sim (design §10.2).
///
/// Single-threaded by design v2 §10.3; the trait does not require `Send`/`Sync`.
/// The eventual real-hardware `MonotonicClock` will fit the same shape.
pub trait Clock {
    fn now(&self) -> Time;
}

/// Test clock with explicit, single-threaded time control. Use `&self`
/// `advance` so callers can pass `&FakeClock` everywhere.
pub struct FakeClock {
    now: Cell<Time>,
}

impl FakeClock {
    pub fn new(start: Time) -> Self {
        Self { now: Cell::new(start) }
    }

    pub fn advance(&self, dt: Duration) {
        self.now.set(self.now.get() + dt);
    }
}

impl Clock for FakeClock {
    fn now(&self) -> Time {
        self.now.get()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::Time;

    #[test]
    fn fake_clock_advances_under_test_control() {
        let clock = FakeClock::new(Time::from_nanos(0));
        assert_eq!(clock.now(), Time::from_nanos(0));
        clock.advance(crate::time::Duration::from_millis(5));
        assert_eq!(clock.now(), Time::from_millis(5));
    }
}

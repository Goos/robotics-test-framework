/// Phase-accumulator scheduler for multi-rate sensor publishing.
///
/// Per design v2 §5.6: accumulates real elapsed time and fires once per period.
/// Long-run firing rate matches the requested Hz exactly; per-tick timing has
/// ±1 sim-tick jitter for non-divisible rates (e.g. 333 Hz vs 1 kHz sim).
pub struct RateScheduler {
    period_ns: i64,
    accumulator_ns: i64,
}

impl RateScheduler {
    pub fn new_hz(hz: u32) -> Self {
        Self {
            period_ns: 1_000_000_000_i64 / hz as i64,
            accumulator_ns: 0,
        }
    }

    /// Advance accumulator by `dt_ns`. Returns `true` if a fire is scheduled
    /// for this tick (subtracts one period from the accumulator).
    pub fn tick(&mut self, dt_ns: i64) -> bool {
        self.accumulator_ns += dt_ns;
        if self.accumulator_ns >= self.period_ns {
            self.accumulator_ns -= self.period_ns;
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn long_run_rate_matches_requested() {
        // 333 Hz against 1 kHz sim: ~1000 ticks should yield ~333 fires (±1).
        let mut sched = RateScheduler::new_hz(333);
        let dt_ns = 1_000_000_i64; // 1 ms
        let mut fires: i32 = 0;
        for _ in 0..1_000 {
            if sched.tick(dt_ns) {
                fires += 1;
            }
        }
        assert!((fires - 333).abs() <= 1, "got {} fires", fires);
    }

    #[test]
    fn divisible_rate_fires_exactly_on_period() {
        let mut sched = RateScheduler::new_hz(100);
        let dt_ns = 1_000_000_i64;
        let mut fires: i32 = 0;
        for _ in 0..1_000 {
            if sched.tick(dt_ns) {
                fires += 1;
            }
        }
        assert_eq!(fires, 100);
    }
}

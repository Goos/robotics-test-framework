//! `StaleData<R, T>` — drops sensor readings whose `sampled_at` is older
//! than `max_age` relative to the supplied clock. Models the case where a
//! controller picks up a buffered reading that no longer reflects reality
//! (e.g. a slow encoder bus).

use std::rc::Rc;

use rtf_core::{
    clock::Clock,
    port::PortReader,
    sensor_reading::SensorReading,
    time::Duration,
};

pub struct StaleData<R: PortReader<T>, T: Clone + SensorReading> {
    inner: R,
    max_age: Duration,
    clock: Rc<dyn Clock>,
    _phantom: core::marker::PhantomData<T>,
}

impl<R: PortReader<T>, T: Clone + SensorReading> StaleData<R, T> {
    pub fn new(inner: R, max_age: Duration, clock: Rc<dyn Clock>) -> Self {
        Self {
            inner,
            max_age,
            clock,
            _phantom: core::marker::PhantomData,
        }
    }

    fn is_stale(&self, r: &T) -> bool {
        let age = self.clock.now() - r.sampled_at();
        age > self.max_age
    }
}

impl<R: PortReader<T>, T: Clone + SensorReading> PortReader<T> for StaleData<R, T> {
    fn latest(&self) -> Option<T> {
        let r = self.inner.latest()?;
        if self.is_stale(&r) {
            None
        } else {
            Some(r)
        }
    }

    fn take(&mut self) -> Option<T> {
        let r = self.inner.take()?;
        if self.is_stale(&r) {
            None
        } else {
            Some(r)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim_clock::SimClock;
    use rtf_core::{
        clock::Clock,
        port::port,
        sensor_reading::SensorReading,
        time::{Duration, Time},
    };
    use std::rc::Rc;

    #[derive(Clone)]
    struct R {
        sampled_at: Time,
    }
    impl SensorReading for R {
        fn sampled_at(&self) -> Time {
            self.sampled_at
        }
    }

    #[test]
    fn stale_data_rejects_old_readings() {
        let raw_clock = Rc::new(SimClock::new());
        let clock_dyn: Rc<dyn Clock> = raw_clock.clone();
        let (tx, rx) = port::<R>();
        let stale = StaleData::new(rx, Duration::from_millis(50), clock_dyn);

        tx.send(R { sampled_at: Time::from_millis(0) });
        raw_clock.advance(Duration::from_millis(60));
        assert!(stale.latest().is_none(), "60ms-old reading should be stale (max_age=50ms)");

        tx.send(R { sampled_at: Time::from_millis(60) });
        assert!(stale.latest().is_some(), "fresh reading at sampled_at=now should pass");
    }

    #[test]
    fn boundary_age_equal_to_max_age_passes() {
        let raw_clock = Rc::new(SimClock::new());
        let clock_dyn: Rc<dyn Clock> = raw_clock.clone();
        let (tx, rx) = port::<R>();
        let mut stale = StaleData::new(rx, Duration::from_millis(10), clock_dyn);

        tx.send(R { sampled_at: Time::from_millis(0) });
        raw_clock.advance(Duration::from_millis(10));
        // Boundary: age == max_age is NOT stale (strict inequality in is_stale).
        assert!(stale.take().is_some());
    }
}

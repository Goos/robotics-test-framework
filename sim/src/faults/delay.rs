//! `Delay<R, T>` — releases each message only after `latency` of sim time
//! has elapsed since it was first observed on the inner port. The buffer
//! lives behind `RefCell` so `latest(&self)` can drain the inner port
//! without taking a mutable borrow of the wrapper (single-threaded contract,
//! design v2 §10.2).

use core::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;

use rtf_core::{
    clock::Clock,
    port::PortReader,
    time::{Duration, Time},
};

pub struct Delay<R: PortReader<T>, T: Clone> {
    inner: RefCell<R>,
    buffer: RefCell<VecDeque<(Time, T)>>,
    latency: Duration,
    clock: Rc<dyn Clock>,
}

impl<R: PortReader<T>, T: Clone> Delay<R, T> {
    pub fn new(inner: R, latency: Duration, clock: Rc<dyn Clock>) -> Self {
        Self {
            inner: RefCell::new(inner),
            buffer: RefCell::new(VecDeque::new()),
            latency,
            clock,
        }
    }

    /// Drain whatever the inner port currently holds into the buffer, stamped
    /// with its release_time = now() + latency. Call before peeking/popping.
    fn pump(&self) {
        let mut inner = self.inner.borrow_mut();
        let mut buf = self.buffer.borrow_mut();
        while let Some(v) = inner.take() {
            let release_time = self.clock.now() + self.latency;
            buf.push_back((release_time, v));
        }
    }
}

impl<R: PortReader<T>, T: Clone> PortReader<T> for Delay<R, T> {
    fn latest(&self) -> Option<T> {
        self.pump();
        let buf = self.buffer.borrow();
        let now = self.clock.now();
        // Walk forward to the *most recent* entry whose release_time has
        // already elapsed — LatestWins semantics on the released subset.
        buf.iter()
            .rev()
            .find(|(release_time, _)| *release_time <= now)
            .map(|(_, v)| v.clone())
    }

    fn take(&mut self) -> Option<T> {
        self.pump();
        let mut buf = self.buffer.borrow_mut();
        let now = self.clock.now();
        // Drop any pending entries up to and including the most recent one
        // that has already been released — LatestWins: only the latest
        // released value matters, older ones are discarded.
        let mut latest: Option<T> = None;
        while let Some((release_time, _)) = buf.front() {
            if *release_time > now {
                break;
            }
            latest = buf.pop_front().map(|(_, v)| v);
        }
        latest
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim_clock::SimClock;
    use rtf_core::{
        clock::Clock,
        port::{port, PortReader},
        time::Duration,
    };
    use std::rc::Rc;

    #[test]
    fn delay_releases_value_only_after_latency_elapses() {
        let raw_clock = Rc::new(SimClock::new());
        let clock_dyn: Rc<dyn Clock> = raw_clock.clone();
        let (tx, rx) = port::<i32>();
        let mut delayed = Delay::new(rx, Duration::from_millis(2), clock_dyn);
        tx.send(7);
        // Time hasn't advanced — value is in the buffer but not released.
        assert!(delayed.latest().is_none());
        raw_clock.advance(Duration::from_millis(2));
        assert_eq!(delayed.latest(), Some(7));
        // latest is non-destructive; same value still available.
        assert_eq!(delayed.latest(), Some(7));
        // take() drains the released entry.
        assert_eq!(delayed.take(), Some(7));
        assert_eq!(delayed.take(), None);
    }

    #[test]
    fn buffer_holds_two_releases_at_distinct_times() {
        // Pump explicitly between sends so both values land in the buffer
        // (the inner port is LatestWins single-slot, so a second send
        // before the wrapper pumps would just overwrite the first).
        let raw_clock = Rc::new(SimClock::new());
        let clock_dyn: Rc<dyn Clock> = raw_clock.clone();
        let (tx, rx) = port::<i32>();
        let mut delayed = Delay::new(rx, Duration::from_millis(5), clock_dyn);
        tx.send(1);
        // pump now — release_time = 5ms.
        let _ = delayed.latest();
        raw_clock.advance(Duration::from_millis(3));
        tx.send(2);
        // pump now — release_time = 3 + 5 = 8ms.
        let _ = delayed.latest();
        raw_clock.advance(Duration::from_millis(2)); // now = 5ms; only entry 1 released.
        // take() collapses to the most recent released entry — value 1.
        assert_eq!(delayed.take(), Some(1));
        assert_eq!(delayed.take(), None);
        raw_clock.advance(Duration::from_millis(3)); // now = 8ms; entry 2 released.
        assert_eq!(delayed.take(), Some(2));
    }
}

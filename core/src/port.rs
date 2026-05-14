use core::cell::{Cell, RefCell};
use std::rc::Rc;

use crate::time::{Duration, Time};

/// Read side of a single-slot LatestWins port. Implemented by the concrete
/// `PortRx<T>` and (in later phases) by fault-injection wrappers.
/// `take` requires `&mut` to enforce single-consumer at the borrow checker.
pub trait PortReader<T> {
    fn latest(&self) -> Option<T>;
    fn take(&mut self) -> Option<T>;
    fn age_at(&self, _now: Time) -> Option<Duration> {
        None
    }
}

/// Producer end of a `port::<T>()`. `Clone`-able by design — many-producer,
/// single-consumer; cloning bumps the inner `Rc` refcount and does not require
/// `T: Clone`.
#[derive(Clone)]
pub struct PortTx<T> {
    slot: Rc<RefCell<Option<T>>>,
    stamp: Rc<Cell<Option<Time>>>,
}

/// Consumer end of a `port::<T>()`. **Intentionally not `Clone`** to enforce
/// the single-consumer invariant (design v2 §4 amendment).
pub struct PortRx<T> {
    slot: Rc<RefCell<Option<T>>>,
    stamp: Rc<Cell<Option<Time>>>,
}

/// Construct a single-slot LatestWins channel: each `send` overwrites the
/// previous value, `latest` peeks non-destructively, `take` drains.
///
/// `T: Send` is intentionally OMITTED — the framework is single-threaded
/// (design §10.2); requiring `Send` would block useful non-`Send` payloads.
pub fn port<T: Clone + 'static>() -> (PortTx<T>, PortRx<T>) {
    let slot = Rc::new(RefCell::new(None));
    let stamp = Rc::new(Cell::new(None));
    (
        PortTx {
            slot: Rc::clone(&slot),
            stamp: Rc::clone(&stamp),
        },
        PortRx { slot, stamp },
    )
}

impl<T> PortTx<T> {
    /// Overwrites any previously-sent value (LatestWins semantics).
    pub fn send(&self, v: T) {
        *self.slot.borrow_mut() = Some(v);
    }

    /// Overwrites value and records `at` as the send timestamp for age tracking.
    pub fn send_at(&self, v: T, at: Time) {
        *self.slot.borrow_mut() = Some(v);
        self.stamp.set(Some(at));
    }
}

impl<T: Clone> PortRx<T> {
    pub fn latest(&self) -> Option<T> {
        self.slot.borrow().clone()
    }
    pub fn take(&mut self) -> Option<T> {
        self.slot.borrow_mut().take()
    }
    pub fn age_at(&self, now: Time) -> Option<Duration> {
        self.stamp.get().map(|sent| now - sent)
    }
}

impl<T: Clone> PortReader<T> for PortRx<T> {
    fn latest(&self) -> Option<T> {
        Self::latest(self)
    }
    fn take(&mut self) -> Option<T> {
        Self::take(self)
    }
    fn age_at(&self, now: Time) -> Option<Duration> {
        Self::age_at(self, now)
    }
}

/// Blanket impl so `Box<dyn PortReader<T>>` (and any other smart-pointer
/// boxing) satisfies the `PortReader<T>` bound. Phase 9 fault wrappers compose
/// into heterogeneous types, and erasing them as `Box<dyn PortReader<T>>` is
/// the cleanest way to hand them to a controller that's generic over `R`.
impl<T, R: ?Sized + PortReader<T>> PortReader<T> for Box<R> {
    fn latest(&self) -> Option<T> {
        (**self).latest()
    }
    fn take(&mut self) -> Option<T> {
        (**self).take()
    }
    fn age_at(&self, now: Time) -> Option<Duration> {
        (**self).age_at(now)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn send_overwrites_then_latest_is_non_destructive() {
        let (tx, rx) = port::<i32>();
        tx.send(1);
        tx.send(2); // overwrites 1
        assert_eq!(rx.latest(), Some(2)); // peek
        assert_eq!(rx.latest(), Some(2)); // still there
    }

    #[test]
    fn take_clears_the_slot() {
        let (tx, mut rx) = port::<i32>();
        tx.send(7);
        assert_eq!(rx.take(), Some(7));
        assert_eq!(rx.take(), None);
        assert_eq!(rx.latest(), None);
    }

    #[test]
    fn empty_port_returns_none() {
        let (_tx, rx) = port::<i32>();
        assert_eq!(rx.latest(), None);
    }

    #[test]
    fn port_rx_implements_port_reader_trait() {
        let (tx, mut rx) = port::<i32>();
        tx.send(42);
        fn pull<R: PortReader<i32>>(r: &mut R) -> Option<i32> {
            r.take()
        }
        assert_eq!(pull(&mut rx), Some(42));

        // Object-safety regression guard: PortReader<T> must stay dyn-compatible
        // — Phase 9 fault wrappers depend on it. Adding a generic method to
        // PortReader later would silently break this line.
        let _: Box<dyn PortReader<i32>> = Box::new(port::<i32>().1);
    }

    #[test]
    fn boxed_dyn_port_reader_satisfies_bound() {
        let (tx, rx) = port::<i32>();
        tx.send(42);
        let mut boxed: Box<dyn PortReader<i32>> = Box::new(rx);
        fn pull<R: PortReader<i32>>(r: &mut R) -> Option<i32> {
            r.take()
        }
        assert_eq!(pull(&mut boxed), Some(42));
    }
}

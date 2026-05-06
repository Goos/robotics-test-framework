//! `DropMessages<R, T>` — lossy channel wrapper. Each `take()` / `latest()`
//! draws a uniform `[0, 1)` value from a seeded `Pcg64`; if the draw is
//! below `drop_rate`, the inner port's value is fetched and discarded
//! (returning `None`). Otherwise the value passes through unchanged.

use core::cell::RefCell;
use rand::{Rng, SeedableRng};
use rand_pcg::Pcg64;

use rtf_core::port::PortReader;

pub struct DropMessages<R: PortReader<T>, T: Clone> {
    inner: R,
    rng: RefCell<Pcg64>,
    drop_rate: f32,
    _phantom: core::marker::PhantomData<T>,
}

impl<R: PortReader<T>, T: Clone> DropMessages<R, T> {
    pub fn new(inner: R, drop_rate: f32, seed: u64) -> Self {
        Self {
            inner,
            rng: RefCell::new(Pcg64::seed_from_u64(seed)),
            drop_rate,
            _phantom: core::marker::PhantomData,
        }
    }

    fn should_drop(&self) -> bool {
        let r: f32 = self.rng.borrow_mut().gen();
        r < self.drop_rate
    }
}

impl<R: PortReader<T>, T: Clone> PortReader<T> for DropMessages<R, T> {
    fn latest(&self) -> Option<T> {
        if self.should_drop() {
            None
        } else {
            self.inner.latest()
        }
    }

    fn take(&mut self) -> Option<T> {
        if self.should_drop() {
            // Drain the inner slot too — otherwise a "drop" still leaves the
            // value behind to be returned on the next non-dropping draw.
            let _ = self.inner.take();
            None
        } else {
            self.inner.take()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rtf_core::port::port;

    #[test]
    fn drop_messages_drops_about_the_specified_fraction() {
        let (tx, rx) = port::<i32>();
        let mut dropper = DropMessages::new(rx, 0.5, 42);
        let mut received = 0;
        for v in 0..1000 {
            tx.send(v);
            if dropper.take().is_some() {
                received += 1;
            }
        }
        assert!(
            (received - 500_i32).abs() < 80,
            "received {received} (expected approx 500)",
        );
    }

    #[test]
    fn drop_rate_zero_passes_everything() {
        let (tx, rx) = port::<i32>();
        let mut dropper = DropMessages::new(rx, 0.0, 1);
        for v in 0..10 {
            tx.send(v);
            assert_eq!(dropper.take(), Some(v));
        }
    }

    #[test]
    fn drop_rate_one_drops_everything() {
        let (tx, rx) = port::<i32>();
        let mut dropper = DropMessages::new(rx, 1.0, 1);
        for v in 0..10 {
            tx.send(v);
            assert_eq!(dropper.take(), None);
        }
    }
}

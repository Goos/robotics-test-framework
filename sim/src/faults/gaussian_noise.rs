//! `GaussianNoise<R, T>` — perturbs each pulled reading in-place via the
//! `Noise` trait. The internal `Pcg64` is seeded explicitly so two runs with
//! the same seed see byte-identical noise sequences (determinism, design §10).

use core::cell::RefCell;
use rand::SeedableRng;
use rand_pcg::Pcg64;

use rtf_core::{port::PortReader, sensor_reading::Noise};

pub struct GaussianNoise<R: PortReader<T>, T: Clone + Noise> {
    inner: R,
    rng: RefCell<Pcg64>,
    stddev: f32,
    _phantom: core::marker::PhantomData<T>,
}

impl<R: PortReader<T>, T: Clone + Noise> GaussianNoise<R, T> {
    pub fn new(inner: R, stddev: f32, seed: u64) -> Self {
        Self {
            inner,
            rng: RefCell::new(Pcg64::seed_from_u64(seed)),
            stddev,
            _phantom: core::marker::PhantomData,
        }
    }
}

impl<R: PortReader<T>, T: Clone + Noise> PortReader<T> for GaussianNoise<R, T> {
    fn latest(&self) -> Option<T> {
        let mut r = self.inner.latest()?;
        r.apply_noise(&mut self.rng.borrow_mut(), self.stddev);
        Some(r)
    }

    fn take(&mut self) -> Option<T> {
        let mut r = self.inner.take()?;
        r.apply_noise(&mut self.rng.borrow_mut(), self.stddev);
        Some(r)
    }
}

#[cfg(test)]
mod tests {
    // Note: unit-tested with arm::JointEncoderReading via an integration-style
    // test in the arm crate (see arm/src/ports.rs); here we test the wrapper
    // mechanics in isolation against a tiny stub Noise type so we don't pull
    // arm into sim's dep graph.
    use super::*;
    use rtf_core::{port::port, sensor_reading::Noise};

    #[derive(Clone)]
    struct Stub {
        x: f32,
    }
    impl Noise for Stub {
        fn apply_noise(&mut self, _rng: &mut rand_pcg::Pcg64, stddev: f32) {
            // Deterministic stand-in for a real Gaussian — just so we can
            // verify the wrapper threads stddev through and mutates the value.
            self.x += stddev;
        }
    }

    #[test]
    fn gaussian_noise_perturbs_value_via_apply_noise() {
        let (tx, rx) = port::<Stub>();
        let mut noisy = GaussianNoise::new(rx, 0.1_f32, 1);
        tx.send(Stub { x: 1.0 });
        let r = noisy.take().unwrap();
        assert!((r.x - 1.1).abs() < 1e-6, "expected x=1.1, got {}", r.x);
    }

    #[test]
    fn empty_inner_yields_none() {
        let (_tx, rx) = port::<Stub>();
        let noisy = GaussianNoise::new(rx, 0.1_f32, 1);
        assert!(noisy.latest().is_none());
    }
}

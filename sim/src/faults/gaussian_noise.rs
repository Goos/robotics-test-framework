//! `GaussianNoise<R, T>` — perturbs each pulled reading in-place via the
//! `Noise` trait, driving the noise from a seeded `PcgNoiseSource`. The
//! `NoiseSource` abstraction means readings never need to name a concrete
//! RNG type; only this wrapper imports `rand_pcg`/`rand_distr`.

use core::cell::RefCell;

use rtf_core::{noise_source::NoiseSource, port::PortReader, sensor_reading::Noise};

use crate::faults::pcg_noise_source::PcgNoiseSource;

pub struct GaussianNoise<R: PortReader<T>, T: Clone + Noise> {
    inner: R,
    source: RefCell<PcgNoiseSource>,
    stddev: f32,
    _phantom: core::marker::PhantomData<T>,
}

impl<R: PortReader<T>, T: Clone + Noise> GaussianNoise<R, T> {
    pub fn new(inner: R, stddev: f32, seed: u64) -> Self {
        Self {
            inner,
            source: RefCell::new(PcgNoiseSource::from_seed(seed)),
            stddev,
            _phantom: core::marker::PhantomData,
        }
    }
}

impl<R: PortReader<T>, T: Clone + Noise> PortReader<T> for GaussianNoise<R, T> {
    fn latest(&self) -> Option<T> {
        let mut r = self.inner.latest()?;
        let mut src = self.source.borrow_mut();
        let dyn_src: &mut dyn NoiseSource = &mut *src;
        r.apply_noise(dyn_src, self.stddev);
        Some(r)
    }

    fn take(&mut self) -> Option<T> {
        let mut r = self.inner.take()?;
        let mut src = self.source.borrow_mut();
        let dyn_src: &mut dyn NoiseSource = &mut *src;
        r.apply_noise(dyn_src, self.stddev);
        Some(r)
    }
}

#[cfg(test)]
mod tests {
    // Note: an integration-style test in arm/src/ports.rs covers the real
    // JointEncoderReading impl. Here we test wrapper mechanics with a stub
    // Noise type so we don't pull arm into sim's dep graph.
    use super::*;
    use rtf_core::{noise_source::NoiseSource, port::port};

    #[derive(Clone)]
    struct Stub {
        x: f32,
    }
    impl Noise for Stub {
        fn apply_noise(&mut self, source: &mut dyn NoiseSource, stddev: f32) {
            // Use the source so we exercise the dyn-dispatch path; throw
            // away the draw and add a fixed offset so the assertion is
            // deterministic regardless of which RNG backs the source.
            let _ = source.standard_normal();
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

//! Production `NoiseSource` impl: PCG64 driving `rand_distr::StandardNormal`.
//! This is the only place in the workspace that names `rand_pcg` /
//! `rand_distr` types — domain crates (`rtf_arm`, `rtf_core`) stay
//! third-party-free at the trait boundary.

use rand::{Rng, SeedableRng};
use rand_distr::{Distribution, StandardNormal};
use rand_pcg::Pcg64;

use rtf_core::noise_source::NoiseSource;

pub struct PcgNoiseSource(pub Pcg64);

impl PcgNoiseSource {
    pub fn from_seed(seed: u64) -> Self {
        Self(Pcg64::seed_from_u64(seed))
    }
}

impl NoiseSource for PcgNoiseSource {
    fn standard_normal(&mut self) -> f32 {
        StandardNormal.sample(&mut self.0)
    }

    fn uniform_unit(&mut self) -> f32 {
        // rand 0.8: Rng::gen::<f32>() draws uniformly from [0, 1).
        self.0.gen::<f32>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_yields_identical_sequence() {
        let mut a = PcgNoiseSource::from_seed(7);
        let mut b = PcgNoiseSource::from_seed(7);
        for _ in 0..16 {
            assert_eq!(a.standard_normal(), b.standard_normal());
        }
    }

    #[test]
    fn different_seeds_yield_different_sequences() {
        let mut a = PcgNoiseSource::from_seed(1);
        let mut b = PcgNoiseSource::from_seed(2);
        // Statistically near-certain at least one of 8 draws differs.
        let any_diff = (0..8).any(|_| a.standard_normal() != b.standard_normal());
        assert!(any_diff);
    }

    #[test]
    fn uniform_unit_in_range() {
        let mut s = PcgNoiseSource::from_seed(42);
        for _ in 0..100 {
            let v = s.uniform_unit();
            assert!((0.0..1.0).contains(&v), "draw out of [0, 1): {v}");
        }
    }

    #[test]
    fn uniform_unit_mean_near_half() {
        let mut s = PcgNoiseSource::from_seed(42);
        let n = 1000;
        let sum: f32 = (0..n).map(|_| s.uniform_unit()).sum();
        let mean = sum / n as f32;
        assert!(
            (0.45..=0.55).contains(&mean),
            "mean {mean} outside [0.45, 0.55]"
        );
    }

    #[test]
    fn uniform_unit_same_seed_reproduces() {
        let mut a = PcgNoiseSource::from_seed(42);
        let mut b = PcgNoiseSource::from_seed(42);
        for _ in 0..100 {
            assert_eq!(a.uniform_unit(), b.uniform_unit());
        }
    }
}

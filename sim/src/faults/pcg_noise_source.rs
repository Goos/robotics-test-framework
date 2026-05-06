//! Production `NoiseSource` impl: PCG64 driving `rand_distr::StandardNormal`.
//! This is the only place in the workspace that names `rand_pcg` /
//! `rand_distr` types — domain crates (`rtf_arm`, `rtf_core`) stay
//! third-party-free at the trait boundary.

use rand::SeedableRng;
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
}

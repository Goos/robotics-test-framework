//! Local abstraction over a seedable normal-distribution sampler so domain
//! types (e.g. `JointEncoderReading::apply_noise` in `rtf_arm`) never need to
//! name a third-party RNG type. Concrete impls live in `rtf_sim::faults`
//! (production: PCG64 + rand_distr) and in tests (deterministic stubs).

pub trait NoiseSource {
    /// Draw a single sample from the standard normal distribution
    /// (mean 0, stddev 1).
    fn standard_normal(&mut self) -> f32;
}

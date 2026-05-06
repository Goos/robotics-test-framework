use crate::noise_source::NoiseSource;
use crate::time::Time;

/// Marker trait for sensor reading types.
///
/// Per design v2 §9.4 (Behavioral Contract for Ports), all sensor reading types
/// in `rtf_arm::ports` (and any future robot-kind crate) MUST include a
/// `sampled_at` field accessible via this method. Controllers must use the
/// reading's `sampled_at` as the authoritative sampling time, NOT a derived
/// schedule.
pub trait SensorReading {
    fn sampled_at(&self) -> Time;
}

/// In-place perturbation of sensor readings (design v2 §13). Implemented by
/// reading types that can have Gaussian noise applied to their continuous
/// fields; the `GaussianNoise` fault wrapper calls this on each take/latest.
///
/// Takes a `&mut dyn NoiseSource` so impls never need to name a concrete RNG
/// type — `rand_pcg`/`rand_distr` stay confined to `rtf_sim::faults`.
pub trait Noise {
    fn apply_noise(&mut self, source: &mut dyn NoiseSource, stddev: f32);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::Time;

    struct Foo { sampled_at: Time }
    impl SensorReading for Foo {
        fn sampled_at(&self) -> Time { self.sampled_at }
    }

    #[test]
    fn sample_timestamp_accessible() {
        let r = Foo { sampled_at: Time::from_millis(42) };
        assert_eq!(r.sampled_at(), Time::from_millis(42));
    }
}

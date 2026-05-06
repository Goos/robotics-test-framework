use core::ops::{Add, Sub};

// Multiplication wraps on overflow (i64). Intentional: from_secs overflows at
// ~292 years, well past any robotics-sim timescale; checked_mul would clutter
// every call site. Add debug_assert! if a user-controlled `secs` ever arrives.
const NANOS_PER_MILLI: i64 = 1_000_000;
const NANOS_PER_SEC: i64 = 1_000_000_000;

/// Signed nanoseconds since scenario start. Range ±292 years.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Time(i64);

/// Signed nanosecond duration. `Time - Time` yields `Duration`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Duration(i64);

impl Time {
    pub const ZERO: Self = Self(0);
    pub const fn from_nanos(n: i64) -> Self { Self(n) }
    pub const fn from_millis(ms: i64) -> Self { Self(ms * NANOS_PER_MILLI) }
    pub const fn from_secs(s: i64) -> Self { Self(s * NANOS_PER_SEC) }
    pub const fn as_nanos(&self) -> i64 { self.0 }
}

impl Duration {
    pub const ZERO: Self = Self(0);
    pub const fn from_nanos(n: i64) -> Self { Self(n) }
    pub const fn from_millis(ms: i64) -> Self { Self(ms * NANOS_PER_MILLI) }
    pub const fn from_secs(s: i64) -> Self { Self(s * NANOS_PER_SEC) }
    pub const fn as_nanos(&self) -> i64 { self.0 }
    /// Lossy: precision degrades above 2^53 nanos (~104 days). Prefer `as_nanos()`
    /// for exact arithmetic; use this only for human-facing output or controller gains.
    pub fn as_secs_f64(&self) -> f64 { self.0 as f64 / NANOS_PER_SEC as f64 }
}

impl Sub<Time> for Time {
    type Output = Duration;
    fn sub(self, rhs: Time) -> Duration { Duration(self.0 - rhs.0) }
}

impl Add<Duration> for Time {
    type Output = Time;
    fn add(self, rhs: Duration) -> Time { Time(self.0 + rhs.0) }
}

impl Add<Duration> for Duration {
    type Output = Duration;
    fn add(self, other: Duration) -> Duration { Duration(self.0 + other.0) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duration_from_secs_and_back() {
        let d = Duration::from_secs(2);
        assert_eq!(d.as_nanos(), 2_000_000_000);
        assert_eq!(d.as_secs_f64(), 2.0);
    }

    #[test]
    fn time_subtraction_yields_signed_duration() {
        let t0 = Time::from_nanos(1_000);
        let t1 = Time::from_nanos(500);
        assert_eq!((t0 - t1).as_nanos(),  500);
        assert_eq!((t1 - t0).as_nanos(), -500);
    }

    #[test]
    fn time_plus_duration() {
        let t = Time::from_nanos(100) + Duration::from_nanos(50);
        assert_eq!(t.as_nanos(), 150);
    }

    #[test]
    fn zero_constants() {
        assert_eq!(Time::ZERO.as_nanos(), 0);
        assert_eq!(Duration::ZERO.as_nanos(), 0);
    }

    #[test]
    fn duration_plus_duration() {
        assert_eq!((Duration::from_nanos(1) + Duration::from_nanos(2)).as_nanos(), 3);
    }
}

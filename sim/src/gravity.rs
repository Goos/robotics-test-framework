//! Gravity-fall — kinematic, vertical-only, simple support resolution.
//!
//! Per design v2 §5.5: free objects fall under constant gravity, snap to the
//! highest support beneath their xy footprint, and become Settled. No physics
//! engine; one linear pass per tick.

/// Standard gravitational acceleration at Earth's surface (m/s²). Acts in
/// the −z direction; only `lin_vel.z` is integrated.
pub const GRAVITY_M_PER_S2: f32 = 9.81;

/// Below this fall-distance threshold (m), an object's z-velocity is zeroed
/// and it is reclassified as Settled. Picks up the small numerical residue
/// from the fixed-step integrator so objects don't drift forever.
pub const SETTLE_EPSILON_M: f32 = 1e-5;

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn gravity_constant_is_9_81() {
        assert!((GRAVITY_M_PER_S2 - 9.81).abs() < 1e-6);
    }

    #[test]
    fn settle_epsilon_is_small() {
        // Compile-time check — clippy::assertions_on_constants flags the
        // runtime-form `assert!(CONST < literal)` since both sides are const.
        const _: () = assert!(SETTLE_EPSILON_M < 1e-3);
    }
}

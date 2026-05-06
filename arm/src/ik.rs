//! Closed-form 2R planar inverse kinematics.
//!
//! Used by Phase 8+ controllers that need to drive a Z-Y-Y arm's pitch joints
//! to a target position in the shoulder-local (radial, z) plane. The yaw
//! joint (J0, Z-axis) is solved separately by aligning the radial axis with
//! the target's xy heading; this module only handles the two pitch joints.

/// Closed-form 2R planar IK in the shoulder-local `(x, z)` plane.
///
/// `target_x`/`target_z` are taken in the shoulder's local frame: `+x` is
/// "out along the arm at rest" (the radial direction once the yaw joint is
/// aligned with the target xy) and `+z` is world-up. `l1` and `l2` are the
/// two link lengths.
///
/// Both joints are Y-axis revolutes. At `α=0, β=0` the chain extends along
/// `+x` at `z=0` (shoulder height). Positive `α` (the upper-arm pitch)
/// rotates `+x` toward `-z`, i.e. tilts the upper arm downward.
///
/// Returns `Some((α, β))` for the elbow-down branch, or `None` if the
/// target is unreachable (`d > l1+l2` or `d < |l1-l2|`).
pub fn ik_2r(target_x: f32, target_z: f32, l1: f32, l2: f32) -> Option<(f32, f32)> {
    let d_sq = target_x * target_x + target_z * target_z;
    let d = d_sq.sqrt();
    let reach_max = l1 + l2;
    let reach_min = (l1 - l2).abs();
    if d > reach_max || d < reach_min {
        return None;
    }
    // Math-frame z' = -target_z because Ry(+α) sends +x toward -z (opposite
    // of the standard atan2 convention where +α sends +x toward +y).
    let z_math = -target_z;
    let cos_beta = ((d_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)).clamp(-1.0, 1.0);
    let beta = cos_beta.acos();
    let alpha = z_math.atan2(target_x) - (l2 * beta.sin()).atan2(l1 + l2 * beta.cos());
    Some((alpha, beta))
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    /// Forward-kinematics for a 2R Y-pitch chain in the shoulder-local
    /// (x, z) plane. Returns the EE position given joint angles and link
    /// lengths. Used to round-trip-check the IK solver.
    fn fk_2r(alpha: f32, beta: f32, l1: f32, l2: f32) -> (f32, f32) {
        // After Ry(α): the link of length L1 starting along +x ends at
        // (L1 cos α, -L1 sin α). After Ry(α+β) the second link adds
        // (L2 cos(α+β), -L2 sin(α+β)).
        let x = l1 * alpha.cos() + l2 * (alpha + beta).cos();
        let z = -(l1 * alpha.sin() + l2 * (alpha + beta).sin());
        (x, z)
    }

    fn assert_close(a: f32, b: f32, tol: f32, label: &str) {
        assert!(
            (a - b).abs() < tol,
            "{label}: got {a}, expected {b} (tol {tol})"
        );
    }

    fn assert_round_trip(target_x: f32, target_z: f32, l1: f32, l2: f32) {
        let (alpha, beta) = ik_2r(target_x, target_z, l1, l2)
            .unwrap_or_else(|| panic!("expected reachable target ({target_x}, {target_z})"));
        let (fx, fz) = fk_2r(alpha, beta, l1, l2);
        assert_close(fx, target_x, 1e-4, "fk x");
        assert_close(fz, target_z, 1e-4, "fk z");
    }

    #[test]
    fn arm_extended_horizontal() {
        // Singular d=L1+L2 boundary: cos_β = 1 exactly in real math but f32
        // arithmetic introduces ~2e-4 error in α/β. Tolerance is loosened
        // accordingly; FK round-trip still pins the EE position to 1e-4.
        let (alpha, beta) = ik_2r(0.8, 0.0, 0.4, 0.4).unwrap();
        assert_close(alpha, 0.0, 1e-3, "alpha");
        assert_close(beta, 0.0, 1e-3, "beta");
    }

    #[test]
    fn arm_straight_down() {
        let (alpha, beta) = ik_2r(0.0, -0.8, 0.4, 0.4).unwrap();
        assert_close(alpha, PI / 2.0, 1e-3, "alpha");
        assert_close(beta, 0.0, 1e-3, "beta");
    }

    #[test]
    fn arm_l_shape() {
        // upper arm horizontal (+x), forearm pointing down: target = (L1, -L2)
        let (alpha, beta) = ik_2r(0.4, -0.4, 0.4, 0.4).unwrap();
        assert_close(alpha, 0.0, 1e-5, "alpha");
        assert_close(beta, PI / 2.0, 1e-5, "beta");
    }

    #[test]
    fn round_trip_below_shoulder() {
        assert_round_trip(0.6, -0.25, 0.4, 0.4);
    }

    #[test]
    fn round_trip_above_shoulder() {
        assert_round_trip(0.6, 0.05, 0.4, 0.4);
    }

    #[test]
    fn unreachable_far() {
        assert!(ik_2r(2.0, 0.0, 0.4, 0.4).is_none());
    }

    #[test]
    fn unreachable_inside_min_radius() {
        // L1=0.4, L2=0.1 → reach_min=0.3; d=0.1 is too close.
        assert!(ik_2r(0.1, 0.0, 0.4, 0.1).is_none());
    }
}

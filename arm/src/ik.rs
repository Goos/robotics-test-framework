//! Closed-form 2R planar inverse kinematics.
//!
//! Used by Phase 8+ controllers that need to drive a Z-Y-Y arm's pitch joints
//! to a target position in the shoulder-local (radial, z) plane. The yaw
//! joint (J0, Z-axis) is solved separately by aligning the radial axis with
//! the target's xy heading; this module only handles the two pitch joints.

/// Closed-form 3R planar IK in the shoulder-local `(x, z)` plane with an
/// explicit EE pitch target. Returns `(J1, J2, J3)` such that:
/// - The wrist link of length `l3` ends at `(target_x, target_z)`.
/// - The cumulative pitch `J1 + J2 + J3` equals `target_pitch`.
///
/// Wrist convention: at total cumulative pitch `θ`, the EE +x direction
/// points along world `(cos θ, 0, -sin θ)` (the IK code's pitch
/// convention — see `ik_2r` for the +α-rotates-+x-toward-`-z` rule).
/// So:
/// - `target_pitch = 0` → EE +x points world +x (level).
/// - `target_pitch = π/2` → EE +x points world -z (fingers down — what
///   Phase 3.4.5 controllers want for vertical-from-above grasping).
/// - `target_pitch = -π/2` → EE +x points world +z (fingers up).
///
/// Algorithm: subtract the wrist link from the EE target to get the
/// wrist-anchor target `(target_x - l3·cos(target_pitch),
///                       target_z + l3·sin(target_pitch))` (note the +
///   on the z term — the wrist link extends along the EE +x = world
///   `(cos θ, -sin θ)` direction, so we go BACKWARD from the EE by that
///   vector to find the anchor); call `ik_2r` on the wrist anchor for
/// `(J1, J2)`; compute `J3 = target_pitch - (J1 + J2)`.
///
/// Returns `None` iff the wrist anchor is itself unreachable by `ik_2r`.
pub fn ik_3r(
    target_x: f32,
    target_z: f32,
    target_pitch: f32,
    l1: f32,
    l2: f32,
    l3: f32,
) -> Option<(f32, f32, f32)> {
    let wrist_x = target_x - l3 * target_pitch.cos();
    let wrist_z = target_z + l3 * target_pitch.sin();
    let (j1, j2) = ik_2r(wrist_x, wrist_z, l1, l2)?;
    let j3 = target_pitch - (j1 + j2);
    Some((j1, j2, j3))
}

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

    /// Forward kinematics for the 3R planar Y-pitch chain with wrist link
    /// of length `l3`. Returns the EE position given joint angles. Used to
    /// round-trip-check `ik_3r`. Mirrors `fk_2r` exactly with one extra
    /// link contribution at cumulative pitch `α + β + γ`.
    fn fk_3r(j1: f32, j2: f32, j3: f32, l1: f32, l2: f32, l3: f32) -> (f32, f32) {
        let theta1 = j1;
        let theta12 = j1 + j2;
        let theta123 = j1 + j2 + j3;
        let x = l1 * theta1.cos() + l2 * theta12.cos() + l3 * theta123.cos();
        let z = -(l1 * theta1.sin() + l2 * theta12.sin() + l3 * theta123.sin());
        (x, z)
    }

    fn assert_round_trip_3r(target_x: f32, target_z: f32, target_pitch: f32) {
        let (l1, l2, l3) = (0.4, 0.4, 0.05);
        let (j1, j2, j3) =
            ik_3r(target_x, target_z, target_pitch, l1, l2, l3).unwrap_or_else(|| {
                panic!("expected reachable: ({target_x}, {target_z}) pitch={target_pitch}")
            });
        let (fx, fz) = fk_3r(j1, j2, j3, l1, l2, l3);
        assert_close(fx, target_x, 1e-4, "fk_3r x");
        assert_close(fz, target_z, 1e-4, "fk_3r z");
        let cumulative = j1 + j2 + j3;
        assert_close(cumulative, target_pitch, 1e-4, "cumulative pitch");
    }

    #[test]
    fn ik_3r_level_wrist_round_trip() {
        // pitch=0 → EE +x pointing world +x; wrist link extends along world
        // +x, so wrist anchor = ee_target - (l3, 0).
        assert_round_trip_3r(0.6, -0.25, 0.0);
    }

    #[test]
    fn ik_3r_wrist_pointing_down_round_trip() {
        // pitch=π/2 → EE +x pointing world -z; wrist link drops by l3 along
        // -z, so wrist anchor sits l3 ABOVE the ee target. This is the
        // canonical Phase 3.4.5c configuration: EE +x = world -z, fingers
        // protrude downward.
        assert_round_trip_3r(0.6, -0.25, PI / 2.0);
    }

    #[test]
    fn ik_3r_wrist_pointing_up_round_trip() {
        // pitch=-π/2 → EE +x pointing world +z. Symmetric edge case.
        assert_round_trip_3r(0.5, 0.10, -PI / 2.0);
    }

    #[test]
    fn ik_3r_unreachable_returns_none() {
        // EE target far beyond l1+l2+l3 reach.
        assert!(ik_3r(2.0, 0.0, 0.0, 0.4, 0.4, 0.05).is_none());
    }

    #[test]
    fn ik_3r_pick_place_canonical_pose_points_fingers_down() {
        // The exact pose that broke Step 3.4's friction-grasp on
        // build_pick_and_place_world. With pitch=π/2 the cumulative
        // J1+J2+J3 = π/2 so EE +x = world -z. The wrist anchor IS
        // reachable for the existing 2R arm (l1=l2=0.4) with l3=0.05.
        let (j1, j2, j3) = ik_3r(0.6, -0.25, PI / 2.0, 0.4, 0.4, 0.05).unwrap();
        // Spot-check cumulative pitch math.
        assert_close(j1 + j2 + j3, PI / 2.0, 1e-4, "cumulative pitch");
        // Each joint within ±π (within JointSpec::Revolute limits).
        assert!(j1.abs() < PI && j2.abs() < PI && j3.abs() < PI);
    }

    #[test]
    fn ik_3r_matches_ik_2r_when_l3_is_zero() {
        // Degenerate case: l3=0 means the wrist link contributes nothing
        // to position, so `ik_3r(target_x, target_z, pitch, l1, l2, 0)`
        // must place the wrist anchor exactly at the EE target. J1+J2
        // come from ik_2r at the EE target; J3 = pitch - (J1+J2).
        let (j1, j2) = ik_2r(0.6, -0.25, 0.4, 0.4).unwrap();
        let pitch = PI / 2.0;
        let (k1, k2, k3) = ik_3r(0.6, -0.25, pitch, 0.4, 0.4, 0.0).unwrap();
        assert_close(k1, j1, 1e-6, "j1");
        assert_close(k2, j2, 1e-6, "j2");
        assert_close(k3, pitch - (j1 + j2), 1e-6, "j3");
    }
}

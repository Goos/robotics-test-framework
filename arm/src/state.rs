use rtf_sim::object::ObjectId;

/// Per-tick mutable arm state (design v2 §5.3): joint positions/velocities,
/// gripper command latch, and the grasped object id (if any).
///
/// Phase 3 introduced `gripper_separation` (continuous lateral finger
/// distance, m) alongside the legacy boolean `gripper_closed`. The two are
/// kept in lockstep through the Phase 3.1/3.2 transition: Step 3.1 derives
/// `gripper_separation` from `gripper_closed` so the new finger physics
/// has a value to consume, and Step 3.2 swaps the API so target_separation
/// drives the actual finger motion (gripper_closed becomes a derived flag).
#[derive(Clone, Debug)]
pub struct ArmState {
    pub q: Vec<f32>,
    pub q_dot: Vec<f32>,
    pub gripper_closed: bool,
    /// Lateral distance (m) between the two finger centers along EE +y.
    /// Initially `FINGER_OPEN_SEPARATION` (0.04 m). Driven by
    /// `apply_gripper_command` in Phase 3.2 and consumed by the per-tick
    /// finger-pose update in `consume_actuators_and_integrate_inner`.
    pub gripper_separation: f32,
    pub grasped: Option<ObjectId>,
}

impl ArmState {
    pub fn zeros(n_joints: usize) -> Self {
        Self {
            q: vec![0.0; n_joints],
            q_dot: vec![0.0; n_joints],
            gripper_closed: false,
            // Match `arm::FINGER_OPEN_SEPARATION` — kept as a literal here
            // to avoid the cross-module dep; the constant is private to
            // arm.rs but the value (0.04 m) is the canonical "open" value.
            gripper_separation: 0.04,
            grasped: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn arm_state_zeros_initializes_correctly() {
        let s = ArmState::zeros(3);
        assert_eq!(s.q.len(), 3);
        assert_eq!(s.q_dot.len(), 3);
        assert!(s.q.iter().all(|&v| v == 0.0));
        assert!(s.q_dot.iter().all(|&v| v == 0.0));
        assert!(!s.gripper_closed);
        assert!((s.gripper_separation - 0.04).abs() < 1e-6);
        assert!(s.grasped.is_none());
    }
}

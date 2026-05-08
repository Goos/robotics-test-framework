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
    /// Initially `FINGER_OPEN_SEPARATION` (0.04 m). Per-tick the world
    /// drives this toward `gripper_target` at a fixed rate (Phase 3.2),
    /// then writes the finger kinematic poses from the new value.
    pub gripper_separation: f32,
    /// Most recent `GripperCommand.target_separation` the controller
    /// asked for. The per-tick consume-actuators path drives
    /// `gripper_separation` toward this value at 0.5 m/s. Initialized
    /// to the open value so an arm with no controller stays open.
    pub gripper_target: f32,
    pub grasped: Option<ObjectId>,
}

impl ArmState {
    pub fn zeros(n_joints: usize) -> Self {
        Self {
            q: vec![0.0; n_joints],
            q_dot: vec![0.0; n_joints],
            gripper_closed: false,
            // Match `arm::FINGER_OPEN_SEPARATION` — kept as a literal here
            // to avoid the cross-module dep; the constant is the
            // canonical "open" value (0.04 m).
            gripper_separation: 0.04,
            gripper_target: 0.04,
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

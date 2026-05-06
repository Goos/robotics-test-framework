use rtf_sim::object::ObjectId;

/// Per-tick mutable arm state (design v2 §5.3): joint positions/velocities,
/// gripper command latch, and the grasped object id (if any).
#[derive(Clone, Debug)]
pub struct ArmState {
    pub q: Vec<f32>,
    pub q_dot: Vec<f32>,
    pub gripper_closed: bool,
    pub grasped: Option<ObjectId>,
}

impl ArmState {
    pub fn zeros(n_joints: usize) -> Self {
        Self {
            q: vec![0.0; n_joints],
            q_dot: vec![0.0; n_joints],
            gripper_closed: false,
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
        assert!(s.grasped.is_none());
    }
}

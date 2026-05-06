//! `PickObject` goal — complete when the arm is grasping the target object.
//! Shaped score uses EE-to-object distance for gradient when not yet grasped.

use rtf_core::{goal::Goal, score::Score};
use rtf_sim::object::ObjectId;

use crate::world::ArmWorld;

pub struct PickObject {
    pub target: ObjectId,
}

impl PickObject {
    pub fn new(target: ObjectId) -> Self {
        Self { target }
    }
}

impl Goal<ArmWorld> for PickObject {
    fn is_complete(&self, world: &ArmWorld) -> bool {
        world.arm.state.grasped == Some(self.target)
    }

    fn evaluate(&self, world: &ArmWorld) -> Score {
        if world.arm.state.grasped == Some(self.target) {
            return Score::new(1.0);
        }
        let Some(obj) = world.scene.object(self.target) else {
            return Score::new(0.0);
        };
        // Cap the shaped score at 0.99 so the grasped→1.0 transition stays a
        // strict, monotonic boundary (the controller can never "earn" 1.0
        // without actually grasping).
        let dist = (world.ee_pose().translation.vector - obj.pose.translation.vector).norm();
        let max_dist = 2.0_f32;
        let normalized = (1.0 - (dist / max_dist) as f64).clamp(0.0, 0.99);
        Score::new(normalized)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;

    #[test]
    fn complete_when_target_object_is_grasped() {
        let mut world = build_pick_and_place_world();
        let block = block_id(&world);
        world.arm.state.grasped = Some(block);
        let goal = PickObject::new(block);
        assert!(goal.is_complete(&world));
        assert!(goal.evaluate(&world).value > 0.99);
    }

    #[test]
    fn evaluate_shaped_by_distance_when_not_grasped() {
        let world = build_pick_and_place_world();
        let block = block_id(&world);
        let goal = PickObject::new(block);
        assert!(!goal.is_complete(&world));
        let s = goal.evaluate(&world).value;
        assert!((0.0..1.0).contains(&s), "score should be in [0, 1) when not grasped, got {}", s);
    }
}

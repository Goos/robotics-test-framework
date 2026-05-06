use nalgebra::Isometry3;
use rtf_core::{goal::Goal, score::Score};

use crate::world::ArmWorld;

/// Reach a target end-effector pose within a translation tolerance
/// (design v2 §6). Score is `1 - dist/tolerance`, clamped to [0, 1]:
/// 1.0 at the target, linearly degrading to 0.0 at the tolerance boundary.
/// Orientation tracking is intentionally deferred until a v2 goal that
/// composes translation + rotation error.
pub struct ReachPose {
    pub target: Isometry3<f32>,
    pub tolerance: f32,
}

impl ReachPose {
    pub fn new(target: Isometry3<f32>, tolerance: f32) -> Self {
        Self { target, tolerance }
    }

    fn distance(&self, world: &ArmWorld) -> f32 {
        (world.ee_pose().translation.vector - self.target.translation.vector).norm()
    }
}

impl Goal<ArmWorld> for ReachPose {
    fn is_complete(&self, world: &ArmWorld) -> bool {
        self.distance(world) <= self.tolerance
    }

    fn evaluate(&self, world: &ArmWorld) -> Score {
        let dist = self.distance(world) as f64;
        let tol = self.tolerance as f64;
        Score::new((1.0 - dist / tol).clamp(0.0, 1.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spec::{ArmSpec, GripperSpec, JointSpec};
    use crate::world::ArmWorld;
    use rtf_sim::scene::Scene;

    fn simple_spec() -> ArmSpec {
        use core::f32::consts::PI;
        use nalgebra::{Isometry3, Vector3};
        ArmSpec {
            joints: vec![
                JointSpec::Revolute {
                    axis: Vector3::z_axis(),
                    limits: (-PI, PI)
                };
                2
            ],
            link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.1); 2],
            gripper: GripperSpec {
                proximity_threshold: 0.02,
                max_grasp_size: 0.05,
            },
        }
    }

    #[test]
    fn score_is_one_when_at_target_within_tolerance() {
        let world = ArmWorld::new(Scene::new(0), simple_spec(), /* gravity */ false);
        let target = world.ee_pose();
        let goal = ReachPose::new(target, /* tolerance */ 0.01);
        assert!(goal.is_complete(&world));
        assert!(goal.evaluate(&world).value > 0.99);
    }
}

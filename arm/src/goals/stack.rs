//! `Stack` goal — complete when object `a` is `Settled` directly above
//! object `b` and has held that arrangement for at least
//! `STACK_HOLD_DURATION_MS`. The hold requirement filters out transient
//! settles caused by physics-step jitter.
//!
//! Pre-Rapier (v1) checked `a.state == Settled { on: SupportId::Object(b) }`
//! since the kinematic gravity_step tracked support chains directly.
//! Rapier doesn't track support, so post-rapier (Phase 1) the check is
//! pose-based: a is at-rest, a's center xy is within 1 cm of b's center xy,
//! and a's z is just above b's top z.

use rtf_core::{goal::Goal, score::Score, time::Time};
use rtf_sim::object::{ObjectId, ObjectState};

use crate::world::ArmWorld;

const STACK_HOLD_DURATION_MS: i64 = 100;
/// Max horizontal offset between a and b for a to count as stacked on b.
const STACK_XY_TOL: f32 = 0.01;
/// Max vertical gap between b's top and a's center for a to count as
/// stacked on b. Generous to absorb solver jitter.
const STACK_Z_TOL: f32 = 0.05;

pub struct Stack {
    pub a: ObjectId,
    pub b: ObjectId,
    settled_since: Option<Time>,
    last_tick: Option<Time>,
}

impl Stack {
    pub fn new(a: ObjectId, b: ObjectId) -> Self {
        Self {
            a,
            b,
            settled_since: None,
            last_tick: None,
        }
    }

    fn currently_stacked(&self, world: &ArmWorld) -> bool {
        let Some(obj_a) = world.scene.object(self.a) else {
            return false;
        };
        let Some(obj_b) = world.scene.object(self.b) else {
            return false;
        };
        if !matches!(obj_a.state, ObjectState::Settled { .. }) {
            return false;
        }
        let dx = (obj_a.pose.translation.x - obj_b.pose.translation.x).abs();
        let dy = (obj_a.pose.translation.y - obj_b.pose.translation.y).abs();
        let b_top_z = obj_b.pose.translation.z + obj_b.shape.half_height_z();
        let dz = (obj_a.pose.translation.z - b_top_z).abs();
        dx <= STACK_XY_TOL && dy <= STACK_XY_TOL && dz <= STACK_Z_TOL
    }
}

impl Goal<ArmWorld> for Stack {
    fn tick(&mut self, t: Time, world: &ArmWorld) {
        self.last_tick = Some(t);
        if self.currently_stacked(world) {
            if self.settled_since.is_none() {
                self.settled_since = Some(t);
            }
        } else {
            self.settled_since = None;
        }
    }

    fn is_complete(&self, _world: &ArmWorld) -> bool {
        let (Some(start), Some(now)) = (self.settled_since, self.last_tick) else {
            return false;
        };
        let elapsed = now - start;
        elapsed.as_nanos() >= STACK_HOLD_DURATION_MS * 1_000_000
    }

    fn evaluate(&self, world: &ArmWorld) -> Score {
        if self.is_complete(world) {
            return Score::new(1.0);
        }
        if self.currently_stacked(world) {
            return Score::new(0.95);
        }
        Score::new(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;
    use nalgebra::Translation3;
    use rtf_core::time::Time;
    use rtf_sim::object::SupportId;

    #[test]
    fn complete_when_a_settled_on_b_for_at_least_100ms() {
        let mut world = build_pick_and_place_world();
        let a = world.scene.add_object_default();
        let b = world.scene.add_object_default();

        // Position b at a known pose; place a directly on top (centers
        // aligned in xy, a's z at b's top + a's half-height ish).
        let b_obj = world.scene.object_mut(b).unwrap();
        b_obj.state = ObjectState::Settled {
            on: SupportId::Unknown,
        };
        b_obj.pose.translation = Translation3::new(0.5, 0.0, 0.5);
        let b_top = 0.5 + b_obj.shape.half_height_z();

        let a_obj = world.scene.object_mut(a).unwrap();
        a_obj.state = ObjectState::Settled {
            on: SupportId::Unknown,
        };
        a_obj.pose.translation = Translation3::new(0.5, 0.0, b_top);

        let mut goal = Stack::new(a, b);
        for ms in 0..100 {
            goal.tick(Time::from_millis(ms), &world);
            assert!(
                !goal.is_complete(&world),
                "should not be complete yet at {} ms",
                ms
            );
        }
        goal.tick(Time::from_millis(100), &world);
        assert!(goal.is_complete(&world));
        assert!(goal.evaluate(&world).value > 0.99);
    }
}

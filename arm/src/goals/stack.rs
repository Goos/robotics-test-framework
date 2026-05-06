//! `Stack` goal — complete when object `a` is `Settled` on object `b` and has
//! held that arrangement for at least `STACK_HOLD_DURATION_MS`. The hold
//! requirement filters out transient settles caused by gravity-step jitter.

use rtf_core::{goal::Goal, score::Score, time::Time};
use rtf_sim::object::{ObjectId, ObjectState, SupportId};

use crate::world::ArmWorld;

const STACK_HOLD_DURATION_MS: i64 = 100;

pub struct Stack {
    pub a: ObjectId,
    pub b: ObjectId,
    settled_since: Option<Time>,
    last_tick: Option<Time>,
}

impl Stack {
    pub fn new(a: ObjectId, b: ObjectId) -> Self {
        Self { a, b, settled_since: None, last_tick: None }
    }

    fn currently_stacked(&self, world: &ArmWorld) -> bool {
        let Some(obj_a) = world.scene.object(self.a) else { return false; };
        matches!(
            obj_a.state,
            ObjectState::Settled { on: SupportId::Object(id) } if id == self.b.0,
        )
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
    use rtf_core::time::Time;

    #[test]
    fn complete_when_a_settled_on_b_for_at_least_100ms() {
        let mut world = build_pick_and_place_world();
        let a = world.scene.add_object_default();
        let b = world.scene.add_object_default();
        world.scene.object_mut(a).unwrap().state =
            ObjectState::Settled { on: SupportId::Object(b.0) };
        let mut goal = Stack::new(a, b);
        for ms in 0..100 {
            goal.tick(Time::from_millis(ms), &world);
            assert!(!goal.is_complete(&world), "should not be complete yet at {} ms", ms);
        }
        goal.tick(Time::from_millis(100), &world);
        assert!(goal.is_complete(&world));
        assert!(goal.evaluate(&world).value > 0.99);
    }
}

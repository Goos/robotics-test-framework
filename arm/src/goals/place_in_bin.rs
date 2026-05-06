//! `PlaceInBin` goal — complete when the target object is `Settled` on the
//! bin fixture. Shaped score uses the block's xy distance to the bin center
//! for gradient when not yet placed.

use rtf_core::{goal::Goal, score::Score};
use rtf_sim::object::{ObjectId, ObjectState, SupportId};

use crate::world::ArmWorld;

pub struct PlaceInBin {
    pub target: ObjectId,
    pub bin: u32,
}

impl PlaceInBin {
    pub fn new(target: ObjectId, bin: u32) -> Self {
        Self { target, bin }
    }

    fn block_xy(&self, world: &ArmWorld) -> Option<(f32, f32)> {
        let obj = world.scene.object(self.target)?;
        Some((obj.pose.translation.x, obj.pose.translation.y))
    }

    fn bin_xy(&self, world: &ArmWorld) -> Option<(f32, f32)> {
        let (_, fix) = world.scene.fixtures().find(|(id, _)| **id == self.bin)?;
        Some((fix.pose.translation.x, fix.pose.translation.y))
    }
}

impl Goal<ArmWorld> for PlaceInBin {
    fn is_complete(&self, world: &ArmWorld) -> bool {
        let Some(obj) = world.scene.object(self.target) else {
            return false;
        };
        matches!(
            obj.state,
            ObjectState::Settled { on: SupportId::Fixture(id) } if id == self.bin,
        )
    }

    fn evaluate(&self, world: &ArmWorld) -> Score {
        if self.is_complete(world) {
            return Score::new(1.0);
        }
        let (Some(b), Some(bn)) = (self.block_xy(world), self.bin_xy(world)) else {
            return Score::new(0.0);
        };
        let dx = b.0 - bn.0;
        let dy = b.1 - bn.1;
        let dist = (dx * dx + dy * dy).sqrt();
        let max_dist = 2.0_f32;
        let normalized = (1.0 - (dist / max_dist) as f64).clamp(0.0, 0.99);
        Score::new(normalized)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;
    use nalgebra::Translation3;

    #[test]
    fn complete_when_object_is_settled_inside_bin() {
        let mut world = build_pick_and_place_world();
        let block = block_id(&world);
        let bin = bin_id(&world);
        let obj = world.scene.object_mut(block).unwrap();
        obj.state = ObjectState::Settled {
            on: SupportId::Fixture(bin),
        };
        obj.pose.translation = Translation3::new(0.0, 0.5, 0.625);
        let goal = PlaceInBin::new(block, bin);
        assert!(goal.is_complete(&world));
        assert!(goal.evaluate(&world).value > 0.99);
    }

    #[test]
    fn not_complete_when_block_is_on_table() {
        let world = build_pick_and_place_world();
        let block = block_id(&world);
        let bin = bin_id(&world);
        let goal = PlaceInBin::new(block, bin);
        assert!(!goal.is_complete(&world));
        let s = goal.evaluate(&world).value;
        assert!((0.0..1.0).contains(&s));
    }
}

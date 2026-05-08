//! `PlaceInBin` goal — complete when the target object is at rest with its
//! center inside the bin's xy footprint, at or above the bin top. Shaped
//! score uses the block's xy distance to the bin center for gradient when
//! not yet placed.
//!
//! Pre-Rapier (v1) this matched on `Settled { on: SupportId::Fixture(bin_id) }`
//! since the kinematic gravity_step tracked support chains directly.
//! Rapier doesn't track support, so post-rapier (Phase 1) the check is
//! pose-based: Settled (any support, incl. SupportId::Unknown) AND inside
//! the bin's xy footprint AND z above the bin top.

use rtf_core::{goal::Goal, score::Score};
use rtf_sim::object::{ObjectId, ObjectState};

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

    /// Bin xy footprint half-extents + bin top z. Returns None if no
    /// such fixture or its shape is non-Aabb.
    fn bin_footprint(&self, world: &ArmWorld) -> Option<((f32, f32), nalgebra::Vector3<f32>, f32)> {
        let (_, fix) = world.scene.fixtures().find(|(id, _)| **id == self.bin)?;
        let half = match fix.shape {
            rtf_sim::shape::Shape::Aabb { half_extents } => half_extents,
            _ => return None,
        };
        let center_xy = (fix.pose.translation.x, fix.pose.translation.y);
        let top_z = fix.pose.translation.z + half.z;
        Some((center_xy, half, top_z))
    }
}

impl Goal<ArmWorld> for PlaceInBin {
    fn is_complete(&self, world: &ArmWorld) -> bool {
        let Some(obj) = world.scene.object(self.target) else {
            return false;
        };
        if !matches!(obj.state, ObjectState::Settled { .. }) {
            return false;
        }
        let Some((center, half, top_z)) = self.bin_footprint(world) else {
            return false;
        };
        let dx = (obj.pose.translation.x - center.0).abs();
        let dy = (obj.pose.translation.y - center.1).abs();
        // Inside the bin's xy footprint, settled at or above the bin top
        // (z slop = 0.05 m so a stack-of-blocks remains "in the bin" too).
        dx <= half.x && dy <= half.y && obj.pose.translation.z >= top_z - 0.05
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
    use rtf_sim::object::SupportId;

    #[test]
    fn complete_when_object_is_settled_inside_bin_footprint() {
        let mut world = build_pick_and_place_world();
        let block = block_id(&world);
        let bin = bin_id(&world);
        let obj = world.scene.object_mut(block).unwrap();
        obj.state = ObjectState::Settled {
            on: SupportId::Unknown, // Rapier-derived; goal no longer cares about `on`.
        };
        // Bin center xy=(0, 0.6); place block well within the footprint
        // (half_extents.x/y = 0.1) at z=0.625 (just above bin top z=0.6).
        obj.pose.translation = Translation3::new(0.0, 0.6, 0.625);
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

//! Gravity-fall — kinematic, vertical-only, simple support resolution.
//!
//! Per design v2 §5.5: free objects fall under constant gravity, snap to the
//! highest support beneath their xy footprint, and become Settled. No physics
//! engine; one linear pass per tick.

use nalgebra::Isometry3;

use crate::{
    object::{ObjectId, ObjectState, SupportId},
    scene::Scene,
    shape::Shape,
};

/// Standard gravitational acceleration at Earth's surface (m/s²). Acts in
/// the −z direction; only `lin_vel.z` is integrated.
pub const GRAVITY_M_PER_S2: f32 = 9.81;

/// Below this fall-distance threshold (m), an object's z-velocity is zeroed
/// and it is reclassified as Settled. Picks up the small numerical residue
/// from the fixed-step integrator so objects don't drift forever.
pub const SETTLE_EPSILON_M: f32 = 1e-5;

/// Find the highest support surface beneath the xy column at `z_above` (the
/// candidate object's bottom). Considers `is_support` fixtures and Settled
/// objects (skipping `ignore`, the falling object itself). Returns
/// `(SupportId, top_z)` of the chosen support, or `None` if nothing
/// qualifies (object falls past existing geometry).
pub fn find_support_beneath(
    scene: &Scene,
    xy: (f32, f32),
    z_above: f32,
    ignore: Option<ObjectId>,
) -> Option<(SupportId, f32)> {
    let mut best: Option<(SupportId, f32)> = None;
    for (id, fix) in scene.fixtures() {
        if !fix.is_support { continue; }
        if !shape_xy_contains(fix.pose, &fix.shape, xy) { continue; }
        let top_z = fix.pose.translation.z + fix.shape.half_height_z();
        if top_z <= z_above && best.is_none_or(|(_, z)| top_z > z) {
            best = Some((SupportId::Fixture(*id), top_z));
        }
    }
    for (id, obj) in scene.objects() {
        if Some(*id) == ignore { continue; }
        if !matches!(obj.state, ObjectState::Settled { .. }) { continue; }
        if !shape_xy_contains(obj.pose, &obj.shape, xy) { continue; }
        let top_z = obj.pose.translation.z + obj.shape.half_height_z();
        if top_z <= z_above && best.is_none_or(|(_, z)| top_z > z) {
            best = Some((SupportId::Object(id.0), top_z));
        }
    }
    best
}

fn shape_xy_contains(pose: Isometry3<f32>, shape: &Shape, xy: (f32, f32)) -> bool {
    let dx = xy.0 - pose.translation.x;
    let dy = xy.1 - pose.translation.y;
    match shape {
        Shape::Sphere { radius } => dx * dx + dy * dy <= radius * radius,
        Shape::Aabb { half_extents } => dx.abs() <= half_extents.x && dy.abs() <= half_extents.y,
        Shape::Cylinder { radius, .. } => dx * dx + dy * dy <= radius * radius,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn gravity_constant_is_9_81() {
        assert!((GRAVITY_M_PER_S2 - 9.81).abs() < 1e-6);
    }

    #[test]
    fn settle_epsilon_is_small() {
        // Compile-time check — clippy::assertions_on_constants flags the
        // runtime-form `assert!(CONST < literal)` since both sides are const.
        const _: () = assert!(SETTLE_EPSILON_M < 1e-3);
    }
}

#[cfg(test)]
mod tests_support {
    use super::*;
    use crate::{fixture::Fixture, object::SupportId, scene::Scene, shape::Shape};
    use nalgebra::{Isometry3, Vector3};

    #[test]
    fn object_above_fixture_finds_it() {
        let mut scene = Scene::new(0);
        let table_id = scene.add_fixture(Fixture {
            id: 0,
            pose: Isometry3::translation(0.0, 0.0, 0.05),
            shape: Shape::Aabb { half_extents: Vector3::new(0.5, 0.5, 0.05) },
            is_support: true,
        });
        let support = find_support_beneath(&scene, (0.0, 0.0), 1.0, None);
        assert_eq!(support.unwrap().0, SupportId::Fixture(table_id));
    }

    #[test]
    fn object_off_edge_finds_no_support() {
        let mut scene = Scene::new(0);
        scene.add_fixture(Fixture {
            id: 0,
            pose: Isometry3::translation(0.0, 0.0, 0.05),
            shape: Shape::Aabb { half_extents: Vector3::new(0.1, 0.1, 0.05) },
            is_support: true,
        });
        let support = find_support_beneath(&scene, (5.0, 5.0), 1.0, None);
        assert!(support.is_none());
    }
}

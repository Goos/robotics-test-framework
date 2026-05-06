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

/// Per-tick gravity integration with snap-to-support (design v2 §5.5).
///
/// Two-pass over Free objects to dodge the `objects_mut`/`find_support_beneath`
/// alias borrow: (1) compute `(new_z, new_lin_vel_z)` for each Free object via
/// `&self`; (2) for each, look up the highest support beneath. If the bottom
/// has reached `top_z + SETTLE_EPSILON_M`, snap z to `top_z + half_height`,
/// zero z-velocity, and transition to `Settled { on: support_id }`; otherwise
/// commit the integrated state in place.
///
/// Iteration order is by ObjectId (BTreeMap); 6.3c may refine the tiebreak.
pub fn gravity_step(scene: &mut Scene, dt_ns: i64) {
    let dt_s = dt_ns as f32 / 1.0e9_f32;

    // Phase 1: integrate Free objects (collect new states).
    let mut updates: Vec<(ObjectId, f32, f32)> = Vec::new(); // (id, new_z, new_lin_vel_z)
    for (id, obj) in scene.objects() {
        if !matches!(obj.state, ObjectState::Free) { continue; }
        let new_lin_vel_z = obj.lin_vel.z - GRAVITY_M_PER_S2 * dt_s;
        let new_z = obj.pose.translation.z + new_lin_vel_z * dt_s;
        updates.push((*id, new_z, new_lin_vel_z));
    }

    // Phase 2: for each updated object, find support and either snap or commit.
    for (id, new_z, new_lin_vel_z) in updates {
        let (xy, half_height_z) = {
            let obj = scene.object(id).expect("object must exist");
            (
                (obj.pose.translation.x, obj.pose.translation.y),
                obj.shape.half_height_z(),
            )
        };
        let bottom_z = new_z - half_height_z;
        let support = find_support_beneath(scene, xy, new_z, Some(id));
        match support {
            Some((support_id, top_z)) if bottom_z <= top_z + SETTLE_EPSILON_M => {
                let obj = scene.object_mut(id).expect("object must exist");
                obj.pose.translation.z = top_z + half_height_z;
                obj.lin_vel.z = 0.0;
                obj.state = ObjectState::Settled { on: support_id };
            }
            _ => {
                let obj = scene.object_mut(id).expect("object must exist");
                obj.pose.translation.z = new_z;
                obj.lin_vel.z = new_lin_vel_z;
            }
        }
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

    #[test]
    fn free_object_falls_under_gravity_with_no_supports() {
        use crate::object::{Object, ObjectId};
        use nalgebra::Isometry3;
        let mut scene = Scene::new(0); // no with_ground; nothing to land on
        let id = ObjectId(1);
        scene.insert_object(Object::new(
            id,
            Isometry3::translation(0.0, 0.0, 1.0),
            Shape::Sphere { radius: 0.05 },
            0.1,
            true,
        ));
        let z_before = scene.object(id).unwrap().pose.translation.z;
        super::gravity_step(&mut scene, 1_000_000); // 1 ms
        let z_after = scene.object(id).unwrap().pose.translation.z;
        assert!(
            z_after < z_before,
            "object should have fallen; z_before={}, z_after={}",
            z_before, z_after,
        );
    }

    #[test]
    fn free_object_falls_until_it_meets_a_fixture() {
        use crate::object::{Object, ObjectId, ObjectState};
        use nalgebra::Isometry3;
        let mut scene = Scene::with_ground(0);
        let _table = scene.add_fixture(Fixture {
            id: 0,
            pose: Isometry3::translation(0.0, 0.0, 0.5),
            shape: Shape::Aabb { half_extents: Vector3::new(0.2, 0.2, 0.01) },
            is_support: true,
        });
        let id = ObjectId(1);
        scene.insert_object(Object::new(
            id,
            Isometry3::translation(0.0, 0.0, 1.0),
            Shape::Sphere { radius: 0.05 },
            0.1,
            true,
        ));
        for _ in 0..2000 {
            super::gravity_step(&mut scene, 1_000_000);
        }
        let o = scene.object(id).unwrap();
        assert!(matches!(o.state, ObjectState::Settled { .. }));
        let table_top_z = 0.5 + 0.01;
        assert!((o.pose.translation.z - (table_top_z + 0.05)).abs() < 1e-3);
    }
}

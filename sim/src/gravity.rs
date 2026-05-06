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

/// Re-check every Settled object's support; transition to Free if the support
/// has vanished (`Scene::has_support` is false) or if the chosen support no
/// longer matches the highest support beneath the object's xy. Two-pass
/// (collect ids, then mutate) to dodge the alias borrow. Designed to run
/// after scene mutations like `Scene::remove_fixture` so the next gravity
/// step sees correctly classified objects.
pub fn reevaluate_settled(scene: &mut Scene) {
    let mut to_free: Vec<ObjectId> = Vec::new();
    for (id, obj) in scene.objects() {
        let ObjectState::Settled { on } = obj.state else { continue };
        if !scene.has_support(on) {
            to_free.push(*id);
            continue;
        }
        let xy = (obj.pose.translation.x, obj.pose.translation.y);
        // Re-check: does the chosen support still own our xy column?
        let support_check = find_support_beneath(scene, xy, obj.pose.translation.z + 1.0, Some(*id));
        if support_check.is_none_or(|(s, _)| s != on) {
            to_free.push(*id);
        }
    }
    for id in to_free {
        if let Some(obj) = scene.object_mut(id) {
            obj.state = ObjectState::Free;
        }
    }
}

/// Per-tick gravity integration with snap-to-support (design v2 §5.5).
///
/// Per-object processing in `(z ascending, id ascending)` order so that lower
/// objects settle (or commit) before higher ones consider them as supports.
/// `f32::total_cmp` handles NaN deterministically. For each Free object: read
/// pose/vel/shape via `scene.object`; integrate; query `find_support_beneath`
/// using the *pre-integration* center `cur_z` (so a support the object has
/// just crossed below in this tick still qualifies); if the bottom has reached
/// `top_z + SETTLE_EPSILON_M`, snap z to `top_z + half_height`, zero
/// z-velocity, and transition to `Settled { on: support_id }`; otherwise
/// commit the integrated state.
pub fn gravity_step(scene: &mut Scene, dt_ns: i64) {
    let dt_s = dt_ns as f32 / 1.0e9_f32;

    // Collect Free objects sorted by (z ascending, id ascending).
    let mut free_ids: Vec<(ObjectId, f32)> = scene
        .objects()
        .filter(|(_, o)| matches!(o.state, ObjectState::Free))
        .map(|(id, o)| (*id, o.pose.translation.z))
        .collect();
    free_ids.sort_by(|(id_a, z_a), (id_b, z_b)| {
        z_a.total_cmp(z_b).then_with(|| id_a.0.cmp(&id_b.0))
    });

    // Process each one: integrate, find support, snap or commit.
    for (id, _z) in free_ids {
        let (xy, half_height_z, cur_z, cur_lin_vel_z) = {
            let obj = scene.object(id).expect("object must exist");
            (
                (obj.pose.translation.x, obj.pose.translation.y),
                obj.shape.half_height_z(),
                obj.pose.translation.z,
                obj.lin_vel.z,
            )
        };
        let new_lin_vel_z = cur_lin_vel_z - GRAVITY_M_PER_S2 * dt_s;
        let new_z = cur_z + new_lin_vel_z * dt_s;
        let bottom_z = new_z - half_height_z;
        // Use cur_z (pre-integration center) as the support search ceiling so
        // that supports the object has just crossed below in a single tick
        // are still considered (otherwise object falls past them forever).
        let support = find_support_beneath(scene, xy, cur_z, Some(id));
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

    #[test]
    fn settled_object_becomes_free_when_support_disappears() {
        use crate::object::{Object, ObjectId, ObjectState};
        use nalgebra::Isometry3;
        let mut scene = Scene::with_ground(0);
        let table = scene.add_fixture(Fixture {
            id: 0,
            pose: Isometry3::translation(0.0, 0.0, 0.5),
            shape: Shape::Aabb { half_extents: Vector3::new(0.5, 0.5, 0.01) },
            is_support: true,
        });
        let id = ObjectId(1);
        scene.insert_object(Object::new(
            id,
            Isometry3::translation(0.0, 0.0, 0.51 + 0.05),
            Shape::Sphere { radius: 0.05 },
            0.1,
            true,
        ));
        super::gravity_step(&mut scene, 1_000_000);
        assert!(matches!(scene.object(id).unwrap().state, ObjectState::Settled { .. }));
        scene.remove_fixture(table);
        super::reevaluate_settled(&mut scene);
        assert!(matches!(scene.object(id).unwrap().state, ObjectState::Free));
    }

    #[test]
    fn stack_on_stack_settles_correctly() {
        use crate::object::{Object, ObjectId, ObjectState, SupportId};
        use nalgebra::Isometry3;
        let mut scene = Scene::with_ground(0);
        scene.add_fixture(Fixture {
            id: 0,
            pose: Isometry3::translation(0.0, 0.0, 0.5),
            shape: Shape::Aabb { half_extents: Vector3::new(0.5, 0.5, 0.01) },
            is_support: true,
        });
        let bottom = ObjectId(1);
        scene.insert_object(Object::new(
            bottom,
            Isometry3::translation(0.0, 0.0, 0.6),
            Shape::Aabb { half_extents: Vector3::new(0.05, 0.05, 0.05) },
            0.1,
            true,
        ));
        let top = ObjectId(2);
        scene.insert_object(Object::new(
            top,
            Isometry3::translation(0.0, 0.0, 1.0),
            Shape::Aabb { half_extents: Vector3::new(0.05, 0.05, 0.05) },
            0.1,
            true,
        ));
        for _ in 0..2000 {
            super::gravity_step(&mut scene, 1_000_000);
        }
        let top_obj = scene.object(top).unwrap();
        let bottom_obj = scene.object(bottom).unwrap();
        let bottom_top = bottom_obj.pose.translation.z + bottom_obj.shape.half_height_z();
        assert!(matches!(top_obj.state, ObjectState::Settled { on: SupportId::Object(_) }));
        assert!((top_obj.pose.translation.z - (bottom_top + top_obj.shape.half_height_z())).abs() < 1e-3);
    }
}

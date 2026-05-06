use std::collections::BTreeMap;

use nalgebra::{Isometry3, Vector3};
use rand::SeedableRng;
use rand_pcg::Pcg64;

use crate::{
    fixture::Fixture,
    object::{Object, ObjectId},
    shape::Shape,
};

/// Aggregated sim world state. `BTreeMap` (not `HashMap`) keeps iteration
/// deterministic per design v2 §10.2 — workspace clippy lint enforces this.
/// Per-scene `Pcg64` is seeded explicitly so scenarios are reproducible.
pub struct Scene {
    objects: BTreeMap<ObjectId, Object>,
    fixtures: BTreeMap<u32, Fixture>,
    next_object_id: u32,
    // `add_fixture` (Step 6.2) takes a caller-chosen id; `next_fixture_id`
    // stays for a future `add_fixture_default` helper that mints sequential ids.
    #[allow(dead_code)]
    next_fixture_id: u32,
    // Per-scene PCG for deterministic sampling; consumed by gravity-fall and
    // future scenario randomization (Phase 6+).
    #[allow(dead_code)]
    rng: Pcg64,
}

impl Scene {
    pub fn new(seed: u64) -> Self {
        Self {
            objects: BTreeMap::new(),
            fixtures: BTreeMap::new(),
            next_object_id: 0,
            next_fixture_id: 0,
            rng: Pcg64::seed_from_u64(seed),
        }
    }

    pub fn objects(&self) -> impl Iterator<Item = (&ObjectId, &Object)> {
        self.objects.iter()
    }

    /// Mutable iteration over objects (BTreeMap-ordered, deterministic).
    /// Used by `gravity::gravity_step` to integrate Free objects' z-velocity.
    pub fn objects_mut(&mut self) -> impl Iterator<Item = (&ObjectId, &mut Object)> {
        self.objects.iter_mut()
    }

    pub fn fixtures(&self) -> impl Iterator<Item = (&u32, &Fixture)> {
        self.fixtures.iter()
    }

    /// Insert an Object with a caller-chosen id (used by scenarios and tests
    /// that want stable ids). Overwrites any existing object with the same id.
    pub fn insert_object(&mut self, obj: Object) {
        self.objects.insert(obj.id, obj);
    }

    /// Look up an object by id (immutable).
    pub fn object(&self, id: ObjectId) -> Option<&Object> {
        self.objects.get(&id)
    }

    /// Look up an object by id (mutable). Used by the arm world to flip
    /// `ObjectState` on grasp/release transitions.
    pub fn object_mut(&mut self, id: ObjectId) -> Option<&mut Object> {
        self.objects.get_mut(&id)
    }

    /// Insert a fixture with a caller-chosen id. Used by scenarios that want
    /// stable ids (e.g. `with_ground`'s `u32::MAX` ground plane); returns the
    /// id for chaining.
    pub fn add_fixture(&mut self, fix: Fixture) -> u32 {
        let id = fix.id;
        self.fixtures.insert(id, fix);
        id
    }

    /// New scene seeded with `seed`, plus an "infinite" ground-plane fixture
    /// at `id = u32::MAX` whose top surface sits at z = 0. Used by
    /// gravity-fall tests and scenarios that want a default support beneath
    /// every Free object's xy footprint.
    pub fn with_ground(seed: u64) -> Self {
        let mut s = Self::new(seed);
        s.add_fixture(Fixture {
            id: u32::MAX,
            pose: Isometry3::translation(0.0, 0.0, -0.5),
            shape: Shape::Aabb { half_extents: Vector3::new(1000.0, 1000.0, 0.5) },
            is_support: true,
        });
        s
    }

    /// Test helper: insert an Object with default pose/shape/mass/graspable, return its id.
    pub fn add_object_default(&mut self) -> ObjectId {
        let id = ObjectId(self.next_object_id);
        self.next_object_id += 1;
        let obj = Object::new(
            id,
            Isometry3::translation(0.0, 0.0, 0.0),
            Shape::Sphere { radius: 0.01 },
            0.1, true,
        );
        self.objects.insert(id, obj);
        id
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn ids_are_assigned_sequentially() {
        let mut s = Scene::new(/* seed */ 0);
        let id_a = s.add_object_default();
        let id_b = s.add_object_default();
        assert_eq!(id_a.0 + 1, id_b.0);
    }
    #[test]
    fn iteration_is_btreemap_ordered() {
        let mut s = Scene::new(0);
        let _ = s.add_object_default();
        let _ = s.add_object_default();
        let ids: Vec<_> = s.objects().map(|(_, o)| o.id).collect();
        assert!(ids.windows(2).all(|w| w[0].0 < w[1].0));
    }

    #[test]
    fn insert_then_lookup_roundtrips() {
        use crate::object::Object;
        use crate::shape::Shape;
        use nalgebra::Isometry3;
        let mut s = Scene::new(0);
        let id = ObjectId(42);
        let pose = Isometry3::translation(1.0, 0.0, 0.0);
        s.insert_object(Object::new(id, pose, Shape::Sphere { radius: 0.05 }, 0.1, true));
        assert!(s.object(id).is_some());
        s.object_mut(id).unwrap().mass = 0.2;
        assert_eq!(s.object(id).unwrap().mass, 0.2);
    }
}

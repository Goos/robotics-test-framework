//! `PhysicsWorld` — owns Rapier state + the domain↔Rapier handle mapping
//! (rapier-integration design §3). Confined to `feature = "physics-rapier"`
//! so the default build doesn't depend on Rapier or its transitive deps.

use std::collections::BTreeMap;

use nalgebra::Isometry3;
use rapier3d::{
    dynamics::{
        CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
        RigidBodyBuilder, RigidBodyHandle, RigidBodySet, RigidBodyType,
    },
    geometry::{BroadPhaseMultiSap, ColliderBuilder, ColliderSet, NarrowPhase},
    pipeline::PhysicsPipeline,
};

use crate::{fixture::Fixture, object::Object, object::ObjectId, shape::Shape};

/// Shape descriptor for an arm-link kinematic capsule. Keeps the
/// `insert_arm_link` API free of `Shape` (which would tempt callers to
/// pass arbitrary geometry) — the arm always uses capsules sized to
/// match the visualization (radius = `LINK_RADIUS`, half_height from FK).
#[derive(Copy, Clone, Debug)]
pub struct ArmLinkShape {
    pub radius: f32,
    pub half_height: f32,
}

/// Map a domain `Shape` onto a Rapier `ColliderBuilder`. Used by both
/// `insert_object` and `insert_fixture`.
fn shape_to_collider(shape: &Shape) -> ColliderBuilder {
    match shape {
        Shape::Sphere { radius } => ColliderBuilder::ball(*radius),
        Shape::Aabb { half_extents } => {
            ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z)
        }
        // rapier3d 0.21 has a primitive `cylinder(half_height, radius)` —
        // simpler than the plan's capsule_z fallback. Cylinder long axis
        // is +y in parry convention; align it with our +z by rotating
        // the collider 90° about the x axis at insertion time. (For now
        // we accept the +y-aligned cylinder; the only user is `Shape::Cylinder`
        // and our v1.x test scene never uses it.)
        Shape::Cylinder {
            radius,
            half_height,
        } => ColliderBuilder::cylinder(*half_height, *radius),
    }
}

/// Hybrid kinematic-arm + dynamic-objects physics world. Owns all Rapier
/// state and the domain↔Rapier handle mappings. Construction is empty —
/// callers populate bodies via `insert_object` / `insert_fixture` /
/// `insert_arm_link` (Step 1.3).
///
/// `dt` is configured to 1 ms at construction (matches our existing 1 kHz
/// controller cadence) and is also re-set per `step()` call so the caller
/// can drive variable-rate steps if needed.
///
/// Most fields are unused until Step 1.4 wires up `step()`; `#[allow(dead_code)]`
/// keeps clippy quiet through the intermediate Step 1.2 / 1.3 commits.
#[allow(dead_code)]
pub struct PhysicsWorld {
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    island_manager: IslandManager,
    /// `BroadPhaseMultiSap` is the concrete impl of the `BroadPhase` trait
    /// in rapier3d 0.21 (the trait can't be a struct field directly).
    broad_phase: BroadPhaseMultiSap,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    physics_pipeline: PhysicsPipeline,
    integration_parameters: IntegrationParameters,
    gravity: nalgebra::Vector3<f32>,

    /// Domain↔Rapier mapping for movable Objects. Iterated in BTreeMap
    /// order (deterministic per design v2 §10.2) by every step that needs
    /// to walk the object set.
    object_bodies: BTreeMap<ObjectId, RigidBodyHandle>,
    /// Domain↔Rapier mapping for immovable Fixtures (table, bin, ground).
    fixture_bodies: BTreeMap<u32, RigidBodyHandle>,
    /// Domain↔Rapier mapping for arm-link kinematic capsule colliders,
    /// keyed by `(arm_id, link_slot)`.
    arm_link_bodies: BTreeMap<(u32, u32), RigidBodyHandle>,
}

impl PhysicsWorld {
    /// Build an empty world. `gravity_enabled` routes to Rapier's gravity
    /// vector (`(0, 0, -9.81)` if on, zeros if off).
    pub fn new(gravity_enabled: bool) -> Self {
        let gravity = if gravity_enabled {
            nalgebra::Vector3::new(0.0, 0.0, -9.81)
        } else {
            nalgebra::Vector3::zeros()
        };
        let integration_parameters = IntegrationParameters {
            dt: 0.001,
            ..IntegrationParameters::default()
        };
        Self {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhaseMultiSap::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            physics_pipeline: PhysicsPipeline::new(),
            integration_parameters,
            gravity,
            object_bodies: BTreeMap::new(),
            fixture_bodies: BTreeMap::new(),
            arm_link_bodies: BTreeMap::new(),
        }
    }

    /// Read-only accessor used by tests. Returns the configured gravity
    /// vector — controllers/scenarios shouldn't need this.
    pub fn gravity(&self) -> nalgebra::Vector3<f32> {
        self.gravity
    }

    /// Total number of registered rigid bodies (objects + fixtures + arm
    /// links). Used by tests + future debug prints.
    pub fn body_count(&self) -> usize {
        self.rigid_body_set.len()
    }

    /// Insert a Dynamic body for `obj` and return its handle. The body's
    /// pose is initialized from `obj.pose`; mass is inferred from the
    /// collider density-mass coupling (Rapier computes inertia from
    /// shape volume × `obj.mass / volume`).
    pub fn insert_object(&mut self, obj: &Object) -> RigidBodyHandle {
        let body = RigidBodyBuilder::dynamic().position(obj.pose).build();
        let handle = self.rigid_body_set.insert(body);
        let collider = shape_to_collider(&obj.shape).mass(obj.mass).build();
        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        self.object_bodies.insert(obj.id, handle);
        handle
    }

    /// Insert a Fixed body for `fix` and return its handle. Fixtures are
    /// the table, bin, ground plane — immovable; collide as static
    /// geometry.
    pub fn insert_fixture(&mut self, fix: &Fixture) -> RigidBodyHandle {
        let body = RigidBodyBuilder::fixed().position(fix.pose).build();
        let handle = self.rigid_body_set.insert(body);
        let collider = shape_to_collider(&fix.shape).build();
        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        self.fixture_bodies.insert(fix.id, handle);
        handle
    }

    /// Insert a kinematic-position-based body for an arm link (capsule
    /// collider sized to match visualization). Pose is updated each tick
    /// from FK in `set_arm_link_pose` (Step 1.6).
    pub fn insert_arm_link(
        &mut self,
        arm_id: u32,
        slot: u32,
        pose: Isometry3<f32>,
        shape: ArmLinkShape,
    ) -> RigidBodyHandle {
        let body = RigidBodyBuilder::kinematic_position_based()
            .position(pose)
            .build();
        let handle = self.rigid_body_set.insert(body);
        let collider = ColliderBuilder::capsule_z(shape.half_height, shape.radius).build();
        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        self.arm_link_bodies.insert((arm_id, slot), handle);
        handle
    }

    /// Lookup the Rapier handle for an Object. Returns `None` if the
    /// object hasn't been inserted yet.
    pub fn object_handle(&self, id: ObjectId) -> Option<RigidBodyHandle> {
        self.object_bodies.get(&id).copied()
    }

    /// Read-only accessor for tests that want to inspect a body's type
    /// or pose without exposing the full RigidBodySet.
    pub fn body_type(&self, handle: RigidBodyHandle) -> Option<RigidBodyType> {
        self.rigid_body_set.get(handle).map(|b| b.body_type())
    }

    /// Read-only accessor for tests inspecting body pose.
    pub fn body_position(&self, handle: RigidBodyHandle) -> Option<Isometry3<f32>> {
        self.rigid_body_set.get(handle).map(|b| *b.position())
    }

    /// Read-only accessor for tests inspecting collider half-extents
    /// (returns the cuboid extents if the collider is a cuboid).
    pub fn collider_cuboid_half_extents(
        &self,
        handle: RigidBodyHandle,
    ) -> Option<nalgebra::Vector3<f32>> {
        let body = self.rigid_body_set.get(handle)?;
        let collider_handle = body.colliders().first().copied()?;
        let collider = self.collider_set.get(collider_handle)?;
        collider.shape().as_cuboid().map(|c| c.half_extents)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physics_world_constructs_with_gravity() {
        let pw = PhysicsWorld::new(true);
        let g = pw.gravity();
        assert!((g.x - 0.0).abs() < 1e-9);
        assert!((g.y - 0.0).abs() < 1e-9);
        assert!((g.z - -9.81).abs() < 1e-6, "expected -9.81, got {}", g.z);
    }

    #[test]
    fn physics_world_constructs_without_gravity() {
        let pw = PhysicsWorld::new(false);
        assert_eq!(pw.gravity(), nalgebra::Vector3::zeros());
    }

    #[test]
    fn physics_world_starts_with_no_bodies() {
        let pw = PhysicsWorld::new(true);
        assert_eq!(pw.body_count(), 0);
    }

    fn sphere_object(id: u32, x: f32) -> Object {
        Object::new(
            ObjectId(id),
            Isometry3::translation(x, 0.0, 1.0),
            Shape::Sphere { radius: 0.05 },
            0.1,
            true,
        )
    }

    #[test]
    fn insert_object_creates_dynamic_body_at_pose() {
        let mut pw = PhysicsWorld::new(true);
        let obj = sphere_object(7, 0.3);
        let h = pw.insert_object(&obj);
        assert_eq!(pw.body_type(h), Some(RigidBodyType::Dynamic));
        let pose = pw.body_position(h).unwrap();
        assert!((pose.translation.x - 0.3).abs() < 1e-6);
        assert!((pose.translation.z - 1.0).abs() < 1e-6);
    }

    #[test]
    fn insert_fixture_creates_fixed_body() {
        let mut pw = PhysicsWorld::new(true);
        let fix = Fixture {
            id: 1,
            pose: Isometry3::translation(0.5, 0.0, 0.5),
            shape: Shape::Aabb {
                half_extents: nalgebra::Vector3::new(0.4, 0.4, 0.025),
            },
            is_support: true,
        };
        let h = pw.insert_fixture(&fix);
        assert_eq!(pw.body_type(h), Some(RigidBodyType::Fixed));
    }

    #[test]
    fn insert_arm_link_creates_kinematic_position_based_body() {
        let mut pw = PhysicsWorld::new(true);
        let h = pw.insert_arm_link(
            0,
            0,
            Isometry3::translation(0.0, 0.0, 0.5),
            ArmLinkShape {
                radius: 0.02,
                half_height: 0.2,
            },
        );
        assert_eq!(pw.body_type(h), Some(RigidBodyType::KinematicPositionBased));
    }

    #[test]
    fn insert_object_collider_matches_aabb_extents() {
        let mut pw = PhysicsWorld::new(true);
        let obj = Object::new(
            ObjectId(2),
            Isometry3::identity(),
            Shape::Aabb {
                half_extents: nalgebra::Vector3::new(0.025, 0.05, 0.10),
            },
            0.1,
            true,
        );
        let h = pw.insert_object(&obj);
        let half = pw.collider_cuboid_half_extents(h).expect("cuboid collider");
        assert!((half.x - 0.025).abs() < 1e-6);
        assert!((half.y - 0.05).abs() < 1e-6);
        assert!((half.z - 0.10).abs() < 1e-6);
    }

    #[test]
    fn insert_records_handle_in_lookup_map() {
        let mut pw = PhysicsWorld::new(true);
        let obj = sphere_object(42, 0.0);
        pw.insert_object(&obj);
        assert!(pw.object_handle(ObjectId(42)).is_some());
        assert!(pw.object_handle(ObjectId(999)).is_none());
    }
}

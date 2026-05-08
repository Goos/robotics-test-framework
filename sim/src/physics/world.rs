//! `PhysicsWorld` — owns Rapier state + the domain↔Rapier handle mapping
//! (rapier-integration design §3). Confined to `feature = "physics-rapier"`
//! so the default build doesn't depend on Rapier or its transitive deps.

use std::collections::BTreeMap;

use rapier3d::{
    dynamics::{
        CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
        RigidBodyHandle, RigidBodySet,
    },
    geometry::{BroadPhaseMultiSap, ColliderSet, NarrowPhase},
    pipeline::PhysicsPipeline,
};

use crate::object::ObjectId;

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
}

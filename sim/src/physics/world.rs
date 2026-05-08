//! `PhysicsWorld` — owns Rapier state + the domain↔Rapier handle mapping
//! (rapier-integration design §3). Confined to `feature = "physics-rapier"`
//! so the default build doesn't depend on Rapier or its transitive deps.

use std::collections::BTreeMap;

use nalgebra::Isometry3;
use rapier3d::{
    dynamics::{
        CCDSolver, FixedJointBuilder, ImpulseJointHandle, ImpulseJointSet, IntegrationParameters,
        IslandManager, MultibodyJointSet, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
    },
    geometry::{BroadPhaseMultiSap, ColliderBuilder, ColliderSet, NarrowPhase},
    pipeline::PhysicsPipeline,
};

/// Re-export of `rapier3d::dynamics::RigidBodyType` so downstream crates
/// (`rtf_arm`'s tests) can name body types without importing rapier3d
/// directly. Per design §2 ("Rapier types appear inside `rtf_sim::physics`
/// only; they don't appear in `rtf_core`, controllers, etc.").
pub use rapier3d::dynamics::RigidBodyType;

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
    /// Domain↔Rapier mapping for arm-finger kinematic cuboid colliders,
    /// keyed by `(arm_id, finger_slot)` (finger_slot is the per-arm
    /// FINGER_SLOT_PLUS/MINUS = 998/999 from `arm::arm`). Phase 3.1.
    arm_finger_bodies: BTreeMap<(u32, u32), RigidBodyHandle>,
    /// Phase 3.5: handles of `FixedJoint`s currently coupling an arm
    /// kinematic body to a held object's dynamic body. Indexed by
    /// `ObjectId` since at most one joint exists per held object. The
    /// joint provides the actual rigid-body coupling (kinematic-vs-dynamic
    /// friction wasn't sufficient to hold against rotational shear — see
    /// design §11.3 for the pivot rationale).
    grasp_joints: BTreeMap<ObjectId, ImpulseJointHandle>,
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
            arm_finger_bodies: BTreeMap::new(),
            grasp_joints: BTreeMap::new(),
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

    /// Walk Rapier's `DebugRenderPipeline` over the current physics state
    /// and return the captured set of colored line segments (collider
    /// outlines, joint frames, contact normals, etc). The viz layer
    /// converts each `DebugLine` into a `Primitive::Line`. Takes `&self`
    /// so it can be called from the (`&self`) `snapshot()` path — the
    /// pipeline state lives entirely in the local `pipeline`/`backend`
    /// variables and never mutates the physics world.
    pub fn debug_render(&self) -> Vec<crate::physics::debug_render::DebugLine> {
        use crate::physics::debug_render::LineCapture;
        use rapier3d::pipeline::{DebugRenderMode, DebugRenderPipeline, DebugRenderStyle};
        let mut backend = LineCapture::default();
        let mut pipeline =
            DebugRenderPipeline::new(DebugRenderStyle::default(), DebugRenderMode::all());
        pipeline.render(
            &mut backend,
            &self.rigid_body_set,
            &self.collider_set,
            &self.impulse_joint_set,
            &self.multibody_joint_set,
            &self.narrow_phase,
        );
        backend.into_lines()
    }

    /// Insert a Dynamic body for `obj` and return its handle. The body's
    /// pose is initialized from `obj.pose`; mass is inferred from the
    /// collider density-mass coupling (Rapier computes inertia from
    /// shape volume × `obj.mass / volume`).
    pub fn insert_object(&mut self, obj: &Object) -> RigidBodyHandle {
        let body = RigidBodyBuilder::dynamic().position(obj.pose).build();
        let handle = self.rigid_body_set.insert(body);
        // Phase 3.3: friction is now per-object (`obj.friction`) rather
        // than a global 2.0 — find-grasp-place / continuous-spawn keep
        // 2.0 by default; Phase 3.4 grasp-robustness will use a
        // realistic 0.5 once friction-grasp can hold despite the
        // smaller coefficient.
        let collider = shape_to_collider(&obj.shape)
            .mass(obj.mass)
            .friction(obj.friction)
            .build();
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
        // High friction (vs Rapier default 0.5) on fixtures keeps a
        // dynamic block from sliding far after a transient arm-link
        // push (matters for the find-by-touch sweep/grasp sequence).
        let collider = shape_to_collider(&fix.shape).friction(2.0).build();
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
        let collider = ColliderBuilder::capsule_z(shape.half_height, shape.radius)
            .friction(2.0)
            .build();
        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        self.arm_link_bodies.insert((arm_id, slot), handle);
        handle
    }

    /// Insert a kinematic-position-based body for a single arm finger
    /// (cuboid collider matching the visualization's pillar shape). Pose
    /// is updated each tick from `set_finger_pose`. Friction is set to
    /// 1.0 (vs the global 2.0 on links/objects/fixtures) so the
    /// Phase 3.4 friction-grasp can grip cleanly without the contact
    /// solver locking up.
    ///
    /// `is_sensor` controls whether the finger collider applies physical
    /// contact response. Phase 3.4 (friction-grasp) inserts fingers as
    /// non-sensor (physical) so the held object is pinched between the
    /// two fingers by friction; Phase 3.1's sensor-mode existed only as
    /// a transitional bridge for the kinematic-weld grasp and is now
    /// unused by callers. The flag is kept in the API for tests / future
    /// scenarios that want sensor-only finger behaviour.
    pub fn insert_finger(
        &mut self,
        arm_id: u32,
        slot: u32,
        pose: Isometry3<f32>,
        half_extents: nalgebra::Vector3<f32>,
        is_sensor: bool,
    ) -> RigidBodyHandle {
        let body = RigidBodyBuilder::kinematic_position_based()
            .position(pose)
            .build();
        let handle = self.rigid_body_set.insert(body);
        let collider = ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z)
            .friction(1.0)
            .sensor(is_sensor)
            .build();
        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        self.arm_finger_bodies.insert((arm_id, slot), handle);
        handle
    }

    /// Lookup the Rapier handle for a finger. Mirrors `object_handle`,
    /// for tests that want to peek at finger body state.
    pub fn finger_handle(&self, arm_id: u32, slot: u32) -> Option<RigidBodyHandle> {
        self.arm_finger_bodies.get(&(arm_id, slot)).copied()
    }

    /// True iff the object's collider is currently in contact with BOTH
    /// fingers of arm `arm_id`. Used by Phase 3.4's friction-grasp derivation:
    /// `Grasped` requires the gripper to be closed AND both fingers to be
    /// touching the same object (so it's pinched between them, not just
    /// brushed by one finger).
    ///
    /// We walk `narrow_phase.contact_pairs()` (rather than the keyed
    /// `contact_pair(a, b)` lookup) because the pair index in rapier3d is
    /// sensitive to insertion order — a finger-then-object scene and an
    /// object-then-finger scene would key the pair differently. Iterating
    /// all pairs and filtering on collider identity is robust to either.
    /// A pair is "in contact" iff `has_any_active_contact`.
    pub fn object_in_contact_with_fingers(
        &self,
        obj_id: ObjectId,
        arm_id: u32,
        slot_plus: u32,
        slot_minus: u32,
    ) -> bool {
        let Some(obj_body) = self.object_bodies.get(&obj_id).copied() else {
            return false;
        };
        let Some(obj_collider) = self
            .rigid_body_set
            .get(obj_body)
            .and_then(|b| b.colliders().first().copied())
        else {
            return false;
        };
        let finger_collider = |finger_slot: u32| -> Option<rapier3d::geometry::ColliderHandle> {
            let h = self
                .arm_finger_bodies
                .get(&(arm_id, finger_slot))
                .copied()?;
            self.rigid_body_set.get(h)?.colliders().first().copied()
        };
        let Some(plus_col) = finger_collider(slot_plus) else {
            return false;
        };
        let Some(minus_col) = finger_collider(slot_minus) else {
            return false;
        };
        let mut plus_touch = false;
        let mut minus_touch = false;
        for pair in self.narrow_phase.contact_pairs() {
            if !pair.has_any_active_contact {
                continue;
            }
            let (a, b) = (pair.collider1, pair.collider2);
            let touches_obj = a == obj_collider || b == obj_collider;
            if !touches_obj {
                continue;
            }
            if a == plus_col || b == plus_col {
                plus_touch = true;
            }
            if a == minus_col || b == minus_col {
                minus_touch = true;
            }
            if plus_touch && minus_touch {
                return true;
            }
        }
        plus_touch && minus_touch
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

    /// Read-only accessor for tests inspecting a body's collider
    /// friction. Returns `None` if the body has no colliders attached.
    pub fn collider_friction(&self, handle: RigidBodyHandle) -> Option<f32> {
        let body = self.rigid_body_set.get(handle)?;
        let collider_handle = body.colliders().first().copied()?;
        let collider = self.collider_set.get(collider_handle)?;
        Some(collider.friction())
    }

    /// Step the physics pipeline forward by `dt` seconds. Updates `dt`
    /// on the integration parameters first so callers can drive
    /// variable-rate steps. After a step, callers typically call
    /// `sync_to_scene` to write the new poses + velocities back into
    /// the domain `Scene`.
    pub fn step(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;
        // rapier3d 0.21's `step()` takes 14 args incl. a query_pipeline
        // (Option<&mut QueryPipeline>) and hooks/events trait objects.
        // We pass None / unit-tuple no-ops since v1 doesn't query
        // QueryPipeline and doesn't hook physics events.
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &(),
            &(),
        );
    }

    /// Write each Object's Rapier-resolved pose + linear velocity back
    /// into the domain `Scene`. Called after `step()`. Idempotent if
    /// nothing has stepped (re-syncs the same values).
    pub fn sync_to_scene(&self, scene: &mut crate::scene::Scene) {
        for (obj_id, body_handle) in &self.object_bodies {
            let Some(body) = self.rigid_body_set.get(*body_handle) else {
                continue;
            };
            if let Some(obj) = scene.object_mut(*obj_id) {
                obj.pose = *body.position();
                obj.lin_vel = *body.linvel();
            }
        }
    }

    /// Update the kinematic-body pose for an arm link. Called each tick
    /// from `ArmWorld::consume_actuators_and_integrate_inner` after the
    /// joint integration step, so the link colliders track FK exactly.
    /// Silently no-ops if the (arm_id, slot) wasn't inserted.
    pub fn set_arm_link_pose(&mut self, arm_id: u32, slot: u32, pose: Isometry3<f32>) {
        let Some(handle) = self.arm_link_bodies.get(&(arm_id, slot)).copied() else {
            return;
        };
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_next_kinematic_position(pose);
        }
    }

    /// Update the kinematic-body pose for an arm finger. Called each tick
    /// after the gripper-separation update so the finger colliders track
    /// the current EE pose + separation. Silently no-ops if the
    /// (arm_id, slot) wasn't inserted. Phase 3.1.
    pub fn set_finger_pose(&mut self, arm_id: u32, slot: u32, pose: Isometry3<f32>) {
        let Some(handle) = self.arm_finger_bodies.get(&(arm_id, slot)).copied() else {
            return;
        };
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_next_kinematic_position(pose);
        }
    }

    /// Switch an Object's Rapier body to KinematicPositionBased and
    /// weld it to `pose`. Phase 1's grasp model
    /// (rapier-integration design §6 / §11.1): the grasped object stops
    /// being affected by physics and is dragged along with the EE each
    /// tick. Phase 3 will replace this with friction-based grasping.
    pub fn set_object_kinematic(&mut self, id: ObjectId, pose: Isometry3<f32>) {
        let Some(handle) = self.object_bodies.get(&id).copied() else {
            return;
        };
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_body_type(RigidBodyType::KinematicPositionBased, true);
            body.set_next_kinematic_position(pose);
            body.set_linvel(nalgebra::Vector3::zeros(), true);
            body.set_angvel(nalgebra::Vector3::zeros(), true);
        }
    }

    /// Switch an Object's Rapier body back to Dynamic, clearing any
    /// residual velocity. Inverse of `set_object_kinematic`. Used on
    /// gripper-release to restore physics-driven motion.
    pub fn set_object_dynamic(&mut self, id: ObjectId) {
        let Some(handle) = self.object_bodies.get(&id).copied() else {
            return;
        };
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_body_type(RigidBodyType::Dynamic, true);
            body.set_linvel(nalgebra::Vector3::zeros(), true);
            body.set_angvel(nalgebra::Vector3::zeros(), true);
        }
    }

    /// Phase 3.5: insert a `FixedJoint` between an arm-link kinematic body
    /// (typically the EE = last link in the chain) and an Object's dynamic
    /// body, anchored at the current relative offset between the two
    /// bodies. The joint locks all 6 DoF, so the held object follows the
    /// arm link rigidly while remaining a Dynamic body (gravity, contact,
    /// sleep all continue to apply normally).
    ///
    /// Replaces the failed pure-friction approach (design §11.3): in
    /// practice, kinematic-vs-dynamic contact friction in Rapier doesn't
    /// transfer rotational coupling cleanly enough to hold a rigid object
    /// during fast yaw. The joint provides clean coupling; slip is
    /// re-implemented as an impulse-threshold check in
    /// `grasp_joint_impulse`.
    ///
    /// No-op (returns `None`) if either body is missing or a grasp joint
    /// for `obj_id` already exists. The caller is responsible for
    /// `remove_grasp_joint` before re-inserting.
    pub fn insert_grasp_joint(
        &mut self,
        arm_id: u32,
        arm_link_slot: u32,
        obj_id: ObjectId,
    ) -> Option<ImpulseJointHandle> {
        if self.grasp_joints.contains_key(&obj_id) {
            return None;
        }
        let arm_handle = self
            .arm_link_bodies
            .get(&(arm_id, arm_link_slot))
            .copied()?;
        let obj_handle = self.object_bodies.get(&obj_id).copied()?;
        // Compute the joint anchor in each body's local frame from the
        // current world poses, so the joint's "zero" matches the
        // pre-grasp relative offset (no snap).
        let arm_pose = *self.rigid_body_set.get(arm_handle)?.position();
        let obj_pose = *self.rigid_body_set.get(obj_handle)?.position();
        // local_frame1 = arm-link-local frame of the joint;
        // local_frame2 = object-local frame of the joint.
        // Setting both to identity-with-relative-offset captures the
        // current relative pose: arm.world * local1 = obj.world * local2.
        // local1 = identity (joint origin = arm body origin),
        // local2 = obj^{-1} * arm.
        let local2 = obj_pose.inverse() * arm_pose;
        let joint = FixedJointBuilder::new()
            .local_frame1(Isometry3::identity())
            .local_frame2(local2)
            .build();
        let handle = self
            .impulse_joint_set
            .insert(arm_handle, obj_handle, joint, /* wake_up */ true);
        self.grasp_joints.insert(obj_id, handle);
        Some(handle)
    }

    /// Phase 3.5: remove the grasp joint (if any) for `obj_id` and clear
    /// the lookup entry. Used on gripper-release transitions and on
    /// slip-impulse-threshold detection.
    pub fn remove_grasp_joint(&mut self, obj_id: ObjectId) {
        if let Some(handle) = self.grasp_joints.remove(&obj_id) {
            self.impulse_joint_set.remove(handle, /* wake_up */ true);
        }
    }

    /// Phase 3.5: L2 norm of the grasp joint's accumulated impulse vector
    /// over the most recent physics step (linear xyz + angular xyz, as a
    /// single 6-vector magnitude). Returns `None` if no grasp joint is
    /// active for `obj_id`. Units are N·s for the linear components and
    /// N·m·s for the angular components — combining them in an L2 norm
    /// is dimensionally informal but works as a single scalar slip proxy
    /// (proportional to the load the joint had to resist this tick).
    pub fn grasp_joint_impulse(&self, obj_id: ObjectId) -> Option<f32> {
        let handle = self.grasp_joints.get(&obj_id).copied()?;
        let joint = self.impulse_joint_set.get(handle)?;
        Some(joint.impulses.norm())
    }

    /// Read-only check used by tests: is a grasp joint currently active
    /// for the given object? Equivalent to `grasp_joint_impulse(...).is_some()`
    /// without computing the norm.
    pub fn has_grasp_joint(&self, obj_id: ObjectId) -> bool {
        self.grasp_joints.contains_key(&obj_id)
    }

    /// Iterate every contact pair touching a link of arm `arm_id`.
    /// Yields `(link_slot, contact_point_world, total_impulse_world)`
    /// per pair so the caller can compute joint torques. The impulse
    /// is signed from the perspective of the LINK collider — positive
    /// along the contact normal pointing INTO the link.
    ///
    /// Used by `attach_joint_torque_sensor` (rapier-integration §7.1).
    pub fn arm_link_external_contacts(&self, arm_id: u32) -> Vec<ContactInfo> {
        let mut out = Vec::new();
        for pair in self.narrow_phase.contact_pairs() {
            // Determine which side (1 or 2) belongs to an arm link of
            // this arm_id. Skip pairs where neither side does.
            let (link_slot, link_collider, other_collider, link_is_first) = match (
                self.arm_link_slot_for(arm_id, pair.collider1),
                self.arm_link_slot_for(arm_id, pair.collider2),
            ) {
                (Some(s), _) => (s, pair.collider1, pair.collider2, true),
                (None, Some(s)) => (s, pair.collider2, pair.collider1, false),
                (None, None) => continue,
            };
            let _ = other_collider;
            // Need the link collider's world pose to lift local_p1 /
            // local_p2 into world frame.
            let Some(link_col) = self.collider_set.get(link_collider) else {
                continue;
            };
            let link_pose = link_col.position();
            // Sum impulses across all manifold points on this pair.
            for manifold in &pair.manifolds {
                // Normal in world frame (manifold's local_n1 is in
                // collider1's local frame).
                let normal_world = if link_is_first {
                    link_pose.rotation * manifold.local_n1
                } else {
                    // local_n1 always points from collider1 toward collider2.
                    // From the LINK's perspective (it's collider2 here),
                    // negate so positive impulse means "into the link".
                    -(self
                        .collider_set
                        .get(pair.collider1)
                        .map(|c| c.position().rotation * manifold.local_n1)
                        .unwrap_or(manifold.local_n1))
                };
                for contact in &manifold.points {
                    // Contact point in world frame: lift local_p (link
                    // frame) by link_pose, or (other frame) by other.
                    let local_p = if link_is_first {
                        contact.local_p1
                    } else {
                        contact.local_p2
                    };
                    let point_world = link_pose * local_p;
                    let impulse_scalar = contact.data.impulse;
                    let impulse_world = normal_world * impulse_scalar;
                    out.push(ContactInfo {
                        link_slot,
                        point_world,
                        impulse_world,
                    });
                }
            }
        }
        out
    }

    /// Return the link slot for `(arm_id, body)` if `collider` belongs
    /// to one of arm `arm_id`'s links; `None` otherwise.
    fn arm_link_slot_for(
        &self,
        arm_id: u32,
        collider: rapier3d::geometry::ColliderHandle,
    ) -> Option<u32> {
        let body_handle = self.collider_set.get(collider)?.parent()?;
        // Reverse-lookup the (arm_id, slot) by scanning the BTreeMap.
        // BTreeMap scan is O(n) in the number of arm-link bodies, which
        // is small (~3-5 per arm).
        for ((aid, slot), h) in &self.arm_link_bodies {
            if *aid == arm_id && *h == body_handle {
                return Some(*slot);
            }
        }
        None
    }
}

/// Per-contact information surfaced by `arm_link_external_contacts`.
#[derive(Clone, Debug)]
pub struct ContactInfo {
    pub link_slot: u32,
    pub point_world: nalgebra::Point3<f32>,
    pub impulse_world: nalgebra::Vector3<f32>,
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

    #[test]
    fn debug_render_empty_world_returns_no_lines() {
        let pw = PhysicsWorld::new(true);
        let lines = pw.debug_render();
        assert!(
            lines.is_empty(),
            "empty world should produce no debug lines, got {}",
            lines.len()
        );
    }

    #[test]
    fn debug_render_returns_lines_for_collider_outlines() {
        let mut pw = PhysicsWorld::new(true);
        // Insert one sphere; Rapier's DebugRenderPipeline draws collider
        // outlines as a fan of line segments around the sphere — the exact
        // count varies by tessellation, but it must be non-zero.
        let obj = Object::new(
            ObjectId(1),
            Isometry3::translation(0.0, 0.0, 0.5),
            Shape::Sphere { radius: 0.1 },
            0.1,
            true,
        );
        pw.insert_object(&obj);
        let lines = pw.debug_render();
        assert!(
            !lines.is_empty(),
            "sphere collider should produce at least one debug line"
        );
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
    fn object_friction_is_set_on_collider() {
        // Phase 3.3: Object.friction propagates into the Rapier collider.
        let mut pw = PhysicsWorld::new(true);
        let mut obj = sphere_object(8, 0.0);
        obj.friction = 0.7;
        let h = pw.insert_object(&obj);
        let f = pw.collider_friction(h).expect("collider exists");
        assert!(
            (f - 0.7).abs() < 1e-6,
            "expected collider friction 0.7, got {f}"
        );
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

    #[test]
    fn step_with_no_bodies_is_noop() {
        let mut pw = PhysicsWorld::new(true);
        for _ in 0..100 {
            pw.step(0.001);
        }
        assert_eq!(pw.body_count(), 0);
    }

    #[test]
    fn dynamic_object_falls_under_gravity_until_resting_on_fixture() {
        // Sphere object at z=1.0, fixture box top at z=0.5. Step long
        // enough for gravity to bring the sphere down + Rapier to
        // resolve contact.
        let mut pw = PhysicsWorld::new(true);
        let mut scene = crate::scene::Scene::new(0);
        // Fixture: large flat box, top at z=0.5.
        let fix = Fixture {
            id: 0,
            pose: Isometry3::translation(0.0, 0.0, 0.45),
            shape: Shape::Aabb {
                half_extents: nalgebra::Vector3::new(2.0, 2.0, 0.05),
            },
            is_support: true,
        };
        scene.add_fixture(fix.clone());
        pw.insert_fixture(&fix);
        // Sphere object at z=1.0.
        let obj = sphere_object(1, 0.0);
        scene.insert_object(obj.clone());
        pw.insert_object(&obj);

        for _ in 0..2000 {
            pw.step(0.001);
        }
        pw.sync_to_scene(&mut scene);

        let final_z = scene.object(ObjectId(1)).unwrap().pose.translation.z;
        // Top of fixture is z=0.5; sphere radius 0.05 ⇒ resting center
        // at z≈0.55. Allow 5 mm slop for solver settling.
        assert!(
            (final_z - 0.55).abs() < 0.005,
            "expected sphere to settle near z=0.55, got z={final_z}"
        );
    }

    #[test]
    fn sync_updates_lin_vel_when_object_is_falling() {
        // No fixture — object free-falls under gravity. After a few
        // steps, lin_vel.z should be negative.
        let mut pw = PhysicsWorld::new(true);
        let mut scene = crate::scene::Scene::new(0);
        let obj = sphere_object(1, 0.0);
        scene.insert_object(obj.clone());
        pw.insert_object(&obj);

        for _ in 0..50 {
            pw.step(0.001);
        }
        pw.sync_to_scene(&mut scene);

        let v = scene.object(ObjectId(1)).unwrap().lin_vel;
        assert!(v.z < -0.1, "expected falling lin_vel.z < -0.1, got {}", v.z);
    }
}

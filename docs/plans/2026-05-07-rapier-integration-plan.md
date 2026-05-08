# Rapier Physics Integration — Implementation Plan

Date: 2026-05-07
Status: Draft (companion to `2026-05-07-rapier-integration-design.md`)

## Conventions

- TDD per step (failing test first, then implementation).
- One commit per numbered step, in order. Each commit must keep `cargo test-all` green and `cargo clippy --workspace --all-targets --features viz-rerun -- -D warnings` clean — except for steps explicitly marked **(intermediate)**, where breakage is expected and gets resolved in a follow-up step within the same phase.
- Final V-gate sweep at end of each phase: `cargo test-all` twice (determinism), clippy, fmt, doc.
- For Phase 1 step "headline e2e" (Step 1.12), run the spec-compliance reviewer pattern. Steps with substantive design impact get the code-quality reviewer too. Routine mechanical steps (deps, struct skeletons, mechanical extractions) can rely on inline self-review.
- Workspace flag for the new physics: `physics-rapier` feature on `rtf_sim`, defaulted on once Phase 1 lands. During Phase 1 development, the kinematic gravity_step path is preserved behind a `#[cfg(not(feature = "physics-rapier"))]` gate so intermediate commits don't break unrelated tests. Final Phase 1 step removes the gate (and the kinematic code path) cleanly.

---

## Phase 1 — Rapier basics + joint torque sensor

13 commits.

### Step 1.1 — `[workspace] Add rapier3d dependency + sim::physics module skeleton`

**Files**: `Cargo.toml` (workspace), `sim/Cargo.toml`, `sim/src/physics/mod.rs` (new), `sim/src/lib.rs`.

Add `rapier3d = { version = "0.21", default-features = false, features = ["serde-serialize", "f32"] }` to `[workspace.dependencies]` and to `rtf_sim`'s deps. Critically: do NOT enable `parallel` (determinism per design §9). Add a `physics-rapier` feature on `rtf_sim`:
```toml
[features]
default = []
physics-rapier = ["dep:rapier3d"]
```
Move `rapier3d` to `dep:`-prefix in `[dependencies]` so the feature actually gates it.

Create `sim/src/physics/mod.rs` with just a module-level doc comment. Add `pub mod physics;` to `sim/src/lib.rs` (ungated — empty module is harmless; submodules will be feature-gated).

Failing test (in `sim/src/physics/mod.rs`):
```rust
#[cfg(all(test, feature = "physics-rapier"))]
mod tests {
    #[test]
    fn rapier_dependency_compiles() {
        let _g = rapier3d::math::Vector::new(0.0, 0.0, -9.81);
    }
}
```

Verify `cargo build -p rtf_sim --features physics-rapier` and `cargo build -p rtf_sim` (without feature) both succeed. No behavior change in any existing crate.

### Step 1.2 — `[sim] PhysicsWorld struct + new() constructor`

**Files**: `sim/src/physics/world.rs` (new), `sim/src/physics/mod.rs`.

Define the struct from design §3:
```rust
#[cfg(feature = "physics-rapier")]
pub struct PhysicsWorld {
    rigid_body_set: rapier3d::dynamics::RigidBodySet,
    collider_set: rapier3d::geometry::ColliderSet,
    island_manager: rapier3d::dynamics::IslandManager,
    broad_phase: rapier3d::geometry::BroadPhase,
    narrow_phase: rapier3d::geometry::NarrowPhase,
    impulse_joint_set: rapier3d::dynamics::ImpulseJointSet,
    multibody_joint_set: rapier3d::dynamics::MultibodyJointSet,
    ccd_solver: rapier3d::dynamics::CCDSolver,
    physics_pipeline: rapier3d::pipeline::PhysicsPipeline,
    integration_parameters: rapier3d::dynamics::IntegrationParameters,
    gravity: nalgebra::Vector3<f32>,

    object_bodies: std::collections::BTreeMap<crate::object::ObjectId, rapier3d::dynamics::RigidBodyHandle>,
    fixture_bodies: std::collections::BTreeMap<u32, rapier3d::dynamics::RigidBodyHandle>,
    arm_link_bodies: std::collections::BTreeMap<(u32, u32), rapier3d::dynamics::RigidBodyHandle>,
}
```

`new(gravity_enabled: bool) -> Self` constructor — initializes empty sets, sets gravity vector to `(0, 0, -9.81)` or zeros based on flag, configures `IntegrationParameters::default()` and overrides `dt = 0.001` (1 ms — matches our existing controller tick cadence; will be passed per `step` call too).

Tests:
- `physics_world_constructs_with_gravity` — call `new(true)`, assert gravity is `(0, 0, -9.81)`.
- `physics_world_constructs_without_gravity` — call `new(false)`, assert gravity is zeros.
- `physics_world_starts_with_no_bodies` — counts on body sets are zero.

Re-export `pub mod world;` from `physics/mod.rs` and `pub use world::PhysicsWorld;` from `sim/src/lib.rs` (gated behind `#[cfg(feature = "physics-rapier")]`).

### Step 1.3 — `[sim] PhysicsWorld body construction from Object/Fixture/ArmLink`

**Files**: `sim/src/physics/world.rs`.

Add three methods:
```rust
impl PhysicsWorld {
    pub fn insert_object(&mut self, obj: &Object) -> RigidBodyHandle;
    pub fn insert_fixture(&mut self, fix: &Fixture) -> RigidBodyHandle;
    pub fn insert_arm_link(&mut self, arm_id: u32, slot: u32, pose: Isometry3<f32>,
                           shape: ArmLinkShape) -> RigidBodyHandle;
}
```

Internal helpers:
- `shape_to_collider(shape: &Shape) -> ColliderBuilder` — maps `Shape::Aabb { half_extents }` → `cuboid(hx, hy, hz)`, `Shape::Sphere { radius }` → `ball(radius)`, `Shape::Cylinder { radius, half_height }` → `capsule_z(half_height, radius)` (closest match — cylinder doesn't exist as a primitive in parry; capsule is fine for v1).
- Object body type: `RigidBodyType::Dynamic`, mass derived from `obj.mass`.
- Fixture body type: `RigidBodyType::Fixed`.
- Arm link body type: `RigidBodyType::KinematicPositionBased`. Capsule collider sized to match `Arm::append_primitives` (radius = `LINK_RADIUS = 0.02`, half_height = `link_vec.norm() / 2.0`).

`ArmLinkShape` is a small struct exposing `radius` and `half_height` so the caller doesn't need to know about `Shape` — keeps the arm-link case clean.

Tests:
- `insert_object_creates_dynamic_body_at_pose` — insert a sphere object, look up its body, assert position matches and type is Dynamic.
- `insert_fixture_creates_fixed_body` — same for fixture.
- `insert_arm_link_creates_kinematic_position_based_body` — assert body type.
- `insert_object_collider_matches_aabb_extents` — insert an Aabb object, assert collider half-extents match.
- `insert_records_handle_in_lookup_map` — insert object, assert `object_bodies.get(&id).is_some()`.

### Step 1.4 — `[sim] PhysicsWorld::step + sync_to_scene`

**Files**: `sim/src/physics/world.rs`.

```rust
impl PhysicsWorld {
    pub fn step(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;
        self.physics_pipeline.step(
            &self.gravity, &self.integration_parameters,
            &mut self.island_manager, &mut self.broad_phase, &mut self.narrow_phase,
            &mut self.rigid_body_set, &mut self.collider_set,
            &mut self.impulse_joint_set, &mut self.multibody_joint_set,
            &mut self.ccd_solver, None, &(), &(),
        );
    }

    pub fn sync_to_scene(&self, scene: &mut Scene) {
        for (obj_id, body_handle) in &self.object_bodies {
            let body = self.rigid_body_set.get(*body_handle).unwrap();
            if let Some(obj) = scene.object_mut(*obj_id) {
                obj.pose = *body.position();
                obj.lin_vel = *body.linvel();
            }
        }
    }
}
```

Tests:
- `dynamic_object_falls_under_gravity` — insert a sphere at `(0, 0, 1.0)` above an Aabb fixture at `(0, 0, 0)`, step 1000 times at dt=1ms, sync, assert `obj.pose.translation.z` is close to top-of-fixture (settles).
- `step_with_no_bodies_is_noop` — empty world, step 100 times, assert no panic.
- `sync_updates_lin_vel` — insert object, step a few times, assert `obj.lin_vel.z < 0` (falling).

### Step 1.5 — `[arm] ArmWorld owns PhysicsWorld; insert all entities at construction`

**Files**: `arm/src/world.rs`, `arm/Cargo.toml`.

Add `physics-rapier` feature to `arm/Cargo.toml`:
```toml
physics-rapier = ["rtf_sim/physics-rapier"]
```

Add field to `ArmWorld`:
```rust
#[cfg(feature = "physics-rapier")]
physics: rtf_sim::physics::PhysicsWorld,
```

In `ArmWorld::new(scene, spec, gravity_enabled)`:
- Construct `PhysicsWorld::new(gravity_enabled)` (gated by feature).
- Iterate `scene.fixtures()` — call `physics.insert_fixture(...)` for each.
- Iterate `scene.objects()` — call `physics.insert_object(...)` for each.
- Iterate arm links — call `physics.insert_arm_link(arm_id=0, slot=i, pose=initial_link_pose_from_fk, shape=...)` for each.

Tests:
- `armworld_with_physics_feature_owns_physics_world_with_all_bodies` — build `build_pick_and_place_world`, assert physics has expected number of bodies (1 object + 3 fixtures + 3 arm links = 7).
- `armworld_without_physics_feature_compiles_unchanged` — basic construction works without the feature flag.

### Step 1.6 — `[arm] Use Rapier step in consume_actuators_and_integrate (intermediate, gated by feature)`

**(intermediate)** — kinematic gravity-fall path is preserved behind `#[cfg(not(feature = "physics-rapier"))]` so default-feature `cargo test-all` stays green.

**Files**: `arm/src/world.rs`.

```rust
fn consume_actuators_and_integrate_inner(&mut self, dt: Duration) {
    // ... existing command draining + gripper logic ...

    #[cfg(feature = "physics-rapier")]
    {
        // Update each arm link's kinematic body pose from current FK
        for (i, link_pose) in self.fk_link_poses().enumerate() {
            self.physics.set_arm_link_pose(0, i as u32, link_pose);
        }
        let dt_f = (dt.as_nanos() as f32) / 1.0e9;
        self.physics.step(dt_f);
        self.physics.sync_to_scene(&mut self.scene);
    }

    #[cfg(not(feature = "physics-rapier"))]
    {
        // existing gravity_step + manual settled-handling path
    }

    // Grasped object weld stays as-is (works both paths)
    if let Some(grasped_id) = self.arm.state.grasped { ... }

    // sim_time advance
}
```

Add `PhysicsWorld::set_arm_link_pose(arm_id, slot, pose)` — looks up the kinematic body and calls `body.set_next_kinematic_position(pose)`.

Add `ArmWorld::fk_link_poses()` helper — returns iterator over FK-derived link midpoint poses (lifted from current `Arm::append_primitives` logic).

Tests with feature on:
- `armworld_step_drops_object_into_table` — build_pick_and_place_world, run for 100ms, assert the block hasn't moved (it's already Settled on the table per the scene fixture).
- `armworld_step_pushes_object_when_arm_collides` — place a small free object directly in the arm's path, drive joint velocities to sweep through it, assert object xy is displaced.

Tests without feature: existing tests pass unchanged (no behavior change).

### Step 1.7 — `[arm] Settled detection from Rapier velocity / sleep state`

**Files**: `arm/src/world.rs`.

After `physics.sync_to_scene`, walk all dynamic objects and update their `state` based on velocity:
```rust
const SETTLED_VELOCITY_THRESHOLD: f32 = 1e-3;

for (id, obj) in self.scene.objects_mut() {
    if matches!(obj.state, ObjectState::Grasped { .. }) { continue; }
    let resting = obj.lin_vel.norm() < SETTLED_VELOCITY_THRESHOLD;
    obj.state = if resting {
        ObjectState::Settled { on: SupportId::Unknown }
    } else {
        ObjectState::Free
    };
}
```

Add `SupportId::Unknown` variant — used when Rapier-driven settle doesn't track support relationships.

Tests:
- `object_transitions_to_settled_when_velocity_below_threshold` — manipulate Rapier body velocity to ~0, step, sync, assert state is Settled.
- `object_remains_free_when_moving` — velocity > threshold → Free.
- `grasped_object_state_unchanged_by_settled_logic` — set Grasped, step, assert still Grasped.

Update existing goal evaluations (`PlaceInBin`, `NObjectsInBin`) that match on `Settled { on: SupportId::Fixture(bin_id) }` — switch to checking pose-vs-bin-footprint directly (per design §6). Should be a small migration.

### Step 1.8 — `[arm] Grasp via kinematic body type switch`

**Files**: `arm/src/world.rs`, `sim/src/physics/world.rs`.

Add `PhysicsWorld::set_object_kinematic(id, pose)` and `set_object_dynamic(id)` — flip the body type:
```rust
pub fn set_object_kinematic(&mut self, id: ObjectId, pose: Isometry3<f32>) {
    let handle = self.object_bodies[&id];
    let body = self.rigid_body_set.get_mut(handle).unwrap();
    body.set_body_type(RigidBodyType::KinematicPositionBased, true);
    body.set_next_kinematic_position(pose);
}

pub fn set_object_dynamic(&mut self, id: ObjectId) {
    let handle = self.object_bodies[&id];
    let body = self.rigid_body_set.get_mut(handle).unwrap();
    body.set_body_type(RigidBodyType::Dynamic, true);
    body.set_linvel(Vector3::zeros(), true);
    body.set_angvel(Vector3::zeros(), true);
}
```

In `apply_gripper_command`:
- On grasp transition: call `physics.set_object_kinematic(id, ee_pose)`.
- On release transition: call `physics.set_object_dynamic(id)`.

In the per-tick loop, the existing "grasped object pose = EE" line now calls `physics.set_object_kinematic(id, ee_pose)` each tick (idempotent — body is already kinematic).

Tests:
- `grasp_switches_body_to_kinematic` — grasp object, assert body type.
- `release_switches_body_back_to_dynamic` — grasp then release, assert dynamic.
- `grasped_object_does_not_fall_under_gravity` — grasp object, step 100 times with gravity on, assert pose doesn't change.

### Step 1.9 — `[arm] schedule_spawn creates Rapier body at spawn time`

**Files**: `arm/src/world.rs`.

In the `pending_spawns` drain loop, after `scene.insert_object(obj)`:
```rust
#[cfg(feature = "physics-rapier")]
self.physics.insert_object(&obj);
```

Tests:
- `scheduled_spawn_creates_rapier_body` — schedule spawn, step past its time, assert the object's body is in physics.
- `spawned_object_falls_when_above_ground` — schedule a spawn at z=1.0 above empty scene, step for 1s, assert object lands on ground.

### Step 1.10 — `[viz] Rotated Box rendering in RerunRecorder`

**Files**: `viz/src/rerun_recorder.rs`.

Today the `Primitive::Box` branch uses `Boxes3D::from_centers_and_half_sizes` — centers only. Replace with the variant that accepts rotation (rerun 0.21: `with_rotations` or `with_quaternions`, depending on what the API exposes; check what's available):
```rust
let center = [pose.translation.x, pose.translation.y, pose.translation.z];
let half_size = [half_extents.x, half_extents.y, half_extents.z];
let q = pose.rotation.coords;  // [x, y, z, w]
let _ = self.stream.log(
    path.as_str(),
    &rerun::archetypes::Boxes3D::from_centers_and_half_sizes([center], [half_size])
        .with_quaternions([rerun::components::RotationQuat::from_xyzw([q.x, q.y, q.z, q.w])]),
);
```

(Adjust to actual rerun 0.21 API — may require `RotationQuat` or `Rotation3D`.)

Tests:
- `recorder_passes_rotation_to_rerun_box_archetype` — log a Primitive::Box with non-identity rotation, assert no panic + (optionally) parse the .rrd back and verify rotation.

### Step 1.11 — `[arm] JointTorqueReading port + attach_joint_torque_sensor`

**Files**: `arm/src/ports.rs`, `arm/src/world.rs`, `sim/src/physics/world.rs`.

In `arm/src/ports.rs`:
```rust
#[derive(Clone, Debug)]
pub struct JointTorqueReading {
    pub joint: JointId,
    pub tau: f32,
    pub sampled_at: Time,
}

impl SensorReading for JointTorqueReading {
    fn sampled_at(&self) -> Time { self.sampled_at }
}
```

In `sim/src/physics/world.rs`, add:
```rust
pub fn arm_link_external_contacts(&self, arm_id: u32)
    -> impl Iterator<Item = ContactInfo> + '_;

pub struct ContactInfo {
    pub link_slot: u32,
    pub point_world: Point3<f32>,  // in world frame
    pub impulse_world: Vector3<f32>, // total contact impulse over the last step
}
```

Implementation walks `narrow_phase.contact_pairs()`, filters to pairs where one collider belongs to an arm link of `arm_id`, sums impulses across manifold points.

In `arm/src/world.rs`:
- Track attached torque sensors as `Vec<(JointId, PortTx<JointTorqueReading>, RateHz, Time /* last_published */)>`.
- Add `attach_joint_torque_sensor(joint, rate) -> PortRx<JointTorqueReading>`.
- In `publish_sensors_for_dt`, for each torque sensor:
  - Compute joint axis `a_i` and anchor `p_i` in world frame from FK.
  - Sum over `arm_link_external_contacts` for downstream links: `tau_i += a_i · ((p − p_i) × (impulse / dt))`.
  - Send the reading.

Tests:
- `joint_torque_zero_when_no_contact` — no objects in scene, drive arm motion, assert all torques are 0.
- `joint_torque_nonzero_when_link_contacts_object` — place a fixed object directly under a moving arm link, drive into contact, assert torque on that joint is non-zero.
- `joint_torque_signed_per_axis_convention` — push the link in opposite directions, assert torque sign flips.
- `joint_torque_publishes_at_configured_rate` — attach at 100 Hz, step world for 50 ms, assert exactly 5 readings sent.

### Step 1.12 — `[arm] Headline e2e: examples/find_by_touch.rs`

**Files**: `arm/examples/find_by_touch.rs` (new), `arm/Cargo.toml`.

New example. The arm sweeps the search region (same as find_grasp_place geometry), but **instead of pressure-sensor peak-tracking**, it terminates the sweep state the moment any joint torque exceeds a threshold (2 N·m default). On contact, descent + grasp + ascend + place follows the same pattern.

Controller `FindByTouch` defined inline. Three #[test]s for seeds 1, 42, 1337. Each asserts `Termination::GoalComplete` + score >= 0.9 within a 30 s deadline. Use `maybe_recorder_for("find_by_touch_seed_<n>")`.

Add `[[example]] name = "find_by_touch", required-features = ["examples", "physics-rapier"]` to `arm/Cargo.toml`.

This is the headline test that proves the joint torque sensor works end-to-end.

After this commit, run the spec-compliance reviewer on the new example to confirm requirements coverage.

### Step 1.13 — `[arm,sim] Drop kinematic gravity_step path; `physics-rapier` is default`

**Files**: `sim/Cargo.toml`, `sim/src/lib.rs`, `sim/src/gravity.rs` (delete), `arm/src/world.rs`, `arm/tests/release_falls.rs`.

- `sim/Cargo.toml`: change `default = []` to `default = ["physics-rapier"]`.
- Delete `sim/src/gravity.rs` and the `pub mod gravity;` line.
- In `arm/src/world.rs`: remove the `#[cfg(not(feature = "physics-rapier"))]` arm; the Rapier path is the only path.
- `arm/tests/release_falls.rs`: rewrite expectations. Object now bounces / rolls slightly before settling. Loosen position tolerance to ~5 cm; keep "object ends up at rest somewhere on the table" as the assertion shape.
- Re-tune any other tests that still assume kinematic gravity-fall (likely some unit tests in arm/src/world.rs's gravity sub-module — delete those that no longer apply).

Tests: full suite passes with default features.

### Phase 1 V-gate sweep

Run all four:
- V.1: `cargo test-all` twice — assert byte-identical scores. (Some scores may shift from Phase 1 baseline due to physics changes; what matters is determinism within the new baseline.)
- V.2: `cargo clippy --workspace --all-targets --features viz-rerun -- -D warnings` clean.
- V.3: `cargo fmt --all -- --check` clean.
- V.4: `cargo doc --workspace --no-deps` warning-free.

If any score regresses below acceptance threshold (e.g. find_grasp_place drops below 0.9 due to physics interactions during sweep), fix in a small follow-up commit (re-tune sweep parameters or pressure_threshold) before declaring Phase 1 done.

---

## Phase 2 — `RapierDebugVisualizable` overlay

3 commits.

### Step 2.1 — `[sim] Hook up Rapier DebugRenderPipeline → Vec<DebugLine>`

**Files**: `sim/src/physics/world.rs`, `sim/src/physics/debug_render.rs` (new).

```rust
pub struct DebugLine {
    pub from: Point3<f32>,
    pub to: Point3<f32>,
    pub color: [u8; 4],
}

impl PhysicsWorld {
    pub fn debug_render(&mut self) -> Vec<DebugLine> {
        let mut backend = LineCapture::default();
        let mut pipeline = DebugRenderPipeline::default();
        pipeline.render(
            &mut backend,
            &self.rigid_body_set, &self.collider_set,
            &self.impulse_joint_set, &self.multibody_joint_set,
            &self.narrow_phase,
        );
        backend.lines
    }
}

#[derive(Default)]
struct LineCapture { lines: Vec<DebugLine> }
impl DebugRenderBackend for LineCapture {
    fn draw_line(&mut self, _obj: DebugRenderObject, a: Point<f32>, b: Point<f32>, color: [f32; 4]) {
        self.lines.push(DebugLine {
            from: Point3::from(a.coords),
            to: Point3::from(b.coords),
            color: [(color[0] * 255.0) as u8, (color[1] * 255.0) as u8, (color[2] * 255.0) as u8, (color[3] * 255.0) as u8],
        });
    }
}
```

Tests:
- `debug_render_returns_lines_for_collider_outlines` — insert a sphere object, call `debug_render`, assert `lines.len() > 0`.
- `debug_render_empty_world_returns_no_lines` — empty world → empty Vec.

### Step 2.2 — `[arm,viz] Plumb debug overlay through SceneSnapshot`

**Files**: `arm/src/world.rs`, `sim/src/primitive.rs`.

Add a flag `debug_overlay: bool` on `ArmWorld` (default false). Add a setter `enable_debug_overlay(&mut self, on: bool)`.

In `snapshot()`:
```rust
if self.debug_overlay {
    for line in self.physics.debug_render() {
        items.push((
            EntityId::DebugOverlay(items.len() as u32),
            Primitive::Line { from: line.from, to: line.to, color: Color::rgba(line.color[0], line.color[1], line.color[2], line.color[3]) },
        ));
    }
}
```

Add `EntityId::DebugOverlay(u32)` variant to `sim/src/entity.rs` (per the existing namespacing pattern from Step 4266c86).

Tests:
- `snapshot_includes_debug_overlay_lines_when_enabled` — enable overlay, call snapshot, assert Line primitives present.
- `snapshot_excludes_debug_overlay_when_disabled` — default-off, no Line primitives from physics.

### Step 2.3 — `[arm] Optional debug overlay in examples via env var or feature`

**Files**: each `arm/examples/*.rs`.

Add a `#[cfg(feature = "viz-rerun")]` block in each example's runner that checks an env var (`RTF_DEBUG_OVERLAY=1`) and calls `world.enable_debug_overlay(true)` accordingly. Documented in a top-of-file comment per example.

Optionally, add a new example `arm/examples/find_grasp_place_debug.rs` that's just `find_grasp_place` with overlay enabled by default — trivial wrapper, useful for "I want to see the colliders during this run."

Tests: existing example tests keep working with overlay off (default). Add one test per example that runs with overlay on and asserts no panic + at least some Line primitives appear in the recorder output (when viz-rerun is on).

### Phase 2 V-gate sweep

Same as Phase 1.

---

## Phase 3 — Friction-based grasping

7 commits.

### Step 3.1 — `[arm] Promote finger boxes to Rapier kinematic colliders`

**Files**: `arm/src/world.rs`, `sim/src/physics/world.rs`.

Add `PhysicsWorld::insert_finger(arm_id, slot, pose, half_extents)` — kinematic position-based body with cuboid collider. Set friction coefficient high (e.g. 1.0) on finger colliders to facilitate grip.

In `ArmWorld::new`, insert two finger bodies (slots 998, 999 — matching the viz convention). Track in `arm_link_bodies` (or a sibling `arm_finger_bodies` map).

Each tick (in `consume_actuators_and_integrate_inner`), after updating arm link poses, also update finger poses based on the current `gripper_separation` (new field on `ArmState`, see Step 3.2).

Tests:
- `fingers_have_kinematic_bodies` — build world, assert two finger bodies present.
- `finger_pose_tracks_ee_with_separation` — set ArmState gripper_separation, step, assert finger poses match expectation.

### Step 3.2 — `[arm] Continuous gripper actuation: GripperCommand.target_separation`

**Files**: `arm/src/ports.rs`, `arm/src/state.rs`, `arm/src/world.rs`.

Replace:
```rust
pub struct GripperCommand { pub close: bool }
```
with:
```rust
pub struct GripperCommand { pub target_separation: f32 }
```

Add `gripper_separation: f32` to `ArmState` (current finger position).

In `apply_gripper_command`:
- Drive `arm.state.gripper_separation` toward `cmd.target_separation` at a fixed close rate (e.g., 0.5 m/s — fingers snap closed/open in 80 ms).
- The grasp/release transitions are now derived from `gripper_separation` crossing a threshold (e.g., < 0.02 m = closed, > 0.035 m = open), matching today's binary semantics.

Update all four existing example controllers (`PickPlace`, `SearchAndPlace`, `ContinuousSearchAndPlace`, `find_by_touch`) to issue `target_separation = 0.012` (close) or `target_separation = 0.04` (open) instead of `close: bool`. Mechanical change, no behavior shift.

Tests:
- `gripper_separation_converges_to_target` — issue close command, step, assert separation decreases.
- `existing_grasp_threshold_semantics_preserved` — close until separation < 0.02, assert grasp transition fires.

### Step 3.3 — `[sim,arm] Configure friction coefficients on graspable objects`

**Files**: `sim/src/physics/world.rs`, `sim/src/object.rs` (maybe — see below).

Add `friction: f32` field to `Object` (default 0.5), or expose via `Shape` (cleaner: friction is a material property, lives on the Object level not the Shape level).

In `PhysicsWorld::insert_object`, set the collider's friction via `ColliderBuilder::friction(obj.friction)`.

Tests:
- `object_friction_is_set_on_collider` — insert object with friction=0.7, look up collider, assert friction matches.

### Step 3.4 — `[arm] Friction-grasp: Grasped state derived from finger contacts`

**Files**: `arm/src/world.rs`, `sim/src/physics/world.rs`.

Add `PhysicsWorld::object_in_contact_with_fingers(obj_id, arm_id) -> bool` — checks `narrow_phase.contact_pair(...)` between the object's collider and both finger colliders.

In `apply_gripper_command` and the per-tick post-step logic:
- **Remove the kinematic-body-switch grasp.** Objects stay Dynamic.
- Detect Grasped state per tick: `Grasped { by }` iff `gripper_separation < 0.02` AND the object is in contact with both fingers AND graspable.
- The object is held in place by friction at the finger contacts (Rapier solves this).

Tests:
- `object_grasped_when_fingers_close_around_it` — place small block between fingers, set target_separation to closed, step, assert object state transitions to Grasped.
- `object_falls_out_of_grip_under_high_acceleration` — grasp object, then drive arm at a velocity step that exceeds friction-supported acceleration, assert object slips (Grasped → Free transition).

This is the central change. Expect existing example tests to break here — controllers that issue `target_separation = 0.012` and expect immediate weld will need re-tuning (e.g., add a delay between close-command and ascend, slow ascend velocity so friction holds).

### Step 3.4.5 — `[arm] Add wrist joint (J3) + 3R IK helper`

Inserted post-implementation. Step 3.4's friction-grasp surfaced that the 3-joint Z-Y-Y arm physically cannot orient EE +z straight down at non-trivial reach (cumulative J1+J2 pitch ~58° at the canonical grasp pose; needs ~90° for fingers to actually envelop the block). Pre-existing geometry debt that the kinematic-weld grasp masked. Treated as an interlude inside Phase 3.

Three commits. Each remains "(intermediate)" — example tests stay broken until Step 3.5 retunes friction.

#### 3.4.5a — `[arm] 3R planar IK helper`

**File**: `arm/src/ik.rs` (extend).

Add:
```rust
/// 3R planar IK in the shoulder-local (radial, z) plane with explicit
/// EE pitch target. Returns (J1, J2, J3) where:
/// - EE position (after the J3 wrist link of length l3) is at (target_x, target_z)
/// - EE +x direction has world pitch `target_pitch` (e.g., -π/2 for EE pointing world -z)
pub fn ik_3r(target_x: f32, target_z: f32, target_pitch: f32,
             l1: f32, l2: f32, l3: f32) -> Option<(f32, f32, f32)>;
```

Math: compute wrist-anchor target by subtracting `l3 * (cos(target_pitch), -sin(target_pitch))` from the EE target. Run `ik_2r` on the wrist anchor for J1, J2. Then `J3 = target_pitch - (J1 + J2)`.

Tests: boundary cases (level wrist, wrist pointing down, wrist pointing up), FK round-trip via `forward_kinematics` on a 4-joint spec, unreachable cases.

Commit `[arm] Step 3.4.5a: 3R planar IK helper`.

#### 3.4.5b — `[arm,sim] Extend arm spec to 4-joint with wrist link`

**Files**: `arm/src/test_helpers.rs` (extend), `arm/examples/find_by_touch.rs` (find_by_touch has its own world builder inline), and any other scenario world builders that have hand-rolled `ArmSpec`s.

Change each ArmSpec:
- Add J3: `JointSpec::Revolute { axis: Vector3::y_axis(), limits: (-PI, PI) }`
- Add wrist link offset: `Isometry3::translation(0.05, 0.0, 0.0)` (5 cm wrist)
- Update any link-length constants used by controllers (`L1`, `L2`, `L3 = 0.05`, etc.)

Tests: existing FK / arm-construction tests still pass with J3 = 0 (verify EE position with J3 = 0 matches old position plus the 5 cm wrist offset along the chain's terminal direction).

Commit `[arm,sim] Step 3.4.5b: 4-joint arm spec with wrist (intermediate)`.

#### 3.4.5c — `[arm] Controllers drive J3 to keep EE pointing down`

**Files**: `arm/examples/pick_place.rs`, `arm/examples/find_grasp_place.rs`, `arm/examples/continuous_spawn.rs`, `arm/examples/find_by_touch.rs`.

Each controller's pre-computed IK calls swap `ik_2r(...)` → `ik_3r(..., target_pitch = -PI/2.0)` (EE +x rotates to world -z, fingers point down). Constructor signatures gain `l3: f32` param. Sweep poses can use the same `target_pitch` (EE pointing down throughout the sweep).

Joint convergence checks now compare 4 joints instead of 3.

Example tests still failing at end of this commit (friction tuning is Step 3.5).

Commit `[arm] Step 3.4.5c: Controllers drive J3 via 3R IK (intermediate)`.

---

### Step 3.5 — `[arm,sim] Joint-attached grasp (FixedJoint) + slip-impulse detection + controller re-tuning`

**Pivot from original plan.** Original framing: "tune friction parameters." After 12 iterations the friction-grasp approach hit a Rapier kinematic↔dynamic friction-coupling limitation; best e2e score was 0.788 vs 0.9 acceptance bar. See updated design doc §11.3 for full rationale.

**New approach:** on grasp transition, insert a Rapier `FixedJoint` between the EE kinematic body and the object's dynamic body, anchored at the current relative offset. Object stays Dynamic. Per tick, query the joint's accumulated impulse magnitude; if > slip threshold, remove the joint and transition `Grasped → Free`. On gripper open, remove the joint explicitly.

This is industry-standard practice (Isaac Sim, NVIDIA PhysX use this pattern). Honest framing: it's a soft-weld with slip-as-impulse-threshold, not pure-friction grip. See design doc §11.3 for the full disclosure.

**Files**: `sim/src/physics/world.rs` (add joint management + slip query), `arm/src/world.rs` (call into joint API on grasp/release transitions; per-tick slip check), `arm/examples/*.rs` (4 controllers — re-tune timing, optionally react to slip events).

**Concrete sub-tasks (all in one commit):**

1. `PhysicsWorld::insert_grasp_joint(arm_id, ee_body_handle, object_id) -> ImpulseJointHandle` — creates a `FixedJoint` between the EE arm-link kinematic body and the object's dynamic body, anchored at the current relative pose. Stores the handle keyed on `object_id` in a new `BTreeMap<ObjectId, ImpulseJointHandle>`.
2. `PhysicsWorld::remove_grasp_joint(object_id)` — removes from `ImpulseJointSet` + the lookup map.
3. `PhysicsWorld::grasp_joint_impulse(object_id) -> f32` — sums the joint's `impulses` (linear + angular magnitudes) over the most recent step.
4. In `arm/src/world.rs`: on grasp transition, call `physics.insert_grasp_joint(...)`. On release transition (gripper open), call `physics.remove_grasp_joint(...)`. Per tick after `physics.step`, for any currently-grasped object: check `grasp_joint_impulse > SLIP_THRESHOLD` (default 5.0 N·s, tunable); if exceeded, remove joint + set state to `Free`.
5. Discard prior friction-tuning state in `apply_gripper_command` and per-tick `derive_friction_grasp_state` (those were the failed-approach scaffolding). Keep finger-contact detection logic only as the *trigger* for grasp transition — the actual coupling is now the joint, not the contact.
6. Re-tune controller examples for the joint-grasp model:
   - `close_hold_ticks`: drop back to 200 (joint forms instantly; long hold no longer helpful)
   - Ascend velocity clamp can be removed (joint provides rigid attachment; no slip risk during slow translation)
   - "Still grasped" verification reads `arm.state.grasped.is_some()` after AscendWithBlock; if it became None (slip during ascend or yaw), abort and restart sweep (matches existing slip-handling in continuous_spawn).
7. Verify all four example seed-test suites pass within their acceptance bars + deadlines.

Tests:
- `joint_grasp_inserts_fixed_joint_on_grasp_transition` — close gripper near object, assert `physics.grasp_joint_impulse(id).is_some()`.
- `joint_grasp_removes_joint_on_release` — open gripper, assert no joint.
- `joint_grasp_releases_under_high_impulse` — set up a held object, apply forces that exceed slip threshold, assert state transitions to Free + joint removed.
- e2e: pick_place, find_grasp_place (3 seeds), continuous_spawn (3 seeds), find_by_touch (3 seeds) all pass.

Commit message: `[arm,sim] Step 3.5: Joint-attached grasp (FixedJoint) with slip-impulse detection; controller re-tune`. Include in commit body: brief disclosure that this pivots from pure-friction (12 tuning iterations failed) to joint-based per design §11.3.

### Step 3.5-original — (REMOVED) `[arm] Re-tune existing example controllers for friction grasp`

**Files**: `arm/examples/pick_place.rs`, `arm/examples/find_grasp_place.rs`, `arm/examples/continuous_spawn.rs`, `arm/examples/find_by_touch.rs`.

Per controller:
- Bump `close_hold_ticks` to give Rapier time to settle the friction grip after fingers close (try 500 = 0.5 s).
- Cap ascend joint velocity to a slow value (e.g. clamp to ±0.5 rad/s during AscendWithBlock state).
- Add an explicit "verify still grasped" check after AscendWithBlock — if the controller transitions away from Grasped state (slip), abort the cycle.

Tests: each example's seed tests pass within their existing deadlines.

### Step 3.6 — `[arm] examples/grasp_robustness.rs — explicit slip-detection test`

**Files**: `arm/examples/grasp_robustness.rs` (new), `arm/Cargo.toml`.

New example demonstrating:
- A "good" run: controller closes gripper, ascends slowly, places successfully.
- A "bad" run: controller closes gripper, then tries to ascend too fast — object slips, never reaches the bin, score = 0.

Two #[test]s asserting the expected outcomes. This is the test that proves Phase 3 actually changed something — without friction-based grasp, both behaviors would succeed (kinematic weld doesn't slip).

### Step 3.7 — `[arm] cargo doc updates + design-doc Phase 3 section finalization`

**Files**: `docs/plans/2026-05-07-rapier-integration-design.md`, various `arm/src/*.rs` doc comments.

- Update `Noise impl for GripperCommand` section in design doc § 11.3 if anything diverged.
- Doc-comment the friction-grasp model in `apply_gripper_command`.
- Add a brief mention in the top-level arm/src/lib.rs doc-comment that grasp is friction-based (Phase 3+).

### Phase 3 V-gate sweep

Same as Phase 1.

---

## Final V-gate (after all three phases land)

Re-run V.1–V.4 from the v1 implementation plan. All 134+ tests pass. Headline scores byte-identical across two release runs. Document any score deltas vs pre-Rapier in a brief commit message in the final commit.

## Out of scope (deferred to later)

- Soft body / fluid sim
- Joint torque control (still velocity-controlled)
- Multi-arm scenes
- Articulated body modeling for the arm via Rapier's multibody joints
- CCD for fast-moving objects
- Static-vs-kinetic friction distinction
- Per-finger pressure sensors (a logical pair with friction grasping but not strictly required)

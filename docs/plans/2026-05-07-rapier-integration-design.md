# Rapier Physics Integration — Design

Date: 2026-05-07
Status: Draft (extends `2026-05-05-robotics-test-framework-design-v2.md`)

## 1. Scope and motivation

V1 ships a kinematic-only sim: arm links phase through scene geometry, objects fall straight down via a hand-rolled `gravity_step`, contact has no consequence beyond the gripper's binary grasp threshold. This was an explicit v1 trade-off (design v2 §3) — pragmatic for the open-loop and proximity-search scenarios we've shipped, but blocks every test that depends on actual physical interaction (sweep-and-push, friction-grasp robustness, contact-driven exploration).

The user-driving motivation is to enable a **contact-feedback search** scenario: the arm should sweep around and *feel resistance* through its joint sensors when it bumps into an object — turning physical contact into a controller-observable signal. The Rapier integration is the substrate for that capability.

This extension swaps in **Rapier** as the physics engine for free objects + arm-vs-object contact, and is structured in three phases:

1. **Rapier basics + joint torque sensor** — integrate Rapier as the engine for object dynamics + arm-vs-object contact, AND expose per-joint contact torque as a new `PortReader<JointTorqueReading>` channel so controllers can sense pushback. Existing scenarios continue to work (with re-tuning where physics changes behavior). Magic-weld grasp preserved.
2. **`RapierDebugVisualizable` wrapper** — opt-in debug overlay rendering Rapier's view of the world (collider outlines, contact points, broadphase AABBs) into our existing rerun pipeline. Useful for debugging physics behavior; not load-bearing for demos.
3. **Friction-based grasping** — replace the magic-weld grasp with real physics: finger colliders, friction-based grip, slip detection.

The user explicitly OK'd partial coupling to a third-party library here — the rule is "don't let Rapier leak into every part of the codebase," not "wrap Rapier exhaustively behind local traits the way `NoiseSource` was." Rapier types appear inside `rtf_sim::physics` only; they don't appear in `rtf_core`, `rtf_viz`, controllers, or domain types like `Object`/`Fixture`.

## 2. Crate layout — where Rapier lives

```
rtf_core    — never sees Rapier (pure abstractions)
rtf_sim     — Rapier IS HERE. New module `rtf_sim::physics` wraps it.
              `gravity.rs` is replaced by Rapier's gravity.
rtf_arm     — never imports rapier3d directly; uses rtf_sim::physics
              indirectly via ArmWorld's internal state.
rtf_harness — never sees Rapier.
rtf_viz     — never sees Rapier directly. Gets primitives via SceneSnapshot
              as today. Phase 2 adds an opt-in `Primitive::Line` overlay
              fed from rtf_sim::physics::debug_render().
```

`rapier3d = "0.21"` (or whichever current version) added to `[workspace.dependencies]` and pulled in by `rtf_sim/Cargo.toml`.

The `rapier3d` crate transitively brings in `parry3d`, `nalgebra` (we already use), and a handful of small deps. Acceptable cost.

## 3. Body / collider model

Every domain entity gets a Rapier representation. The mapping is owned by `PhysicsWorld`, not by the domain types themselves — `Object`/`Fixture`/`Arm` stay free of Rapier handles.

| Domain entity         | Rapier body type        | Notes |
|-----------------------|-------------------------|-------|
| `Object` (Free)       | `RigidBodyType::Dynamic`         | Full physics. Affected by gravity, contact, friction. |
| `Object` (Settled)    | `RigidBodyType::Dynamic` (asleep) | Settled is a *derived* status — see §6. Body type doesn't change. |
| `Object` (Grasped)    | `RigidBodyType::KinematicPositionBased` | Phase 1: pose-welded to EE each tick. Phase 3: stays Dynamic, held by friction (state's meaning shifts — see §11.3). |
| `Fixture`             | `RigidBodyType::Fixed`            | Immovable. Static collider. Includes the ground plane. |
| Arm link (capsule)    | `RigidBodyType::KinematicPositionBased` | Pose updated from FK each tick. Pushes dynamic objects on contact, never pushed back. |
| Gripper finger boxes  | (Phase 1) viz-only — no collider. (Phase 3) `KinematicPositionBased` colliders. | Phase 3 promotion enables friction grasping. |

Mapping ownership in `PhysicsWorld`:
```rust
struct PhysicsWorld {
    // Rapier state
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    physics_pipeline: PhysicsPipeline,
    integration_parameters: IntegrationParameters,
    gravity: Vector3<f32>,

    // Domain ↔ Rapier mapping (BTreeMap per design v2 §10.2 — no HashMap)
    object_bodies: BTreeMap<ObjectId, RigidBodyHandle>,
    fixture_bodies: BTreeMap<u32 /* fixture id */, RigidBodyHandle>,
    arm_link_bodies: BTreeMap<(u32 /* arm id */, u32 /* link slot */), RigidBodyHandle>,
}
```

## 4. Time stepping integration

Existing per-tick flow in `ArmWorld::consume_actuators_and_integrate_inner`:
```
1. Drain joint velocity commands → update q via PD-on-velocity logic
2. Drain gripper commands → update grasped state (apply_gripper_command)
3. Drain pending spawns → insert new Objects
4. gravity_step (kinematic gravity-fall)
5. Update grasped object pose to EE
6. Advance sim_time + sim_clock
```

New per-tick flow with Rapier:
```
1. Drain joint velocity commands → update q via PD-on-velocity (unchanged)
2. Drain gripper commands → update grasped state (Phase 1: magic weld; Phase 3: friction grasp)
3. Drain pending spawns → insert new Objects (also create Rapier bodies for them)
4. Update each arm-link kinematic body pose from current FK
5. PhysicsWorld::step(dt)   ← Rapier handles gravity, contact, friction, sleep
6. Sync Rapier body poses → Object.pose for every dynamic Object
7. Re-evaluate Settled/Free derivation from velocity / sleep state
8. Apply Grasped weld (Phase 1: kinematic body pose = EE pose)
9. Advance sim_time + sim_clock
```

Step 5 is the new heavy lift. Rapier's `physics_pipeline.step(...)` does collision detection, integration, constraint resolution. Deterministic given fixed inputs.

## 5. Domain ↔ physics sync

The domain side owns the canonical `Object.pose` field; Rapier is the engine that updates it. Each tick, after `physics_pipeline.step`:

```rust
for (obj_id, body_handle) in &self.object_bodies {
    let body = self.rigid_body_set.get(*body_handle).unwrap();
    let pose = body.position(); // Isometry3<f32>
    let obj = self.scene.object_mut(*obj_id).unwrap();
    obj.pose = *pose;
    obj.lin_vel = *body.linvel();
}
```

Inverse direction (Object → Rapier) only happens at:
- **Insertion** (new Object → create body with Object.pose)
- **Grasp transitions** (Object becomes kinematic; pose set from EE)
- **Manual pose override** (Phase 1: not needed beyond grasp)

Goals/queries (`PlaceInBin`, `NObjectsInBin`, etc.) read `Object.pose` as today — they don't see Rapier.

## 6. Settled / Free / Grasped under Rapier

Domain semantics survive but their derivation changes:

- **`Free`**: Object is a Dynamic Rapier body, not currently grasped, not at rest.
- **`Settled`**: Object is a Dynamic Rapier body, currently at rest. Derived per tick from `body.is_sleeping()` OR `body.linvel().norm() < ε_velocity` (default ε = 1e-3 m/s). The `on: SupportId` field of v1's `Settled { on }` becomes meaningful only as a *historical* hint — Rapier doesn't track "what's supporting what." For Phase 1, set `on: SupportId::Unknown` (new variant) when transitioning Free → Settled via Rapier; goals that need "is on the bin" should check pose-vs-bin-footprint directly, not the `on` field.
- **`Grasped { by }`**: Object's Rapier body is switched to `KinematicPositionBased`; pose is welded to EE each tick (Phase 1). State transition mechanics in `apply_gripper_command` are unchanged.

State transitions per tick:
- Free → Settled: when velocity threshold met (or Rapier sleep)
- Settled → Free: when Rapier wakes the body (any contact disturbance) OR when the controller pushes/grasps it
- Free / Settled → Grasped: gripper closes near a graspable object (existing logic)
- Grasped → Free: gripper opens (existing logic)

## 7. Arm-vs-object interaction

The arm chain stays kinematic — we drive joint angles directly via PD-on-velocity, then update each arm-link kinematic body's pose from forward kinematics. Each arm link gets a Rapier capsule collider (matching the visualization capsule).

This is the **hybrid kinematic-arm + dynamic-objects** model:
- Arm links are kinematic bodies → infinite effective mass → never pushed back by contact
- Dynamic objects in contact with arm links get pushed (or are constrained by the arm's pose)
- No joint torque *control* (still velocity-driven); no arm dynamics tuning

This makes "sweep and push" naturally fall out: the arm sweeps, link colliders contact object colliders, objects get pushed. No special-case code.

### 7.1 Joint torque sensing

Even though the arm doesn't react to contact (kinematic motion wins), the controller can *observe* contact forces via a new sensor channel. This is the primary motivation for the Rapier upgrade — making "feel resistance during sweep" possible.

**New port type** in `arm/src/ports.rs`:
```rust
#[derive(Clone, Debug)]
pub struct JointTorqueReading {
    pub joint: JointId,
    /// Estimated external torque about the joint axis, in N·m. Positive
    /// follows the joint's positive-rotation convention. Zero in free
    /// motion; non-zero only when downstream link colliders are in contact
    /// with dynamic Rapier bodies.
    pub tau: f32,
    pub sampled_at: Time,
}

impl SensorReading for JointTorqueReading {
    fn sampled_at(&self) -> Time { self.sampled_at }
}
```

**Computation** (each tick, after `physics_pipeline.step`, in `publish_sensors_for_dt`):

For each joint `i` (axis `a_i` in world frame, anchor position `p_i` in world frame, both from FK), sum the torque contribution of every external contact force on every link `≥ i` in the kinematic chain (i.e., every link distal to joint `i`):

```
τ_i = Σ over downstream link colliders L:
        Σ over contact pairs (L, other_collider):
          Σ over contact_points p with impulse J:
            a_i · ((p − p_i) × (J / dt))
```

Rapier exposes contact pairs and their impulses via `narrow_phase.contact_pair(...).manifolds[*].points[*]`. We divide by dt to convert impulse → time-averaged force. Filter to contact pairs where one collider belongs to an arm link and the other belongs to a dynamic Object (skip arm-vs-fixture if undesired; v1 includes both since fixtures still produce real reaction torques).

**Attach API:**
```rust
impl ArmWorld {
    pub fn attach_joint_torque_sensor(&mut self, joint: JointId, rate: RateHz)
        -> PortRx<JointTorqueReading>;
}
```

Default rate 1 kHz (matches encoders).

**No noise model in Phase 1** — torque readings are clean. A `Noise impl for JointTorqueReading` (perturbing `tau` by a Gaussian) can land later via the existing `GaussianNoise` wrapper without changes to this sensor's core code.

**Determinism**: contact-pair iteration in Rapier follows insertion order (deterministic given our BTreeMap-driven insertion); contact impulses are deterministic per Rapier's single-threaded step. The torque computation is pure math on those values.

**What this enables (controller side):**
- Stop sweeping the moment torque exceeds a threshold (faster than peak-tracking)
- Detect collisions during free-space motion (e.g., arm bumped into something unexpected)
- Estimate object mass / weight from grasp torque
- Future: torque-feedback controllers (PD with torque setpoint instead of velocity setpoint) — but that's well beyond Phase 1; the *sensor* infrastructure suffices for now.

## 8. Gravity

Replace v1's `gravity_step` with Rapier's per-step gravity force. Configured at `PhysicsWorld` construction:
```rust
self.gravity = if gravity_enabled {
    Vector3::new(0.0, 0.0, -9.81)
} else {
    Vector3::zeros()
};
```

The `ArmWorld::gravity_enabled: bool` flag stays at the domain layer — we just route it to Rapier. The existing `sim/src/gravity.rs` module is removed; its tests either move into `physics.rs` (rewritten as Rapier tests) or are deleted (some are testing kinematic-fall edge cases that no longer exist).

## 9. Determinism

Rapier is deterministic *within* the same:
- Rust compiler version
- Target architecture (CPU)
- Rapier crate version
- Sequence of operations (insertion order, command order, dt sequence)

This matches v1's existing determinism caveat (design §10: "fixed toolchain+target"). No new restriction.

Insertion order: we need to insert bodies in a deterministic order. Already enforced by our use of `BTreeMap` for object/fixture iteration (lexicographic by id) and the `pending_spawns: VecDeque` (FIFO by schedule time + insertion order).

Parallel features: **disable** Rapier's parallel features (`features = ["serde-serialize"]`, NOT `parallel`). Single-threaded steps are deterministic; parallel ones are not.

## 10. Visualization changes

Mostly insulated. The `Visualizable` impls on Object/Fixture stay as-is — they read `pose` and `shape` and emit `Primitive::Box`/`Sphere`/`Capsule`. Rapier sync (§5) ensures `pose` is up-to-date.

**Two concrete viz fixes needed for Phase 1:**
1. **Rotated boxes**: today's `RerunRecorder::record` for `Primitive::Box` uses `Boxes3D::from_centers_and_half_sizes` (centers only). With Rapier, objects rotate freely. Switch to the rotation-bearing rerun archetype variant (`Boxes3D::from_centers_and_half_sizes(...)` followed by `.with_quaternions(...)` or whatever rerun 0.21 exposes).
2. **Rotated spheres / capsules**: spheres are rotation-invariant (no fix). Capsules already pass orientation via the Visualizable impl — should already work but verify.

Phase 2 adds `Primitive::Line` overlays from Rapier's debug renderer. The recorder already handles `Primitive::Line` (added in Step 8.3d). No new primitive types.

## 11. Per-phase scope

### 11.1 Phase 1 — Rapier basics + joint torque sensor

Substantive integration. After this phase:
- Objects fall, settle, get pushed by the arm via real physics
- Controllers can read per-joint contact torque via the new `JointTorqueReading` channel — the primary user-visible new capability
- Existing tests continue to pass (with re-tuning where behavior changes)
- Magic-weld grasp preserved (no new gripper actuation model)
- The viz pipeline produces rotation-aware output

Implementation steps (~14–18 commits, exact breakdown in the implementation plan):
- Workspace dep + `rtf_sim::physics` module skeleton
- `PhysicsWorld` struct + body/collider construction from domain entities
- Per-tick step + pose sync
- Replace `gravity_step` with Rapier gravity
- Update `apply_gripper_command` for kinematic-body switch on grasp
- Update `pending_spawns` to also create Rapier bodies
- Rotation-aware Box rendering in `RerunRecorder`
- **`JointTorqueReading` port type + `attach_joint_torque_sensor` + per-tick contact-impulse → joint-torque computation** (see §7.1)
- **Headline e2e test** demonstrating joint-torque-driven sweep termination (e.g., a `find_by_touch` example: arm sweeps, exits sweep state on torque > threshold instead of pressure peak)
- Re-tune existing example tolerances (some seeds may need adjusted thresholds; sweep parameters in find_grasp_place may need updating since objects can now be displaced by sweep contact)

### 11.2 Phase 2 — `RapierDebugVisualizable`

Opt-in debug overlay. After this phase:
- A `RunConfig::with_debug_overlay(true)` or similar flag enables the overlay
- Rerun playback shows collider outlines, contact points, broadphase AABBs alongside the regular scene
- Useful when investigating "why is this object behaving weirdly?"

Implementation (~3–5 commits):
- Wire up `rapier3d::pipeline::DebugRenderPipeline`
- Define a small `DebugRenderBackend` impl that captures Rapier's line output as `Primitive::Line` entries
- Plumb through a `debug_overlay: bool` flag on `RunConfig` / `RunnableWorld::snapshot`
- Add a viz sub-feature (`viz-rerun-debug`?) for tests that want to verify the overlay

### 11.3 Phase 3 — Joint-attached grasping with friction-flavored slip detection

**Original framing was "friction-based grasping"** — gripper closes, fingers physically pinch the object via Coulomb friction at contact. After 12 tuning iterations during implementation (Step 3.5), this approach hit a fundamental Rapier limitation: kinematic-vs-dynamic friction coupling doesn't transfer rotational forces cleanly through contact patches. Pure-friction grasps reliably hold during translation but slip during fast yaw (best score reachable: 0.78 vs 0.9 acceptance bar). This is a well-known hard problem in rigid-body physics; no amount of tuning crosses it.

**Revised approach:** *joint-attached grasping with slip detection on joint impulse.* This is the same approach used by Isaac Sim, NVIDIA PhysX, and most production robotic simulators. After this phase:
- On grasp: a Rapier `FixedJoint` is created between the EE kinematic body and the object's dynamic body, anchored at the current relative offset.
- The object **stays Dynamic** — gravity, contact, fixture interactions all continue to apply.
- The visualization shows fingers physically gripping (friction contact lights up, finger separation tightens), making the model look like a real friction grasp even though the actual coupling is through the joint.
- Slip detection: per tick, query the joint's accumulated impulse magnitude. If it exceeds a configured threshold (proxying "this much load would have slipped under real friction"), release the joint and transition `Grasped → Free`.
- Release: gripper opens → joint removed.

**Honest framing:** this is a soft-weld dressed in physics-shaped API. It's *closer* to Phase 1's kinematic-weld than to true friction grasp. What it adds vs Phase 1: object stays Dynamic (preserves gravity / contact), slip is a real testable failure mode (joint impulse threshold), and the system is physics-coherent in the rerun output (no body-type flips). What it does NOT do: simulate continuous Coulomb friction at the contact patch as the actual mechanism of grip.

The `Grasped` state's *meaning*: "gripper is closed AND a `FixedJoint` is currently active between EE and object." Transitions:
- Free / Settled → Grasped: gripper closes within proximity of a graspable object (existing trigger)
- Grasped → Free: gripper opens (explicit release) OR joint impulse exceeds slip threshold (slip release)

Implementation (~5–8 commits):
- Promote finger boxes to Rapier kinematic colliders attached to EE (visual + contact only; not the actual coupling)
- Define gripper actuation model: `GripperCommand` extended with `target_separation: f32` (replaces binary `close: bool`)
- Configure friction coefficients on graspable objects (default µ = 0.5; mostly visual at this point — friction governs finger-block contact dynamics during close, not the held-object grip)
- Modify `apply_gripper_command` to drive finger separation toward target
- On grasp transition: insert `FixedJoint` between EE kinematic body and object dynamic body via Rapier's `ImpulseJointSet`
- On release: remove the joint
- Slip detection: per tick, sum the joint's accumulated impulse over the past step; threshold-trigger release
- Update `PickPlace`, `SearchAndPlace`, `ContinuousSearchAndPlace`, `FindByTouch` controllers to issue progressive close commands and (optionally) react to slip events

The `GripperCommand` API change is the biggest user-visible delta — break the API and update all four examples cleanly.

**Why not pursue real friction grasp further?** Solver iterations help (4 → 16 added 0.13 to e2e score) but plateau and become unstable past 32. Increasing finger friction past 10 has near-zero effect because the failure isn't tangential slip — it's rotational coupling failure between contact patches. The fundamental mechanism rigid-body solvers use for friction (impulse-based) doesn't transfer angular momentum cleanly through small contact areas at the rotational rates a yawing arm produces. Real-world friction grasps work because of compliance and adhesion at the macro scale; rigid-body sim doesn't model either. Rather than fight this with workarounds (tighter solvers, scenario restructuring), accepting the joint-based pattern matches industry practice and ships a working test.

## 12. What changes vs v1

| Thing | Pre-Rapier | Phase 1 | Phase 3 |
|-------|------------|---------|---------|
| Object motion | gravity-fall only | full rigid body dynamics | (same) |
| Object orientation | locked to identity | free rotation | (same) |
| Arm-vs-object contact | none (phases through) | arm pushes objects | (same) |
| Grasp model | weld to EE pose | weld to EE pose | friction grip |
| Gripper actuation | binary `close: bool` | binary (unchanged) | continuous `target_separation` |
| Settled detection | gravity-fall convergence | velocity / sleep state | (same) |
| Determinism | byte-identical per fixed-toolchain | byte-identical per fixed (toolchain, Rapier version) | (same) |
| Viz Box rendering | center + half-size only | center + half-size + rotation | (same) |
| `gravity_step` module | present | removed | (same) |
| `Settled.on: SupportId` | tracked | becomes hint-only | (same) |
| Joint torque sensor | absent | `JointTorqueReading` port available | (same) |

## 13. Test impact

Existing examples should mostly continue to work, but expect:

- **Tolerances** may need loosening — e.g., score thresholds in find_grasp_place might drop from `> 0.99` to `> 0.95` due to physics jitter affecting peak-tracking precision.
- **Find-grasp-place sweep**: arm-vs-block contact during sweep can now displace the block. Need to verify the sweep doesn't push the block out of the search region before the controller commits to descent. Likely need to slow the sweep and/or set a floor-friction high enough that the block resists displacement.
- **Continuous spawn**: same sweep concern.
- **Pick-place**: open-loop, knows block xy a priori; should work unchanged.
- **Reach-pose / faults**: no objects in scene; trivially unchanged.
- **release_falls**: fundamentally changes — object now bounces / rolls a bit instead of straight settling. May need to relax the "expected settled position" tolerance or rewrite as "object is at rest somewhere reasonable."

We'll update these in Phase 1 as needed; the implementation plan calls them out per step.

## 14. Out of scope for this extension

- **Soft body** simulation (cloth, ropes)
- **Fluid** simulation
- **Joint torque control** for the arm (arm stays velocity-controlled and kinematic)
- **Multi-arm** scenes (single arm, single domain id)
- **Articulated body** modeling for the arm via Rapier's multibody joints (we keep our hand-rolled FK; Rapier sees only the arm's link colliders as kinematic bodies)
- **Joint limits** enforcement via Rapier (still done in our IK)
- **Continuous collision detection (CCD)** for fast-moving objects (default disabled; can enable per-body if a scenario demands it)
- **Friction tuning** beyond a single µ-per-material (no static-vs-kinetic distinction in Phase 3 — Rapier supports it but we'd need to expose the API; defer until needed)

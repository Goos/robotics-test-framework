# Find-Grasp-Place Extension — Implementation Plan

Date: 2026-05-06
Status: Draft (companion to `2026-05-06-find-grasp-place-design.md`)

Standard workflow: TDD per step (failing test first, then implement). Each step is one commit. Every commit must keep `cargo test --workspace --features e2e` green and `cargo clippy --workspace --all-targets -- -D warnings` clean. Final V-gates after the last step (re-run V.1 twice for determinism, V.2/V.3/V.4 once).

Total: 7 commits.

---

## Step 1: `[sim] Distance-to-AABB-surface helper`

**File**: `sim/src/shape.rs` (extend).

Add a free function or method that computes Euclidean distance from a world-frame point to the surface of a shape with a given pose. For v1.x scope only `Shape::Aabb` is required (Sphere/Cylinder can `unimplemented!()` and be filled in later).

```rust
impl Shape {
    /// Euclidean distance from `point_world` to the nearest surface point of
    /// this shape posed at `pose`. Returns 0.0 if the point is inside the
    /// shape. For Aabb, computes distance from the point (in shape-local
    /// frame) to the box; for other variants, panics for now.
    pub fn distance_to_surface(&self, pose: &Isometry3<f32>, point_world: &Point3<f32>) -> f32 {
        // ...
    }
}
```

Implementation for Aabb:
- Transform `point_world` into shape-local frame via `pose.inverse() * point_world`.
- Per axis: `clamp(local.coord, -half_extent, +half_extent)` gives nearest point in local frame.
- Distance = `(local - nearest).norm()`. Returns 0 if point is inside box.

Failing tests in `sim/src/shape.rs`:
- Point above an Aabb at distance 0.05 → returns 0.05
- Point inside an Aabb → returns 0.0
- Point at corner of Aabb (just outside) → returns expected diagonal distance

Commit `[sim] Step 1: Shape::distance_to_surface for Aabb`.

---

## Step 2: `[arm] PressureReading port + sensor publish`

**Files**: `arm/src/ports.rs` (extend); `arm/src/world.rs` (extend).

In `arm/src/ports.rs`, add:

```rust
#[derive(Clone, Debug)]
pub struct PressureReading {
    pub pressure: f32,
    pub sampled_at: Time,
}

impl SensorReading for PressureReading {
    fn sampled_at(&self) -> Time { self.sampled_at }
}
```

In `arm/src/world.rs`:
- Track attached pressure sensors as `Vec<(PortTx<PressureReading>, RateHz, f32 /* eps */, Time /* last_published */)>` on `ArmWorld`.
- Add `pub fn attach_pressure_sensor(&mut self, rate: RateHz, eps: f32) -> PortRx<PressureReading>`.
- In `publish_sensors_for_dt`, for each pressure sensor: if elapsed since last publish ≥ rate's period, compute pressure from `ee_pose()` against all `Object`s and `Fixture`s using `Shape::distance_to_surface`, send the reading, advance `last_published`.

Pressure formula:
```rust
let mut max_pressure = 0.0_f32;
for (_, obj) in self.scene.objects() {
    let d = obj.shape.distance_to_surface(&obj.pose, &ee_point);
    if d <= eps { max_pressure = max_pressure.max((eps - d) / eps); }
}
for (_, fix) in self.scene.fixtures() {
    let d = fix.shape.distance_to_surface(&fix.pose, &ee_point);
    if d <= eps { max_pressure = max_pressure.max((eps - d) / eps); }
}
```

Failing tests in `arm/src/world.rs`:
- `pressure_sensor_zero_when_no_object_nearby`: build simple world (no objects close to EE), attach sensor, step once, assert pressure = 0.0.
- `pressure_sensor_falls_off_linearly_with_distance`: place a small Aabb directly below EE at known distance d (varying), assert pressure ≈ (eps - d)/eps within 1e-4.
- `pressure_sensor_saturates_inside_object`: EE inside an Aabb → pressure = 1.0.
- `pressure_sensor_publishes_at_configured_rate`: attach at 100 Hz, step world for 50ms, assert exactly 5 readings sent.

Commit `[arm] Step 2: PressureReading port + EE-mounted scalar pressure sensor`.

---

## Step 3: `[core,sim] NoiseSource::uniform_unit`

**Files**: `core/src/sensor_reading.rs` (or wherever `NoiseSource` lives); `sim/src/faults/pcg_noise_source.rs`.

Extend the trait:

```rust
pub trait NoiseSource {
    fn standard_normal(&mut self) -> f32;
    fn uniform_unit(&mut self) -> f32;
}
```

Implement for `PcgNoiseSource`:
```rust
fn uniform_unit(&mut self) -> f32 {
    use rand::Rng;
    self.0.gen::<f32>()  // rand::Rng::gen for f32 returns [0, 1)
}
```

Failing tests in `sim/src/faults/pcg_noise_source.rs`:
- `uniform_unit_in_range`: 100 draws, all in [0.0, 1.0).
- `uniform_unit_mean_near_half`: 1000 draws, mean ∈ [0.45, 0.55].
- `uniform_unit_same_seed_reproduces`: two `PcgNoiseSource::from_seed(42)` produce identical first 100 values.

If any other `impl NoiseSource` exists elsewhere (none expected — `PcgNoiseSource` is the only impl per Phase 9 wrap), add the new method there too.

Commit `[core,sim] Step 3: NoiseSource::uniform_unit`.

---

## Step 4: `[arm] build_search_world helper with seeded block placement`

**File**: `arm/src/test_helpers.rs` (extend).

Constants for the search region (place near top of the file):
```rust
pub const SEARCH_REGION_X: (f32, f32) = (0.40, 0.70);
pub const SEARCH_REGION_Y: (f32, f32) = (-0.25, 0.25);
pub const SEARCH_REGION_Z: f32 = 0.55;  // block top
pub const BLOCK_HALF_HEIGHT: f32 = 0.025;
```

```rust
pub fn build_search_world(seed: u64) -> ArmWorld {
    let mut src = rtf_sim::faults::PcgNoiseSource::from_seed(seed);
    let x = SEARCH_REGION_X.0 + src.uniform_unit() * (SEARCH_REGION_X.1 - SEARCH_REGION_X.0);
    let y = SEARCH_REGION_Y.0 + src.uniform_unit() * (SEARCH_REGION_Y.1 - SEARCH_REGION_Y.0);
    let block_z = 0.475 + BLOCK_HALF_HEIGHT;  // table top + half-height

    // Same scene/arm geometry as build_pick_and_place_world, but block xy is randomized.
    let mut scene = Scene::with_ground(0);
    scene.add_fixture(Fixture {
        id: 0,
        pose: Isometry3::translation(0.5, 0.0, 0.475),
        shape: Shape::Aabb { half_extents: Vector3::new(0.4, 0.4, 0.025) },
        is_support: true,
    });
    scene.add_fixture(Fixture {
        id: BIN_FIXTURE_ID,
        pose: Isometry3::translation(0.0, 0.6, 0.55),
        shape: Shape::Aabb { half_extents: Vector3::new(0.1, 0.1, 0.05) },
        is_support: true,
    });
    scene.insert_object(Object {
        id: BLOCK_OBJECT_ID,
        pose: Isometry3::translation(x, y, block_z),
        shape: Shape::Aabb { half_extents: Vector3::new(0.025, 0.025, BLOCK_HALF_HEIGHT) },
        mass: 0.1,
        graspable: true,
        state: ObjectState::Settled { on: SupportId::Fixture(0) },
        lin_vel: Vector3::zeros(),
    });

    // Same arm spec as build_pick_and_place_world.
    let spec = ArmSpec {
        joints: vec![
            JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-PI, PI) },
            JointSpec::Revolute { axis: Vector3::y_axis(), limits: (-PI, PI) },
            JointSpec::Revolute { axis: Vector3::y_axis(), limits: (-PI, PI) },
        ],
        link_offsets: vec![
            Isometry3::translation(0.0, 0.0, 0.8),
            Isometry3::translation(0.4, 0.0, 0.0),
            Isometry3::translation(0.4, 0.0, 0.0),
        ],
        gripper: GripperSpec { proximity_threshold: 0.05, max_grasp_size: 0.1 },
    };
    ArmWorld::new(scene, spec, /* gravity */ true)
}
```

Failing tests in the same module:
- `build_search_world_places_block_in_region`: build with seed 42, assert block xy is within the region bounds.
- `build_search_world_is_deterministic_per_seed`: build twice with seed 42, assert block xy identical.
- `build_search_world_varies_with_seed`: build with seeds 1 and 2, assert block xy differs.

Commit `[arm] Step 4: build_search_world helper with seeded block placement`.

---

## Step 5: `[arm] SearchAndPlace controller — sweep state`

**File**: `arm/examples/find_grasp_place.rs` (new).

Note: per the recent refactor (`4bed123`), purpose-built scenario controllers now live inline in their own `arm/examples/<scenario>.rs` file with `main()` + `#[test]` blocks. `SearchAndPlace` follows the same pattern — it does **not** go in `src/` (only `PdJointController` is library-grade infrastructure).

Create the new example file with:
- `SearchAndPlace` struct + `SearchAndPlaceState` enum + impl (only the `Sweeping` state implemented; post-contact states `unimplemented!()` until Step 6).
- `pub fn main()` left as a stub (`fn main() { /* Step 6 fills in */ }`) — actual scenario harness goes in Step 6.
- `#[cfg(test)] mod search_and_place_tests` for the sweep-phase mock-rig unit tests.

The unit tests cover only the sweep phase.

```rust
pub struct SearchAndPlace<R, P, Pr>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
    Pr: PortReader<PressureReading>,
{
    state: SearchAndPlaceState,
    sweep_waypoints: Vec<(f32, f32, f32)>,  // (J0, J1, J2) joint targets
    sweep_idx: usize,
    contact_xy: Option<(f32, f32)>,
    target_bin_xy: (f32, f32),
    pressure_threshold: f32,
    arm_shoulder_z: f32,
    l1: f32,
    l2: f32,
    encoder_rxs: Vec<R>,
    ee_pose_rx: P,
    pressure_rx: Pr,
    velocity_txs: Vec<PortTx<JointVelocityCommand>>,
    gripper_tx: PortTx<GripperCommand>,
    joint_tol: f32,
}

pub enum SearchAndPlaceState {
    Sweeping,
    DescendToContact,
    CloseGripper(u32),
    AscendWithBlock,
    YawToBin,
    OpenGripper(u32),
    Done,
    Failed,  // sweep exhausted without contact
}
```

Constructor takes:
- region bounds `(x_min, x_max, y_min, y_max)`,
- `sweep_z`, `stripe_dy`,
- bin_xy, arm geometry params,
- the four port collections.

In the constructor:
1. Generate xy waypoints in serpentine order over the region.
2. For each xy: call `ik_2r(radial=sqrt(x²+y²) /* relative to shoulder */, sweep_z - shoulder_z, l1, l2)` → (J1, J2). J0 = `atan2(y, x)`. Store (J0, J1, J2).
3. Save in `sweep_waypoints`.
4. Initial state = `Sweeping`, `sweep_idx = 0`.

Step logic for `Sweeping`:
```
let pressure = self.pressure_rx.latest().map(|r| r.pressure).unwrap_or(0.0);
if pressure > self.pressure_threshold {
    self.contact_xy = self.ee_xy();
    self.state = SearchAndPlaceState::DescendToContact;
    return Ok(());
}
let target = self.sweep_waypoints[self.sweep_idx];
self.drive_joints_toward(target);
if self.joints_converged_to(target) {
    self.sweep_idx += 1;
    if self.sweep_idx >= self.sweep_waypoints.len() {
        self.state = SearchAndPlaceState::Failed;
    }
}
```

Other states: `unimplemented!()` (with a comment "Step 6 fills these in").

Failing tests in `pick_place_tests` (or new `search_and_place_tests` module):
- `sweep_advances_waypoint_when_joints_converge`
- `sweep_transitions_to_descend_when_pressure_spikes`
- `sweep_transitions_to_failed_when_exhausted`
- `sweep_waypoints_cover_region_in_serpentine_order`

Commit `[arm] Step 5: SearchAndPlace controller skeleton + Sweeping state`.

---

## Step 6: `[arm] SearchAndPlace — descend, grasp, ascend, yaw, place`

**File**: `arm/examples/find_grasp_place.rs` (extend — same file from Step 5).

Fill in the remaining states. Logic mirrors `PickPlace`, but the IK targets for `DescendToContact` / `AscendWithBlock` use `contact_xy` (set at runtime, not constructor input):

```rust
DescendToContact => {
    let (cx, cy) = self.contact_xy.expect("set on entry");
    let target_radial = (cx * cx + cy * cy).sqrt();
    let (j1, j2) = ik_2r(target_radial, GRASP_Z - self.arm_shoulder_z, self.l1, self.l2)
        .expect("contact_xy reachable by construction");
    let yaw = cy.atan2(cx);
    let target = (yaw, j1, j2);
    self.drive_joints_toward(target);
    self.halt_gripper();
    if self.joints_converged_to(target) {
        self.state = SearchAndPlaceState::CloseGripper(0);
    }
}
CloseGripper(n) => /* same as PickPlace */,
AscendWithBlock => { /* IK at (contact_xy radial, 0.85) */ },
YawToBin => { /* IK at (bin_xy radial, 0.85), J0 toward bin */ },
OpenGripper(n) => /* same as PickPlace */,
Done => {},
Failed => /* keep joints halted, do nothing */,
```

`GRASP_Z` constant (~0.55, top-of-block height in the world).

Failing tests:
- `descend_transitions_to_close_when_joints_converge`
- `close_transitions_to_ascend_after_hold_ticks`
- `ascend_transitions_to_yaw_to_bin`
- `yaw_to_bin_transitions_to_open_when_yaw_and_joints_converged`
- `open_transitions_to_done`

Commit `[arm] Step 6: SearchAndPlace post-contact states (descend → grasp → ascend → yaw → place)`.

---

## Step 7: `[arm] Headline e2e — find_grasp_place across multiple seeds`

**File**: `arm/examples/find_grasp_place.rs` (extend — same file from Steps 5–6).

Per the new example pattern: `fn main()` runs one seed (e.g. seed=42) for interactive demo / `cargo run --example find_grasp_place`. The `#[test]` blocks below run all three seeds for CI assertions. Add a `[[example]] name = "find_grasp_place" required-features = ["examples"]` entry to `arm/Cargo.toml`. Add a `#[cfg(feature = "viz-rerun")] #[test]` save_rrd variant for one seed (mirrors pick_place pattern).

```rust
#![cfg(feature = "e2e")]

use rtf_arm::{
    examples_::SearchAndPlace,
    goals::PlaceInBin,
    test_helpers::{
        block_id, bin_id, build_search_world,
        SEARCH_REGION_X, SEARCH_REGION_Y,
    },
    RateHz,
};
use rtf_core::time::Duration;
use rtf_harness::{run, RunConfig, Termination};

fn run_one_seed(seed: u64) {
    let mut world = build_search_world(seed);
    let ports = world.attach_standard_arm_ports();
    let ee_pose_rx = world.attach_ee_pose_sensor(RateHz::new(100));
    let pressure_rx = world.attach_pressure_sensor(RateHz::new(1000), 0.03);
    let block = block_id(&world);
    let bin = bin_id(&world);

    let controller = SearchAndPlace::new(
        ports.encoder_rxs,
        ee_pose_rx,
        pressure_rx,
        ports.velocity_txs,
        ports.gripper_tx,
        SEARCH_REGION_X, SEARCH_REGION_Y,
        /* sweep_z */ 0.57,
        /* stripe_dy */ 0.05,
        /* target_bin_xy */ (0.0, 0.6),
        /* arm_shoulder_z */ 0.8,
        /* l1 */ 0.4,
        /* l2 */ 0.4,
    );
    let goal = PlaceInBin::new(block, bin);

    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(30))
        .with_seed(seed);
    let res = run(world, controller, goal, cfg);
    eprintln!("seed={}: terminated_by={:?}, final_time={:?}, score={}",
        seed, res.terminated_by, res.final_time, res.score.value);
    assert!(matches!(res.terminated_by, Termination::GoalComplete),
        "seed {} did not converge in 30s; terminated_by={:?}, score={}",
        seed, res.terminated_by, res.score.value);
    assert!(res.score.value > 0.9);
}

#[test] fn find_grasp_place_seed_1()    { run_one_seed(1); }
#[test] fn find_grasp_place_seed_42()   { run_one_seed(42); }
#[test] fn find_grasp_place_seed_1337() { run_one_seed(1337); }
```

Optionally, an additional `#[cfg(feature = "viz-rerun")]` `save_rrd` variant for one of the seeds (mirrors Phase 8 pattern).

Commit `[arm] Step 7: Headline e2e — find_grasp_place across 3 seeds`.

---

## Final verification

Re-run V.1–V.4 from `2026-05-05-robotics-test-framework-implementation-plan-v2.md`:
- V.1: `cargo test --workspace --release` twice; identical headline scores per seed.
- V.2: `cargo clippy --workspace --all-targets -- -D warnings` clean (default + e2e + viz-rerun configs).
- V.3: `cargo fmt --all -- --check` exit 0.
- V.4: `cargo doc --workspace --no-deps` warning-free.

If any V-gate breaks: a small follow-up commit (e.g. `[workspace] cargo fmt`) is fine, no need to amend Step 7.

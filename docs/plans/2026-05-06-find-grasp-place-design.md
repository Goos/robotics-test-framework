# Find-Grasp-Place Extension — Design

Date: 2026-05-06
Status: Draft (extends `2026-05-05-robotics-test-framework-design-v2.md`)

## 1. Scope and motivation

V1 ships a fully open-loop pick-and-place: the controller knows the block's xy a priori and drives the EE there directly. This extension adds the first **closed-loop sensing** scenario:

- A new **scalar pressure sensor** at the EE.
- A **randomized block placement** within a fixed search region (seeded for determinism).
- A **`SearchAndPlace` controller** that sweeps the search region using the pressure sensor as its only source of object-location information, then grasps + places into a known bin.

This stresses the framework in three ways v1 didn't:
1. Forces a new sensor type through the existing `PortReader<T>` abstraction (verifies the boundary holds for non-encoder data).
2. Introduces seeded scene randomization (verifies harness determinism survives non-trivial randomization at construction time).
3. Requires the controller to make decisions from sensor data, not from pre-known scene state.

This is purely additive. No breaking changes to v1 design or APIs.

## 2. Pressure sensor model

### 2.1 Reading type

Lives in `arm/src/ports.rs`:

```rust
pub struct PressureReading {
    /// Scalar in [0.0, 1.0+]. 0 = no contact within sensing range; values
    /// above ~0.5 indicate strong proximity / contact.
    pub pressure: f32,
    pub sampled_at: Time,
}

impl SensorReading for PressureReading {
    fn sampled_at(&self) -> Time { self.sampled_at }
}
```

Scalar (not binary) is chosen for **future-proofing**: the same reading shape can later carry penetration depth, multi-finger force estimates, or noise-perturbed values without an API break.

### 2.2 Computation

At each `publish_sensors_for_dt`, the world computes:

```
pressure = max over Object o of:
    let d = distance(EE position, o.shape surface in world frame)
    if d <= ε: (ε - d) / ε
    else: 0.0
```

Note: Fixtures are deliberately excluded from the scan. Fixtures are support
surfaces (table, bin, ground), not contact targets, and including them
caused spurious sweep-altitude triggers when joint-space interpolation
dipped EE z toward the table top mid-stripe. Boundary detection against
fixtures, if needed in a future scenario, would belong on a separate
sensor type.

Where:
- `EE position` = `world.ee_pose().translation`.
- `distance(point, shape)` is signed-positive distance to the nearest surface point. For `Aabb`: `max(0, |dx|-hx) + max(0, |dy|-hy) + max(0, |dz|-hz)` (L∞-ish) or proper Euclidean — we'll use L2 (Euclidean from EE to closest point on box).
- `ε` is configured per-sensor (default 0.03 m = 3 cm).

This is a **proximity-falloff** model — the EE doesn't have to physically touch a surface to register pressure, it just has to be within ε. Justified for v1.x because the kinematic sim has no link-collision detection anyway; "real contact" would require collision modeling we don't have.

### 2.3 Attach API

```rust
impl ArmWorld {
    pub fn attach_pressure_sensor(&mut self, rate: RateHz, eps: f32)
        -> PortRx<PressureReading>;
}
```

Default rate 1 kHz (matches encoders). Default eps 0.03 m.

## 3. Randomization

### 3.1 NoiseSource extension

The `NoiseSource` trait (added in Phase 9 wrap-up) currently only exposes `standard_normal`. Add one method:

```rust
pub trait NoiseSource {
    fn standard_normal(&mut self) -> f32;
    fn uniform_unit(&mut self) -> f32;  // NEW: in [0.0, 1.0)
}
```

`PcgNoiseSource` implements via `rand::Rng::gen::<f32>()` (or `rand_distr::Uniform`).

### 3.2 Seeded placement

`arm/src/test_helpers.rs::build_search_world(seed: u64) -> ArmWorld`:

- Same arm geometry as `build_pick_and_place_world` (Z-Y-Y, pedestal-mounted).
- Same table fixture, same bin fixture.
- One graspable block placed at a uniform-random xy within the search region (defined below); z = table top + block half-height (sits on table surface).
- Internal: constructs `PcgNoiseSource::from_seed(seed)`, draws two `uniform_unit` for xy.

### 3.3 Search region

Fixed Aabb on the table top:

| Axis | Min  | Max  | Width |
|------|------|------|-------|
| x    | 0.40 | 0.70 | 0.30 m |
| y    | -0.25| 0.25 | 0.50 m |
| z    | 0.55 | 0.55 | (block top sits at 0.55 m) |

Selected so that:
- All points in the region are reachable by the arm (max distance from shoulder ≈ 0.74 m, within 0.8 m total reach).
- No overlap with bin xy (bin at y=0.6, region y ∈ [-0.25, 0.25]).
- No overlap with pedestal xy (pedestal at origin, region x ∈ [0.4, 0.7]).
- Sweep altitude (see §4.2) stays clear of the table top.

The controller is given the region bounds at construction; it does **not** know the block's xy.

## 4. SearchAndPlace controller

### 4.1 State machine

```rust
enum SearchAndPlaceState {
    Sweeping { waypoint_idx: usize },
    DescendToContact,         // pressure peaked, descending to grasp altitude
    CloseGripper(u32),
    AscendWithBlock,
    YawToBin,
    OpenGripper(u32),
    Done,
}
```

Same conventions as `PickPlace` (PD-on-velocity per joint, IK targets pre-computed at construction, joint convergence threshold drives transitions).

### 4.2 Sweep waypoints

Pre-computed at construction time. Serpentine raster over the search region at fixed `sweep_z`:

```
sweep_z = 0.57 m   // 2cm above block top, well above table top (0.5)
stripe_dy = 0.05 m // 5cm between rows
```

Generates ~10 stripes × 2 endpoints each = ~20 xy waypoints. For each, IK is called once at construction to convert (x, y, sweep_z) → (J0, J1, J2). Joint waypoint list stored.

### 4.3 Sweeping state

Each tick:
1. Read pressure. If `pressure > pressure_threshold` (default 0.2): record current EE xy as `contact_xy`, transition to `DescendToContact`.
2. Otherwise: drive joints toward `joint_waypoints[waypoint_idx]` via PD-on-velocity (same pattern as PickPlace's other states).
3. If joints converge to current waypoint and pressure still 0: advance `waypoint_idx`. If exhausted: signal failure (controller returns `ControlError::SearchExhausted`, harness terminates).

### 4.4 DescendToContact

IK-driven descent from `(contact_xy, sweep_z)` to `(contact_xy, grasp_z)` where `grasp_z = 0.55` (top of block). Transitions to `CloseGripper(0)` on joint convergence.

### 4.5 Close → Ascend → Yaw → Open

Same logic as `PickPlace::CloseGripper`/`AscendWithBlock`/`YawToBin`/`OpenGripper`. Reuses IK-target pre-computation pattern. Block-xy for IK targets is `contact_xy` (learned at runtime, not constructor input).

### 4.6 Goal — reuse `PlaceInBin`

No new goal needed. Phase 7's `PlaceInBin::new(block_id, bin_id)` already scores by "is block within bin's xy footprint and Settled." Test passes the block id and bin id from the world.

## 5. Interaction with existing framework

| Subsystem | Change |
|-----------|--------|
| `core::sensor_reading::NoiseSource` | Add `uniform_unit` method (extension, not breaking — but every `impl NoiseSource` in the codebase needs the method added). |
| `arm::ports` | Add `PressureReading` struct + `SensorReading` impl. |
| `arm::world` | Add `attach_pressure_sensor`; add pressure publish in `publish_sensors_for_dt`. Need a small helper `distance_to_aabb_surface(point, pose, half_extents)` somewhere reusable (likely `sim::shape` since `Shape::Aabb` lives there). |
| `arm::test_helpers` | Add `build_search_world(seed)`. |
| `arm::examples_` | Add `SearchAndPlace` controller. |
| `arm::tests/search_and_place_e2e.rs` | New e2e. |

No design changes to `core` traits, `harness` lifecycle, or `viz` rendering (this scenario produces no new primitive types).

## 6. Visualization

Reuses everything from Phase 8. The arm visualizes as it does today; the block visualizes as a Box. The serpentine sweep should be visually obvious in the rerun playback. No `Primitive` extensions, no new `EntityId` variants.

## 7. Testing strategy

- **`PressureReading` model**: unit test in `arm/src/world.rs` — place block at known position, verify pressure with EE at: (a) far away → 0, (b) directly above at distance 0.5ε → 0.5, (c) inside block bounding box → 1.0+ (we don't clamp upper bound, which is fine for "future-proofing").
- **`uniform_unit`**: unit test on `PcgNoiseSource` — 1000 draws, assert mean ≈ 0.5 ± 0.05.
- **`build_search_world`**: unit test — same seed produces same block xy; different seeds produce different block xy.
- **`SearchAndPlace` state transitions**: rig-style tests as in PickPlace — stub ports with publishers, verify each state transition fires under the right conditions.
- **Headline e2e**: `find_grasp_place_finds_random_block_and_drops_in_bin` — runs 3 seeds (e.g. 1, 42, 1337), each must reach `Termination::GoalComplete` within a 30 s deadline with `score > 0.9`. Optionally save_rrd variant for visual inspection.

## 8. Out of scope

- **Per-finger pressure**: single EE-mounted scalar only. Per-finger is a v1.x+ extension.
- **Distractor objects**: single graspable object only. Adding distractors would require pressure-channel disambiguation (e.g., signed pressure per "object class").
- **Real contact physics**: kinematic proximity-falloff only. True contact mechanics are a v2 concern.
- **Search-strategy intelligence**: hard-coded serpentine raster only. Spiral, gradient-following, or learned strategies are explicit non-goals — they belong in the SNN phase.
- **Block placement adversarial corners**: for v1.x the search region is large enough that average-case performance is what matters. We don't optimize for "block placed exactly at corner of region."

# Robotics Test Framework — Design (v2)

**Date:** 2026-05-05 (revised same-day after plan review)
**Status:** Revised after adversarial design review (round 1) and amended after adversarial plan review (round 1).
**Supersedes:** [`2026-05-05-robotics-test-framework-design.md`](2026-05-05-robotics-test-framework-design.md)
**Reviews addressed:** [`2026-05-05-robotics-test-framework-design-review-1.md`](2026-05-05-robotics-test-framework-design-review-1.md), [`2026-05-05-robotics-test-framework-implementation-plan-review-1.md`](2026-05-05-robotics-test-framework-implementation-plan-review-1.md)

---

## Changes from v1

**User-driven design changes** (T1–T3 from review tradeoffs):
- **Gravity-fall is now a v1 feature, on by default.** Released objects fall under simple kinematic gravity until they meet a fixture or another settled object. `Stack` becomes a sound v1 goal. Pure-kinematic mode is still selectable per-test.
- **`PortKind` removed; `LatestWins` semantics only for v1.** All sensor/actuator channels are single-slot overwriting buffers. Sidesteps the silent-misbehavior issue. Other channel kinds may return when v1.x grows event-style ports.
- **Explicit "Behavioral Contract for Ports" section added** (§9). Lightweight HAL contract — controllers get a documented checklist of what they may and may not assume. Sim is not yet "realistic by default"; that's a v2 axis.

**Issues addressed from the review (clear fixes)**:
- Port registry spelled out (§5.1).
- SNN drift bug fixed using fixed-step accumulator + integer arithmetic (§8.2).
- Multi-rate sensor scheduling: phase accumulator with documented ±1-tick jitter; readings carry sample timestamp (§5.6).
- Visualizer snapshot collection: `RunnableWorld::snapshot(&self) -> SceneSnapshot`; the world is the single point that walks its own visualizables (§7).
- `Goal<W>` simplified — `View` indirection dropped (`Goal<W: WorldView>` directly over the owned world type).
- Determinism audit extended (§10) — forbids `HashMap` in tick path, threading in core/sim, and `Instant::now()`. Caveats f32 transcendentals to "fixed toolchain+target."
- Test isolation: `harness::run` documented as single-threaded, no background threads, no global state (§6).
- `Time` is `i64` nanos; `Time - Time → Duration` (well-defined sign).
- `Recorder` made generic (`R: Recorder`) on `harness::run` to monomorphize.
- `Score` gains a `breakdown: SmallVec<(&'static str, f64), 4>` field, empty by default.
- `ControllerEvent` recording added behind a feature flag.
- Sensor-noise combinator added in `sim::faults`.
- `faults` collapsed into `sim::faults` (one fewer crate).
- `Sequence` composite goal dropped from v1.

**Crate count:** v1 had 6; v2 has 5 (`faults` collapsed into `sim`).

**Amendments after plan review (same-day):**
- Added object-safe `PortReader<T>` trait in §4. Both `PortRx<T>` and every `sim::faults` combinator implement it; controllers should be generic over `R: PortReader<T>` (or take `Box<dyn PortReader<T>>`) so wrapping a port with a fault combinator does not require source changes to the controller.
- Dropped `T: Send` bound from `port<T>`; the framework is single-threaded by §10 so the bound is gratuitous.
- `PortRx<T>` is intentionally `!Clone` (single-consumer guarantee) — fault wrappers consume the receiver and return a new `PortReader<T>`.
- **Crate naming:** all workspace member crates use the `rtf_` prefix (`rtf_core`, `rtf_sim`, `rtf_arm`, `rtf_harness`, `rtf_viz`) to avoid collision with stdlib `core` and for naming consistency. References to bare `core`/`sim`/`arm`/etc. throughout the doc refer to these crates by their conceptual short names.
- **Added §9.5 "Controller threading and external resources"** — clarifies that the framework's single-threading constraint applies to the framework only; controllers may use threads, GPU, or external services internally, as long as `step()` blocks until outputs are finalized and outputs are deterministic. Old §9.5 (deferred items) renumbered to §9.6.

---

## 1. Problem & Goals

Build a Rust framework for simulating simple robotics tasks and testing controller "business logic" against deterministic, mockable physical layers.

**Functional goals:**
- Simulate at least the **robotic arm / manipulator** modality, with the architecture cleanly extensible to mobile robots, drones, etc.
- Run a controller against a simulated world, score how well it achieved a defined goal, return a result.
- Tests are **code-defined** (Rust `#[test]`), discovered and run by `cargo test`.

**Non-functional goals (load-bearing):**
- **Determinism.** Same scenario + same seed → bit-identical outcome on a fixed toolchain+target.
- **HAL-style abstraction.** The controller cannot see the simulator. Same controller code can later run against real hardware *if it abides by the documented behavioral contract* (§9).
- **Mockability.** Sensor/actuator channels, the clock, and individual port wrappers can all be substituted, including by fault-injecting wrappers that simulate hardware-style misbehavior.
- **No premature flexibility.** Anywhere a future need is *anticipated* but not *current*, the design leaves a clean extension point but does not build for it.

**Eventual intent (informs design but is not v1 scope):**
- Business logic implemented as a **spiking neural network (SNN)** running at its own internal update rate.
- Same controllers eventually running against **real hardware** (HAL).

---

## 2. Key Decisions

| Decision | Choice | Why |
|---|---|---|
| Language | Rust | First-class channels/traits/lifetimes; workspace crates physically enforce abstraction boundaries. |
| Simulation fidelity | Kinematic + simple gravity-fall (default) | Determinism preserved; enables `Stack` and avoids "release-from-millimeter" servoing. |
| Controller programming model | Typed-channel "ports" with controller owning its own clock | Subsumes Gym-style and SNN; HAL-friendly. |
| Channel semantics (v1) | `LatestWins` only | Sensors/actuators all want freshest. Avoids silent-misbehavior issues; defers events to v1.x. |
| Test format | Code-defined (`#[test]`) | Lowest infra overhead; full type system; cargo runs tests. |
| Crate organization | Workspace, 5 crates | Compiler enforces dependency arrows. |
| Visualizer | Out-of-process via `Recorder` trait; **rerun.io** as default backend | Tests stay headless. `viz` only sees `Primitive`s; never imports robot-kind crates. |
| Snapshot collection | `RunnableWorld::snapshot(&self) -> SceneSnapshot` | The world is the only thing that knows about all its visualizables. Single, named integration point. |
| Actuator vocabulary (v1) | Joint velocity commands only | Simplest, integrates trivially. Position/torque later. |
| Math dep | `nalgebra` | Standard in Rust robotics. |
| HAL contract | Lightweight: documented behavioral contract for ports (§9) | Controller authors get a checklist; sim doesn't add forced-realism in v1. |
| Determinism scope | Bit-identical *on fixed toolchain+target* | Honest about f32 transcendental cross-platform reproducibility. |

---

## 3. Architecture

### Crate layout (Cargo workspace)

```
robotics-test-framework/
├── core/         — traits + types only. The contract.
├── sim/          — kinematic simulator (incl. gravity, faults submodule).
├── arm/          — manipulator-specific: kinematic chain, FK, gripper, arm port types.
├── harness/      — test runner: build sim, run controller, score, return result.
├── viz/          — visualizer (thin rerun-sdk wrapper).
└── examples/     — your scenarios as `#[test]`s.
```

### Dependency arrows (strict, one-way)

```
arm     → core
sim     → core
arm     → sim
harness → sim, core
viz     → sim                 (NEVER → arm; only sees Primitives)
```

`core` depends on **nothing**. It defines `Controller`, `Port<T>`, `Clock`, `WorldView`, `Goal`, `Score`, and supporting types. No knowledge of arms, kinematics, scene geometry, or simulators.

### Where the user's controller code lives

Controllers depend only on `core` + the specific port types they consume (e.g., `arm::ports::JointEncoderReading`). They never import `sim` or `harness`. **HAL boundary**, enforced by the workspace.

---

## 4. Core Abstractions (`core`)

### Time, Duration, Clock

```rust
pub struct Time(i64);          // signed nanos since scenario start
pub struct Duration(i64);      // signed nanos

impl Sub for Time { type Output = Duration; ... }   // well-defined ordering
impl Add<Duration> for Time { ... }

pub trait Clock { fn now(&self) -> Time; }
```

**Signed nanos:** lets `t - last_t` be a well-defined `Duration` even if a controller passes times in unexpected order; eliminates the unsigned-wrap footgun. Range is ±292 years; ample.

Clock is injected. Sim provides `SimClock` (advances under sim control). Real hardware provides `MonotonicClock` (`std::time::Instant`-backed). **Controllers never read a global wall clock.**

### Typed Ports — `LatestWins` only (v1)

```rust
pub fn port<T: Clone + 'static>() -> (PortTx<T>, PortRx<T>);

impl<T> PortTx<T> {
    pub fn send(&self, v: T);          // overwrites any prior unread value
}
impl<T> PortRx<T> {
    pub fn latest(&self) -> Option<T>;     // non-destructive read of the freshest
    pub fn take(&mut self) -> Option<T>;   // destructive read; clears slot
}
// PortRx<T> is intentionally !Clone — single-consumer guarantee. Wrapping in faults
// combinators (§5.8) consumes the receiver and returns a new PortReader<T>.
```

**`T: Send` is intentionally not required.** Per §10, the entire framework is single-threaded by construction; ports are not crossed across threads. Imposing `Send` would gratuitously block non-`Send` reading types (e.g., readings carrying `Rc` payloads).

### `PortReader<T>` — object-safe reader trait

To make `sim::faults` combinators (Delay, DropMessages, etc.) transparent to controllers, both `PortRx<T>` and every fault wrapper implement an object-safe reader trait:

```rust
pub trait PortReader<T> {
    fn latest(&self) -> Option<T>;
    fn take(&mut self) -> Option<T>;
}

impl<T: Clone> PortReader<T> for PortRx<T> { /* delegate to inherent methods */ }
```

Controllers should be generic over `R: PortReader<T>` (or store `Box<dyn PortReader<T>>` when heterogeneous wrappers are used at runtime). This way the same controller code accepts a raw `PortRx<JointEncoderReading>` from sim, a `Delay<PortRx<JointEncoderReading>, _>` in a robustness test, or a hardware-backed reader in `rtf_arm_real` — all without source changes.

Single-slot overwriting buffer. No `Fifo`/`Buffered` variants in v1 — we have no events, so we don't need them. Adding them later means adding new `port_*` constructors and new endpoint types; existing `LatestWins` ports are unaffected.

**Sender lives somewhere; receiver lives somewhere else.** The world owns one half (Tx for sensors, Rx for actuators); the controller owns the other half. See §5.1 for the registry that makes this scale.

### Controller (the system under test)

```rust
pub trait Controller {
    fn step(&mut self, t: Time) -> Result<(), ControlError>;
}

pub struct ControlError {
    pub kind: ControlErrorKind,
    pub detail: Cow<'static, str>,
}
pub enum ControlErrorKind { Unrecoverable, Recoverable }
```

The controller has no observation argument and no action return. It owns its port endpoints as fields (handed in at construction) and reads/writes them inside `step`. `step(t)` is just "framework telling you a tick has elapsed." `Recoverable` errors continue the run; `Unrecoverable` ends it.

For SNNs: `step` advances the SNN's internal clock by N microsteps and reads/writes channels at whatever cadence the SNN wants — see §8.2.

### Score

```rust
pub struct Score {
    pub value: f64,                                // convention: [0.0, 1.0]
    pub breakdown: SmallVec<[(&'static str, f64); 4]>, // optional components
}
```

`value` is the headline; `breakdown` is empty by default. Composite scoring (smoothness + accuracy + time) becomes a struct-population exercise, not an API redesign.

### World view & scoring

```rust
pub trait WorldView {}  // marker; specific worlds extend with own accessors

pub trait Goal<W: WorldView> {
    fn tick(&mut self, t: Time, world: &W) {}
    fn is_complete(&self, world: &W) -> bool { false }
    fn evaluate(&self, world: &W) -> Score;
}
```

`Goal` is parameterized over the **owned** world type (no GAT, no `View` indirection — v1 doesn't need a borrowed projection; if it ever does, that's a future change with a known answer).

**Key invariant:** `Goal` *can* see the world — it is test scaffolding. `Controller` cannot.

---

## 5. Kinematic Simulator with Gravity (`sim` + `arm`)

### 5.1 Port registry — explicit ownership

The world holds, per port type, a `BTreeMap<PortId, _>`:

```rust
// in core
pub struct PortId(u32);          // monotonic, assigned by the world

// in arm::world (sketch)
pub struct ArmWorld {
    pub scene: Scene,
    pub arm: Arm,
    pub arm_state: ArmState,
    // sensor publishers: world owns the Tx side
    sensors_joint_encoder: BTreeMap<PortId, SensorPublisher<JointEncoderReading>>,
    sensors_ee_pose:       BTreeMap<PortId, SensorPublisher<EePoseReading>>,
    // actuator consumers: world owns the Rx side
    actuators_joint_velocity: BTreeMap<PortId, ActuatorConsumer<JointVelocityCommand>>,
    actuators_gripper:        BTreeMap<PortId, ActuatorConsumer<GripperCommand>>,
    next_id: u32,
}
```

`SensorPublisher<T>` wraps `(PortTx<T>, RateScheduler, source: SensorSource)`. `ActuatorConsumer<T>` wraps `(PortRx<T>, sink: ActuatorSink)`. Both are concrete types in `arm`.

**`attach_*` returns the opposite endpoint to the caller, who hands it to the controller:**

```rust
let encoder_rx: PortRx<JointEncoderReading> =
    world.attach_joint_encoder_sensor(JointId(0), RateHz(1000));
let velocity_tx: PortTx<JointVelocityCommand> =
    world.attach_joint_velocity_actuator(JointId(0));
```

**Per-tick iteration:** the world's tick method iterates each `BTreeMap` in `PortId` order. `BTreeMap` iteration is sorted-by-key — deterministic by construction. All publish and consume operations within a tick happen in a stable, named order.

### 5.2 Scene model (`sim`, robot-agnostic)

- **Fixtures** — immutable geometry (ground, table, bin). `id`, `pose`, primitive shape, `is_support: bool`.
- **Objects** — mutable: `id`, pose, primitive shape (sphere / AABB / cylinder), `mass: f32`, `graspable: bool`, `state: ObjectState { Free, Grasped(by_arm), Settled(on: SupportId) }`.
- **Robots** — provided by robot-kind crates.

No physics engine. Primitives only.

### 5.3 Kinematic arm (`arm`)

```rust
pub struct ArmSpec {
    pub joints: Vec<JointSpec>,            // revolute/prismatic, axis, limits
    pub link_offsets: Vec<Isometry3<f32>>,
    pub gripper: GripperSpec,
}
pub struct ArmState {
    pub q: Vec<f32>, pub q_dot: Vec<f32>,
    pub gripper_closed: bool,
    pub grasped: Option<ObjectId>,
}
```

FK is a chain of transform multiplies — ~50 lines. EE pose = last link transform.

### 5.4 Grasping

Each tick after FK:
- Gripper closing + nothing held + a `graspable` `Free` object's pose within `gripper.proximity_threshold` of EE → attach. Object state → `Grasped`.
- While `Grasped`, object pose is rigidly slaved to EE pose.
- Gripper opens → detach. Object state → `Free`. Gravity applies next tick (§5.5).

### 5.5 Gravity-fall (kinematic + simple support resolution)

This is the v2-promoted feature; it lives directly in `sim` and is on by default.

**Per tick, after FK and grasp updates, for each `Free` object `o`** (sorted by ascending `q.z` for determinism):
1. Apply gravity: `o.q_dot.z -= GRAVITY * dt` (default `GRAVITY = 9.81`).
2. Integrate position: `o.q.z += o.q_dot.z * dt`.
3. Find highest support: walk fixtures with `is_support = true` and other `Settled` objects, take the highest `top_z` whose xy footprint contains `o.xy` (within object's footprint).
4. If `o.bottom_z <= top_z + EPSILON`: snap `o.q.z = top_z + o.half_height_z`, set `o.q_dot.z = 0`, transition `state → Settled(on: support_id)`.
5. Else: `o` remains `Free`, falling.

**Settled objects are recomputed** if their support disappears (e.g., gripper grasps the support) — at the start of each tick, every `Settled(on: X)` object is checked: if `X` no longer exists or no longer occupies `o.xy`, transition back to `Free` and re-run the per-tick logic.

**Lateral motion:** none. Free objects only move on `z`. Released-then-falling objects retain the `xy` they had at release time. This keeps determinism trivial and matches the pure-kinematic spirit of v1.

**Per-test override:** `RunConfig::with_gravity(false)` returns to pure-kinematic-stay-put mode for tests that care.

**Determinism notes:**
- Iteration order is by ascending `q.z` then by `id` for ties — stable.
- `EPSILON` is a single named constant (default `1e-5` m).
- No iterative contact solver; one linear pass per tick.

### 5.6 Multi-rate sensor scheduling

Each sensor has a `RateHz`. Internally, a `RateScheduler` carries a phase accumulator in nanos:

```rust
pub struct RateScheduler { period_ns: i64, accumulator_ns: i64 }

impl RateScheduler {
    pub fn tick(&mut self, dt_ns: i64) -> bool {
        self.accumulator_ns += dt_ns;
        if self.accumulator_ns >= self.period_ns {
            self.accumulator_ns -= self.period_ns;
            true
        } else { false }
    }
}
```

For `RateHz(333)` against sim at 1000 Hz, the scheduler fires roughly every 3 ticks but with slight jitter (±1 tick) so that the *long-run rate* matches 333 Hz exactly. **Sensor readings carry the sample timestamp**, not the schedule:

```rust
pub struct JointEncoderReading {
    pub joint: JointId,
    pub q: f32, pub q_dot: f32,
    pub sampled_at: Time,   // authoritative — controllers must use this, not a derived schedule
}
```

This is required by the behavioral contract (§9.4).

### 5.7 Tick loop

Fixed step, default 1000 Hz. Per tick at sim time `t`:

1. **Re-evaluate settled objects** (§5.5 last paragraph).
2. **Gravity integration** for `Free` objects (§5.5).
3. **Forward kinematics** for the arm.
4. **Grasp update** (§5.4).
5. **Sensor publish:** iterate every sensor `BTreeMap` in `PortId` order. For each, call `scheduler.tick(dt)`; if true, build a reading from current world state and `tx.send(reading)`.
6. **Controller step:** call `controller.step(t)`. Controller may read sensors and write actuators.
7. **Actuator consume:** iterate every actuator `BTreeMap` in `PortId` order. For each, `rx.latest()` → apply to world state (e.g., joint velocity command updates `arm_state.q_dot[joint]`).
8. **Time advance:** `sim_time += dt`. World now at `t + dt`.

**Single-threaded, no background tasks, no global state.** See §10.

### 5.8 `sim::faults` — channel and reading wrappers

In `sim::faults`:

```rust
faults::DropMessages::new(rx, drop_rate, seed) -> PortRx<T>
faults::Delay::new(rx, latency)                -> PortRx<T>
faults::StaleData::new(rx, max_age)            -> PortRx<T>
faults::GaussianNoise::new(rx, stddev_fn, seed) -> PortRx<T>  // for sensor reading types that expose noise points
```

Each wrapper is itself a `PortRx<T>` — transparent to the controller. All deterministic given a seed.

---

## 6. Test Harness & Scoring (`harness`)

### Setup is two-phase

Build world → wire ports → run harness. Forced by ownership — port endpoints come from the world.

### World builder + port wiring

```rust
let mut world = arm::ArmWorldBuilder::new()
    .with_arm(franka_panda_spec())
    .with_object(Object::block().at([0.5, 0.0, 0.05]).graspable())
    .with_fixture(Fixture::table().at([0.5, 0.0, 0.0]).extents([1.0, 1.0, 0.05]))
    .with_fixture(Fixture::bin().at([0.0, 0.5, 0.0]).extents([0.2, 0.2, 0.1]))
    .build();

let encoder_rx  = world.attach_joint_encoder_sensor(JointId(0), RateHz(1000));
let velocity_tx = world.attach_joint_velocity_actuator(JointId(0));
let gripper_tx  = world.attach_gripper_actuator();
let controller  = MyController::new(encoder_rx, velocity_tx, gripper_tx);
```

Helpers like `world.attach_standard_arm_ports() -> StandardArmPorts` can wrap common bundles.

### Run loop — single-threaded, no spawned threads, no global state

```rust
pub fn run<W, C, G, R>(
    world: W,
    controller: C,
    goal: G,
    cfg: RunConfig<R>,
) -> RunResult
where
    W: RunnableWorld,
    C: Controller,
    G: Goal<W>,
    R: Recorder;

pub struct RunConfig<R: Recorder = NullRecorder> {
    pub tick_rate: RateHz,            // default 1000 Hz
    pub deadline: Duration,
    pub seed: u64,
    pub recorder: R,
    pub gravity: bool,                // default true
}

pub struct RunResult {
    pub score: Score,
    pub final_time: Time,
    pub terminated_by: Termination,
}
pub enum Termination {
    GoalComplete,
    Deadline,
    ControllerError(ControlError),
}
```

Termination on `goal.is_complete()`, deadline reached, or controller `Unrecoverable`. `Recoverable` errors are reported via the recorder and otherwise ignored.

**`R: Recorder` is generic, not boxed**: monomorphizes; `NullRecorder::record` is a no-op the optimizer eliminates.

### Concrete arm goals (`arm::goals`)

`ReachPose`, `PickObject`, `PlaceInBin`, `Stack`. All `impl Goal<ArmWorld>`. `Sequence` deferred from v1.

`Stack(A, on=B)` works under gravity-fall: `is_complete` checks that A is `Settled(on: B)` and has been so for ≥ 100 ms. `evaluate` returns 1.0 when complete, else a shaped value based on `(A.bottom_z - B.top_z)` distance.

### End-to-end `#[test]`

```rust
#[test]
fn pid_places_block() {
    let mut world = build_pick_and_place_world();
    let ports = world.attach_standard_arm_ports();
    let controller = controllers::PickPlacePid::new(ports);
    let result = harness::run(
        world, controller, PlaceInBin::new(BLOCK, BIN),
        RunConfig::default()
            .with_deadline(Duration::secs(15))
            .with_seed(42),
    );
    assert!(result.score.value > 0.9, "score {}", result.score.value);
}
```

---

## 7. Visualizer (`viz`)

### Snapshot collection lives on the world

`sim` defines `Primitive`, `SceneSnapshot`, and a `Visualizable` trait:

```rust
pub enum Primitive {
    Sphere  { pose, radius, color },
    Capsule { pose, half_height, radius, color },
    Box     { pose, half_extents, color },
    Line    { from, to, color },
    Label   { pose, text, color },
}
pub struct SceneSnapshot { pub t: Time, pub items: Vec<(EntityId, Primitive)> }

pub trait Visualizable {  // object-safe by construction
    fn append_primitives(&self, out: &mut Vec<(EntityId, Primitive)>);
}
```

`sim` impls `Visualizable` for `Object` and `Fixture`. `arm` impls it for `Arm` (capsules for links, box for gripper).

**Snapshot collection happens on the world** via the `RunnableWorld` trait method, which is the single named integration point:

```rust
pub trait RunnableWorld {
    fn tick(&mut self, dt: Duration) -> Result<(), SimError>;
    fn snapshot(&self) -> SceneSnapshot;
    fn time(&self) -> Time;
}
```

`ArmWorld::snapshot` walks: scene fixtures, scene objects, the arm. Each appends its primitives. New robot kinds extend the same way. **`viz` only sees `Primitive`s and `SceneSnapshot`. It never imports `arm` or any robot-kind crate** — the dependency arrow `viz → sim` holds.

### Transport — out-of-process via `Recorder`

```rust
pub trait Recorder {
    fn record(&mut self, snapshot: &SceneSnapshot);
    fn record_event(&mut self, _event: &ControllerEvent) {}  // optional
}
```

Three impls in `viz`: `NullRecorder` (default, zero overhead), `FileRecorder` (binary or JSON to disk), `RerunRecorder` (translates primitives to `rerun-sdk` log calls).

**Tests stay fully headless by default** — `RunConfig::default()` uses `NullRecorder`. Opt in per-test or via env var.

### `ControllerEvent` (debug aid, behind a feature flag)

```rust
pub enum ControllerEvent {
    PortSend  { port_id: PortId, t: Time, type_name: &'static str },
    PortRecv  { port_id: PortId, t: Time, type_name: &'static str },
    ControllerError { t: Time, err: ControlErrorKind, detail: String },
}
```

Recorded only when the `controller_events` cargo feature is enabled — zero overhead otherwise. When a test fails, replay the recording to see exactly what the controller did per tick.

---

## 8. Extensibility Paths

### 8.1 New robot modalities

Each modality ships its own world type owning a `Scene` + robot-specific state and impls `RunnableWorld`. New modality = new sibling crate; **zero changes** to `sim`, `core`, `harness`, or `viz`.

```rust
// arm    : pub struct ArmWorld    { scene, arm, .. }   impl RunnableWorld
// mobile : pub struct MobileWorld { scene, base, .. }  impl RunnableWorld
// drone  : pub struct DroneWorld  { scene, body, .. }  impl RunnableWorld
```

### 8.2 SNN integration — drift-free fixed-step accumulator

Zero framework changes. The SNN is a `Controller` whose `step` advances internal sim time using **integer arithmetic with an accumulator**:

```rust
pub struct SnnController {
    network: SpikingNetwork,
    inner_dt_ns: i64,
    accumulator_ns: i64,        // residual carried between calls
    last_t: Time,
    /* port endpoints */
}

impl Controller for SnnController {
    fn step(&mut self, t: Time) -> Result<(), ControlError> {
        let dt_ns = (t - self.last_t).nanos();              // i64, exact
        self.accumulator_ns += dt_ns;
        let n = (self.accumulator_ns / self.inner_dt_ns) as usize;
        self.accumulator_ns -= (n as i64) * self.inner_dt_ns;

        let spikes = self.encode_sensor_ports();            // rate/temporal coding
        for _ in 0..n {
            self.network.step(self.inner_dt_ns, &spikes);
        }
        if let Some(cmd) = self.decode_action_neurons() {
            self.actuator_tx.send(cmd);
        }
        self.last_t = t;
        Ok(())
    }
}
```

No float division, no rounding: the residual carries forward exactly. Phase error is bounded by one `inner_dt`; over arbitrary durations it does not accumulate.

### 8.3 Real-hardware port

**Critical layout commitment:** port type definitions (`JointEncoderReading`, `JointVelocityCommand`, etc.) live in `arm`, not `sim`. A future `arm-real` sibling crate shares them.

`arm-real` builds the same `PortTx<T>` / `PortRx<T>` ends — but the producer side reads a real encoder (SPI/CAN/EtherCAT) and the consumer side writes a real motor controller. The `Clock` impl becomes a `MonotonicClock`. **Controller code is unchanged — *if and only if it abides by the behavioral contract* (§9).**

### 8.4 Fault injection (in `sim::faults`)

Channel combinators that are themselves `PortRx<T>` / `PortTx<T>` — fully transparent to the controller:

```rust
let rx = sim::faults::Delay::new(
    sim::faults::DropMessages::new(raw_rx, rate: 0.05, seed: 42),
    latency: Duration::from_millis(2));
let rx = sim::faults::StaleData::new(rx, max_age: Duration::from_millis(50));
```

Lets you stress-test controllers against hardware-style failures inside a `#[test]`, without hardware. **In v1 these are opt-in per port.** A "realistic by default" mode (where `attach_*` automatically wraps with bounded jitter/drop) is a future v2 axis if the HAL contract proves too lenient in practice.

---

## 9. Behavioral Contract for Ports

**This section is load-bearing for the HAL promise.** A controller that abides by this contract will run in sim and on `arm-real` without source changes. A controller that violates it works in sim but is broken on hardware — a category of bug we want to catch by writing the contract down.

### 9.1 The controller MAY assume

- A reading received from `PortRx<T>::latest()` is well-formed (typed values within their declared range).
- A command sent via `PortTx<T>::send()` is delivered to the simulator/hardware unless a `faults` wrapper interposes.
- The injected `Clock::now()` is monotonically non-decreasing across calls within one `step`.
- `step(t)` is called at the framework's nominal tick rate, ±1 tick of jitter.

### 9.2 The controller MUST NOT assume

- A specific cadence of sensor *publishing*. Sensors fire at their declared rates ±1 tick of jitter. **The reading's `sampled_at: Time` field is authoritative**, not a derived schedule.
- Zero latency from `tx.send(cmd)` to physical effect. There is at minimum one tick of latency (the actuator is consumed at the *next* tick boundary). Real hardware adds more.
- That a sensor reading is fresh. Use `(now - reading.sampled_at)` to gauge staleness; reject readings older than your tolerance.
- That `latest()` returns a different value each call. If no new value has arrived, it returns the same value (or `None`).
- That `step` is called at exactly equal intervals. Compute `dt_ns` from `t - last_t`; do not assume a constant.

### 9.3 The framework GUARANTEES

- Determinism of the sim under a fixed seed and a fixed toolchain+target (§10).
- `step(t)` is called from the same thread that constructed the `Controller`; no synchronization needed inside.
- The injected `Clock` is the only source of time. Calling `std::time::Instant::now()` is a controller bug.

### 9.4 Sensor readings carry sample timestamp

All sensor reading types in `arm::ports` (and in any future robot-kind crate) **must** include a `sampled_at: Time` field. This is a hard-coded convention enforced by code review (and, where convenient, by a marker trait `SensorReading { fn sampled_at(&self) -> Time; }`).

### 9.5 Controller threading and external resources

The single-threading constraint (§10.3) applies to the **framework** — `harness::run`'s tick loop, sim integration, port operations on the tick thread. It does **not** restrict what the controller does inside `step()`.

**The controller MAY:**
- Spawn worker threads at construction time or lazily on first `step()`.
- Maintain a long-lived thread pool, CUDA context, Vulkan compute pipeline, network connection to a remote inference server, or similar.
- Inside `step()`, submit work to those threads / GPU / external services, **block** until that work completes, read the results, and return.

This makes the SNN-on-GPU pattern (and similar) a first-class use case:

```rust
impl Controller for SnnGpuController {
    fn step(&mut self, t: Time) -> Result<(), ControlError> {
        let sensor_state = self.read_sensors();                 // single-threaded port reads
        self.gpu_buffer.copy_from(&self.encode(&sensor_state)); // upload
        for _ in 0..self.n_microsteps_per_outer_tick {
            unsafe { snn_kernel<<<...>>>(self.gpu_buffer); }    // launch
        }
        self.cuda_stream.synchronize();                         // BLOCK
        let actions = self.gpu_buffer.copy_to_host();
        self.write_actuator_commands(&actions);                 // single-threaded port writes
        Ok(())
    }
}
```

**Required of the controller (regardless of internal threading):**
- `step()` must return synchronously, with all output port writes finalized before return.
- Outputs must be deterministic w.r.t. inputs, within the toolchain+target determinism scope (§10.1). For GPU code, this means deterministic kernels (e.g., avoid float `atomicAdd`, enable CuDNN deterministic mode, use `torch.use_deterministic_algorithms(True)` for PyTorch). If bit-identical reproducibility is not required, statistical reproducibility is fine.

**Practical constraint — port endpoints are not `Send`/`Sync`.**

`PortTx<T>` and `PortRx<T>` are backed by `Rc<RefCell<...>>` (zero-cost single-consumer). They cannot be moved or shared across threads. So a worker thread inside the controller cannot directly read sensor ports or write actuator ports — the controller must marshal data across the thread boundary using its own internal channels (e.g., `std::sync::mpsc`, `crossbeam`, or shared `Arc<Mutex<...>>`).

If a future use case requires writing to actuator ports from a long-lived background thread (e.g., a continuously-running SNN that asynchronously emits spikes between `step()` calls), the design has a clean v2 axis: introduce `port_sync<T>() -> (PortTxSync<T>, PortRxSync<T>)` backed by `Arc<Mutex<...>>` or a lock-free SPSC queue, opt-in per-port. Determinism for such a setup requires a deterministic schedule for which producer-thread events are visible to which sim tick — solvable but non-trivial. Out of scope for v1.

### 9.6 What v1 does *not* yet do (deferred to v2)

- Inject jitter, drop, or latency by default. Today the sim is "clean" — meaning a controller can violate §9.2 and still pass tests in sim. **The user is on their honor** (or, more practically, must opt into `sim::faults` wrappers to exercise robustness).
- Audit controllers for §9.2 compliance (e.g., a static-analysis lint or a runtime "trickster mode" that enables fault wrappers globally during `cargo test`).
- Thread-safe port variants (`port_sync<T>`) for controllers with long-lived background threads writing to actuator ports — see §9.5 last paragraph.

The decision to defer §9.6 is deliberate: lightweight contract first, enforcement later if and when controller-vs-hardware drift becomes a real problem.

---

## 10. Determinism Audit

### 10.1 What "deterministic" means here

Same scenario + same seed → bit-identical `RunResult` and snapshot stream **on a fixed toolchain version + target triple**.

Cross-platform / cross-toolchain bit-identity is *not* claimed because `f32` transcendentals (`sin`, `cos` for FK) are not bit-identical across libm versions / `-ffast-math` settings / SIMD widening. If cross-platform reproducibility ever matters, the path is a software softfloat or fixed-point integration of FK — out of scope for v1.

### 10.2 Forbidden in `core` and `sim` tick path (lint as a code-review invariant)

| Forbidden | Reason | Replace with |
|---|---|---|
| `std::collections::HashMap` (during tick) | Iteration order is randomized | `BTreeMap` or `IndexMap` |
| `std::time::Instant::now()` | Wall clock | Inject `Clock` |
| `std::thread::spawn` / `rayon::*` | Schedule nondeterminism | Single-threaded only |
| `rand::thread_rng()` | Implicit global RNG | Pass an explicitly seeded `StdRng` |
| `f32::powi` / fast-math attributes | Cross-toolchain divergence | Plain integer ops where possible |

These are documented in the workspace `README` and checked by reviewer eyeballs; a future `clippy` config can mechanize.

### 10.3 Test-time isolation

`harness::run` is **single-threaded on the calling thread**. No background threads. No global state. Therefore: panic-safety is automatic (a panicking test does not strand background workers), `cargo test`'s parallel test execution is safe (each `run` is wholly contained in its test thread), and there is no shared resource that needs locking.

If a `Recorder` ever wants a background writer (e.g., `StreamRecorder` flushing to a socket), the writer is owned by the recorder and joined/dropped on `Drop` — its lifetime is bounded by the `RunResult` returned to the test.

---

## 11. Out of Scope for v1

- Physics engine; rigid-body dynamics; contact forces other than vertical-stack support resolution.
- Lateral motion of free objects (no friction, no sliding).
- Joint torque commands (no inertia model).
- Mesh geometry (primitives only).
- Multi-robot scenes.
- Mobile robot, drone, or other non-arm modalities (the architecture supports them; we just don't ship them).
- `arm-real` adapter or any real-hardware integration.
- Data-defined scenarios (YAML/RON/JSON). Code-defined only.
- Multi-component score *aggregation policy* (the `breakdown` field exists; how to combine is per-goal).
- Realistic-by-default sim (jitter/drop/latency injected automatically). Opt-in via `sim::faults`.
- Composite goal types (`Sequence`, `Choice`, `Parallel`).
- Cross-toolchain / cross-platform bit-identical determinism.

---

## 12. Suggested Implementation Order

For a future writing-plans session to expand into concrete steps. Roughly:

1. **`core` crate.** `Time`, `Duration`, `Clock`, `port<T>`, `PortTx`/`PortRx`, `PortId`, `Controller`, `ControlError`, `WorldView`, `Goal`, `Score`. No tests yet, just trait surface.
2. **`sim` crate (foundations).** `Scene`, `Object`, `Fixture`, `RateScheduler`, `RunnableWorld` trait, `Primitive`, `SceneSnapshot`, `Visualizable`, `Recorder`, `NullRecorder`. Sim-side `SimClock`. No gravity yet.
3. **`arm` crate (core).** `ArmSpec`, `ArmState`, FK, port types (`JointEncoderReading`, `JointVelocityCommand`, `GripperCommand`), `ArmWorld` skeleton, `ArmWorld::snapshot()`. Grasp model (without gravity).
4. **`harness` crate.** `RunConfig`, `RunResult`, `Termination`, the `run` function (generic over `R: Recorder`). Single-threaded loop.
5. **First end-to-end test (no gravity).** `ReachPose` goal + a hand-rolled PD controller. Validates the whole stack.
6. **Add gravity-fall to `sim`.** Settling logic, support resolution, per-tick integration. New tests: object falls onto fixture; object falls onto another settled object.
7. **`PickObject`, `PlaceInBin`, `Stack` goals.** Hand-rolled controller passes each.
8. **`viz`: `RerunRecorder`.** Smoke-test by enabling on existing tests.
9. **`sim::faults`: `Delay`, `DropMessages`, `StaleData`, `GaussianNoise`.** Add a robustness test.

Steps 1–5 are the minimum coherent slice without gravity. Step 6 makes `Stack` and `PlaceInBin` honest. 8 and 9 are independently shippable.

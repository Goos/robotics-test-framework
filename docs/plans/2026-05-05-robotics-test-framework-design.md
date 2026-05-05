# Robotics Test Framework — Design

**Date:** 2026-05-05
**Status:** Validated through brainstorming. Ready for implementation planning.
**Scope:** Personal project. No team, no public API, no external users.

---

## 1. Problem & Goals

Build a Rust framework for simulating simple robotics tasks and testing controller "business logic" against deterministic, mockable physical layers.

**Functional goals:**
- Simulate at least the **robotic arm / manipulator** modality, with the architecture cleanly extensible to mobile robots, drones, etc.
- Run a controller against a simulated world, score how well it achieved a defined goal, return a result.
- Tests are **code-defined** (Rust `#[test]`), discovered and run by `cargo test`.

**Non-functional goals (the load-bearing ones — these shaped almost every decision):**
- **Determinism.** Same scenario + same seed → bit-identical outcome. Required for trustworthy assertions and bisecting controller regressions.
- **HAL-style abstraction.** The controller (system under test) cannot see the simulator. Same controller code can later run against real hardware.
- **Mockability.** The framework's primitives (sensor channels, actuator channels, clock) can all be substituted, including by fault-injecting wrappers that simulate hardware-style misbehavior.
- **No premature flexibility.** Anywhere a future need is *anticipated* but not *current*, the design leaves a clean extension point but does not build for it.

**Eventual intent (informs design but is not v1 scope):**
- The business logic will likely be implemented as a **spiking neural network (SNN)** running at its own internal update rate, decoupled from the simulator's tick rate.
- The same controllers should eventually run against **real hardware** with no source-level changes — only the adapter (sensor drivers, actuator backends, clock) differs.

---

## 2. Key Decisions

| Decision | Choice | Why |
|---|---|---|
| Language | Rust | Top preference; first-class channels/traits/lifetimes are exactly what the abstraction needs. |
| Simulation fidelity | Pure kinematic (v1) | Goals are testability/determinism, not realism. Physics engine is a v2+ extension. |
| Controller programming model | Typed-channel "ports" with controller owning its own clock | Subsumes Gym-style step(obs)→action as a special case; accommodates SNNs running at independent rates; maps to real hardware HAL cleanly. |
| Test format | Code-defined (`#[test]`) | Lowest infra overhead; full type system for scenario construction; cargo's runner for free. |
| Crate organization | Workspace, ~6 crates | Compiler physically prevents abstraction leaks. Cheap upfront. |
| Visualizer | Out-of-process via `Recorder` trait; **rerun.io** as default backend | Tests stay headless. Visualizer never depends on robot-kind crates — `viz` only sees `Primitive`s. |
| Released-object behavior | Stay put (pure kinematic). Goals score release-time conditions. | Keeps v1 truly free of dynamics. Optional gravity-fall is a clean v2 extension. |
| Actuator vocabulary (v1) | Joint velocity commands only | Simplest, integrates trivially. Position/torque are easy to add later. |
| Math dep | `nalgebra` | Standard in Rust robotics ecosystem. |

---

## 3. Architecture

### Crate layout (Cargo workspace)

```
robotics-test-framework/
├── core/         — traits + types only. The contract.
├── sim/          — kinematic simulator. Owns world state, time, port registry.
├── arm/          — manipulator-specific: kinematic chain, FK, gripper, arm port types.
├── harness/      — test runner: build sim, run controller, score, return result.
├── faults/       — channel wrappers (drop, delay, stale, reorder).
├── viz/          — visualizer (thin rerun-sdk wrapper).
└── examples/     — your scenarios as `#[test]`s.
```

### Dependency arrows (strict, one-way)

```
arm     → core
sim     → core
arm     → sim
harness → sim, core
faults  → core
viz     → sim                 (NEVER → arm; only sees Primitives)
```

`core` depends on **nothing**. It defines `Controller`, `Port<T>`, `Clock`, `Scenario`, `Goal`, and supporting types. No knowledge of arms, kinematics, scene geometry, or simulators.

### Where the user's controller code lives

Controllers depend only on `core` + the specific port types they consume (e.g., `arm::ports::JointEncoderReading`). They never import `sim` or `harness`. **That's the HAL boundary**, enforced by the workspace.

---

## 4. Core Abstractions (`core`)

### Time & Clock

```rust
pub struct Time(u64);  // nanos since scenario start
pub trait Clock { fn now(&self) -> Time; }
```

Clock is injected at construction. Sim provides a deterministic `SimClock`; real hardware provides a `MonotonicClock`. The controller never reads a global wall clock.

### Typed Ports

```rust
pub enum PortKind {
    Fifo      { capacity: usize },  // events; no loss
    LatestWins,                      // state-like; only freshest survives
    Buffered  { capacity: usize },   // batch reads
}

pub fn port<T>(kind: PortKind) -> (PortTx<T>, PortRx<T>);

impl<T> PortTx<T> { pub fn send(&self, v: T); }
impl<T> PortRx<T> {
    pub fn try_recv(&self) -> Option<T>;
    pub fn drain(&mut self, into: &mut Vec<T>);
    pub fn latest(&self) -> Option<T> where T: Clone;
}
```

**Only the creator of a port chooses its kind.** Both endpoints see a uniform read/write API. PortKind is policy; the API is mechanism. Channel kinds change retention/delivery, never ordering within a stream.

### Controller (the system under test)

```rust
pub trait Controller {
    fn step(&mut self, t: Time) -> Result<(), ControlError>;
}
```

Note what's **missing**: no observation, no action return. The controller owns its port endpoints as fields (handed in at construction) and reads/writes them inside `step`. `step` is just "framework telling you a tick has elapsed; do whatever you do."

A Gym-style `step(obs) → action` controller is just one configuration of the more general thing: register one `LatestWins` port per sensor field, one `LatestWins` port per actuator, read inputs at the top of `step`, write outputs at the bottom.

For an SNN: `step` advances the SNN's internal clock by N microsteps and reads/writes channels at whatever cadence the SNN wants — see §8.

### World view & scoring

```rust
pub trait WorldView {}  // marker; specific worlds extend with own accessors

pub struct Score(pub f64);  // convention: [0.0, 1.0]

pub trait Goal<W: WorldView> {
    fn tick(&mut self, t: Time, world: &W) {}        // optional: track best-so-far
    fn is_complete(&self, world: &W) -> bool { false } // optional: early termination
    fn evaluate(&self, world: &W) -> Score;            // required: final score
}
```

`tick` lets a goal accumulate state (best-distance-achieved, time-in-zone — useful for shaped scoring). `is_complete` short-circuits the run on success. `evaluate` produces the final number.

**Key invariant:** `Goal` *can* see the world — it is test scaffolding, not business logic. `Controller` cannot.

---

## 5. Kinematic Simulator (`sim` + `arm`)

### Scene model (`sim`)

Robot-agnostic. Three flat collections:

- **Fixtures** — immutable geometry (ground, table, bin). Defines layout for grasp/score predicates.
- **Objects** — mutable rigid bodies: `id`, pose (`Isometry3<f32>`), primitive shape (sphere / AABB / cylinder), `graspable: bool` flag.
- **Robots** — provided by robot-kind crates (e.g., `arm`).

No physics engine. Primitives only — no meshes.

### Kinematic arm (`arm`)

```rust
pub struct ArmSpec {
    pub joints: Vec<JointSpec>,            // revolute/prismatic, axis, limits
    pub link_offsets: Vec<Isometry3<f32>>, // parent → child transforms
    pub gripper: GripperSpec,              // proximity threshold, max grasp size
}

pub struct ArmState {
    pub q: Vec<f32>,                       // joint positions
    pub q_dot: Vec<f32>,                   // joint velocities
    pub gripper_closed: bool,
    pub grasped: Option<ObjectId>,         // currently held object, if any
}
```

Forward kinematics is a chain of transform multiplies — ~50 lines, zero extra deps. End-effector pose = last link transform.

### Grasping (kinematic, no contact forces)

Each tick after FK:
- Gripper closing + nothing held + a `graspable` object's pose within `gripper.proximity_threshold` of EE → attach.
- While grasped, the object pose is rigidly slaved to EE pose.
- Gripper opens → detach. Released objects stay put.

Tasks like "put in bin" score the *condition at release time* (EE over bin, gripper open, correct object), not a fall.

### Tick loop

Fixed step, default 1000 Hz. Per tick at time `t`:

1. Sim pushes sensor readings reflecting world state at `t` (each sensor only fires on its scheduled tick).
2. Framework calls `controller.step(t)` — reads sensors, writes actuator commands.
3. Sim drains actuator port queues; latest command per port wins for that tick.
4. Sim integrates `q += q_dot * dt`, recomputes FK, updates grasp state. World now at `t + dt`.

### Multi-rate sensors

Each attached sensor has a `RateHz`; sim keeps a per-sensor tick counter and only pushes when scheduled. Camera at 30 Hz, encoder at 1 kHz, force sensor at 500 Hz — all coexist by construction.

### Determinism guarantees

- Sim-side ops execute in stable order (sorted by port id) each tick.
- Within a port, FIFO.
- Any randomness (sensor noise, etc.) takes a seed.
- Identical seed + identical inputs → bit-identical output.

---

## 6. Test Harness & Scoring (`harness`)

### Setup is two-phase

Build world → wire controller ports → run harness. Forced by ownership: port endpoints come from the world, so the controller can't exist until the world does.

### World builder (`arm`)

```rust
let mut world = arm::ArmWorldBuilder::new()
    .with_arm(franka_panda_spec())
    .with_object(Object::block().at([0.5, 0.0, 0.05]).graspable())
    .with_fixture(Fixture::bin().at([0.0, 0.5, 0.0]).extents([0.2, 0.2, 0.1]))
    .build();
```

### Port wiring (explicit; helpers can come later)

```rust
let encoder_rx  = world.attach_joint_encoder_sensor(RateHz(1000), PortKind::LatestWins);
let velocity_tx = world.attach_joint_velocity_actuator();
let gripper_tx  = world.attach_gripper_actuator();
let controller  = MyController::new(encoder_rx, velocity_tx, gripper_tx);
```

### The run loop

```rust
pub fn run<W, C, G>(world: W, controller: C, goal: G, cfg: RunConfig) -> RunResult
where W: RunnableWorld, C: Controller, G: Goal<W::View>;

pub struct RunConfig {
    pub tick_rate: RateHz,         // default 1000 Hz
    pub deadline: Time,
    pub seed: u64,
    pub recorder: Box<dyn Recorder>, // default NullRecorder
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

Loop terminates on `goal.is_complete()`, deadline reached, or controller `Err`.

### Concrete arm-side goals (`arm::goals`)

`ReachPose`, `PickObject`, `PlaceInBin`, `Stack`, `Sequence([Pick, Place])` (composite). Each `impl Goal<ArmWorldView>`. Custom goals are just trait impls — no framework changes.

### End-to-end `#[test]`

```rust
#[test]
fn pid_places_block() {
    let mut world = build_pick_and_place_world();
    let controller = controllers::PickPlacePid::new(world.attach_standard_arm_ports());
    let result = harness::run(
        world, controller, PlaceInBin::new(BLOCK, BIN),
        RunConfig::default().with_deadline(Time::secs(15.0)).with_seed(42),
    );
    assert!(result.score.0 > 0.9, "score {}", result.score.0);
}
```

---

## 7. Visualizer (`viz`)

### Pattern: scene snapshot via introspection trait

`sim` defines a small primitive vocabulary and a snapshot struct:

```rust
pub enum Primitive {
    Sphere  { pose, radius, color },
    Capsule { pose, half_height, radius, color },   // joint links
    Box     { pose, half_extents, color },          // gripper, bin, table
    Line    { from, to, color },                    // axes, debug arrows
    Label   { pose, text, color },                  // ids, scores
}

pub struct SceneSnapshot {
    pub t: Time,
    pub items: Vec<(EntityId, Primitive)>,
}

pub trait Visualizable {
    fn append_primitives(&self, out: &mut Vec<(EntityId, Primitive)>);
}
```

`sim` impls `Visualizable` for `Object` and `Fixture`. `arm` impls it for the arm (capsules for links, box for gripper). The visualizer sees only `Primitive`s — **never imports `arm` or any robot-kind crate**.

### Transport — out of process via `Recorder`

```rust
pub trait Recorder {
    fn record(&mut self, snapshot: &SceneSnapshot);
}
```

Three impls:
- **`NullRecorder`** — default, zero overhead.
- **`FileRecorder`** — binary or JSON to disk; CI artifacts and replay.
- **`StreamRecorder`** — line-delimited over a Unix socket / TCP for live viewing.

Tests stay fully headless by default. Opt in per-test or via env var.

### Default backend: rerun.io

Ship a `RerunRecorder` impl that converts `Primitive`s to `rerun-sdk` log calls. The actual viewer is the rerun process, which provides timeline scrubbing, multiple synced views, plot panels, and replay built in. This skips weeks of UI work and yields a better tool than would otherwise be built.

---

## 8. Extensibility Paths

Each of the four anticipated future needs grows independently because `core` knows nothing and `sim` knows scenes-not-robots.

### 8.1 New robot modalities

`sim` provides a robot-agnostic `Scene` plus a `RunnableWorld` trait that `harness::run` is generic over. Each modality ships its own world type owning a `Scene` + robot-specific state and impls `RunnableWorld`. New modality = new sibling crate; **zero changes** to `sim`, `core`, `harness`, or `viz`.

```rust
// arm    : pub struct ArmWorld    { scene: Scene, arm: Arm, .. }     impl RunnableWorld
// mobile : pub struct MobileWorld { scene: Scene, base: .., .. }     impl RunnableWorld
// drone  : pub struct DroneWorld  { scene: Scene, body: .., .. }     impl RunnableWorld
```

### 8.2 SNN integration

Zero framework changes. The SNN is a `Controller` whose `step` runs N internal microsteps:

```rust
fn step(&mut self, t: Time) -> Result<(), ControlError> {
    let n = ((t - self.last_t) / self.inner_dt).round() as usize;
    let spikes = self.encode_sensor_ports();        // rate/temporal coding
    for _ in 0..n { self.network.step(self.inner_dt, &spikes); }
    self.actuator_tx.send(self.decode_action_neurons());
    self.last_t = t;
    Ok(())
}
```

The SNN's internal clock is its own concern; the framework only sees `Controller`.

### 8.3 Real-hardware port

**Critical layout commitment:** port type definitions (`JointEncoderReading`, `JointVelocityCommand`, etc.) live in `arm`, **not** `sim`. That way a future `arm-real` sibling crate shares them with `arm`'s sim impl.

`arm-real` builds the same `PortTx<T>` / `PortRx<T>` ends — but the producer side reads a real encoder (SPI/CAN/EtherCAT) and the consumer side writes a real motor controller. The `Clock` impl becomes a `MonotonicClock` over the OS. **Controller code is unchanged.** That's the HAL promise made concrete.

### 8.4 Fault injection

`faults` exposes channel combinators that are themselves `PortRx<T>` / `PortTx<T>` — fully transparent to the controller:

```rust
let rx = faults::Delay::new(
    faults::DropMessages::new(raw_rx, rate: 0.05, seed: 42),
    latency: Duration::from_millis(2));
let rx = faults::StaleData::new(rx, max_age: Duration::from_millis(50));
```

Deterministic given seed. Lets you stress-test controllers against hardware-style failures inside a `#[test]`, without hardware.

---

## 9. Out of Scope for v1

Listed explicitly so they aren't sneaked in.

- Physics engine; rigid-body dynamics; contact forces.
- Gravity-fall on released objects (planned v2 feature flag).
- Joint torque commands (no inertia model in v1).
- Mesh geometry (primitives only).
- Multi-robot scenes.
- Mobile robot, drone, or other non-arm modalities (the architecture supports them; we just don't ship them).
- `arm-real` adapter or any real-hardware integration.
- Data-defined scenarios (YAML/RON/JSON). Code-defined only.
- Scenario discovery or test runner beyond what `cargo test` provides.
- Multi-component score breakdown (start with `Score(f64)`).
- Framework-managed RL training loops or policy optimization.

---

## 10. Suggested Implementation Order

For a future writing-plans session to expand into concrete steps. Roughly:

1. `core` crate: `Time`, `Clock`, `PortKind`, `port<T>`, `PortTx`/`PortRx`, `Controller`, `WorldView`, `Goal`, `Score`. No tests yet, just the trait surface.
2. `sim` crate: `Scene`, `Object`, `Fixture`, `Primitive`, `SceneSnapshot`, `Visualizable`, `Recorder`, `NullRecorder`, port registry, sim-side `Clock`, deterministic tick scheduler.
3. `arm` crate: `ArmSpec`, `ArmState`, FK, grasp model, port types (encoder reading, velocity command, gripper command), `ArmWorld` impl of `RunnableWorld`, `ArmWorldView`.
4. `harness` crate: `RunConfig`, `RunResult`, `Termination`, the `run` function.
5. `arm::goals`: `ReachPose`, `PickObject`, `PlaceInBin`. Just enough for the first end-to-end test.
6. First `#[test]` end-to-end: hand-rolled PID controller picks a block. Validates the whole stack.
7. `viz` crate: `RerunRecorder`. Smoke-test by enabling on the existing test.
8. `faults` crate: `Delay`, `DropMessages`, `StaleData`. Add a robustness test using one of them.

Steps 1–6 are the minimum coherent slice. 7–8 are independently shippable extensions.

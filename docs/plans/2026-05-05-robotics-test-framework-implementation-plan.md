# Plan: Robotics Test Framework — v1 Implementation

**Goal**: Build a Rust workspace implementing the v1 design ([`2026-05-05-robotics-test-framework-design-v2.md`](2026-05-05-robotics-test-framework-design-v2.md)) — a kinematic-with-gravity robotics simulator with HAL-style port abstraction, deterministic test harness, code-defined `#[test]` scenarios, and out-of-process visualization. End state: a passing `#[test]` in which a hand-rolled PD/PID controller picks up a block and places it in a bin, scored ≥ 0.9.

**Architecture**: 5-crate Cargo workspace (`core`, `sim`, `arm`, `harness`, `viz`) with strict one-way dependency arrows enforced by the workspace. `core` depends on nothing. The controller-under-test depends only on `core` + `arm` port type definitions, never on `sim` or `harness` — the HAL boundary made structural.

**Tech Stack**: Rust 2021, `nalgebra` (math), `smallvec` (Score breakdown), `rand` + `rand_pcg` (deterministic RNG), `rerun` SDK (visualizer backend, optional). No async, no threads in the tick path.

---

## Conventions

- **Test commands** — every test step uses `cargo test -p <crate> <test_name> -- --exact` from the workspace root, or `cargo test --workspace` for the full suite. Run from `/Users/goos/AAI/robotics-test-framework`.
- **Commit messages** — `[<crate>] Step N.M: <description>`. Use git (`git add` + `git commit`); no `--no-verify`.
- **TDD discipline** — every implementation step is preceded by a failing test that targets exactly the new behavior; verify failure before writing the implementation, verify pass after.
- **Per-step time budget** — 2–5 minutes of focused work. If a step balloons, split it.
- **Determinism invariants** — `BTreeMap` over `HashMap` in tick path; no `std::time::Instant::now()` in `core`/`sim`; no thread spawning; explicit seeded RNG. (Design §10.)
- **API spec** — when a step says "implement <Type>", consult the corresponding section of the design v2 doc for the canonical signature; this plan repeats only the load-bearing details.

---

## Task Dependencies

| Phase | Steps | Depends On | Can Parallelize Within Phase? |
|-------|-------|------------|-------------------------------|
| 0. Workspace setup | 0.1–0.3 | — | No (serial) |
| 1. `core` crate | 1.1–1.10 | Phase 0 | Mostly no (each builds on prior types) |
| 2. `sim` foundations (no gravity) | 2.1–2.10 | Phase 1 | No |
| 3. `arm` crate | 3.1–3.12 | Phase 2 | No |
| 4. `harness` crate | 4.1–4.5 | Phase 3 | No |
| 5. First end-to-end test (no gravity) | 5.1–5.4 | Phase 4 | No |
| 6. Gravity-fall in `sim` | 6.1–6.7 | Phase 5 | No |
| 7. Goals: `PickObject`, `PlaceInBin`, `Stack` | 7.1–7.4 | Phase 6 | Yes (independent goals) |
| 8. `viz` crate (rerun) | 8.1–8.4 | Phase 5 | Independent of 6/7 |
| 9. `sim::faults` | 9.1–9.5 | Phase 2 | Independent of 6/7/8 |

Phases 8 and 9 can run in parallel with 6 and 7 once Phase 5 is done.

## Phase 0: Workspace setup

### Step 0.1: Initialize Cargo workspace

**Files**: `Cargo.toml` (new, workspace root), `core/Cargo.toml`, `sim/Cargo.toml`, `arm/Cargo.toml`, `harness/Cargo.toml`, `viz/Cargo.toml`, `core/src/lib.rs`, `sim/src/lib.rs`, `arm/src/lib.rs`, `harness/src/lib.rs`, `viz/src/lib.rs`.

```bash
cargo new --lib core
cargo new --lib sim
cargo new --lib arm
cargo new --lib harness
cargo new --lib viz
```

Replace the auto-generated workspace-root `Cargo.toml` with:

```toml
[workspace]
resolver = "2"
members = ["core", "sim", "arm", "harness", "viz"]

[workspace.package]
edition = "2021"
license = "MIT"

[workspace.dependencies]
nalgebra = "0.33"
smallvec = "1"
rand = { version = "0.8", default-features = false }
rand_pcg = "0.3"
rerun = { version = "0.21", default-features = false, features = ["sdk"] }
```

In each crate's `Cargo.toml`, set `edition.workspace = true`. Do not add deps yet — added per crate in their own steps.

**Verify**: `cargo build --workspace` compiles five empty libs.

```bash
cargo build --workspace
git add Cargo.toml */Cargo.toml */src/lib.rs && git commit -m "[workspace] Step 0.1: Init Cargo workspace with 5 empty crates"
```

### Step 0.2: Configure dependency arrows

**Files**: `core/Cargo.toml`, `sim/Cargo.toml`, `arm/Cargo.toml`, `harness/Cargo.toml`, `viz/Cargo.toml`.

Wire the dependency arrows from design §3 (workspace deps via path):

- `core/Cargo.toml`: no internal deps.
- `sim/Cargo.toml`: `core = { path = "../core" }`, plus workspace dep `nalgebra`, `rand`, `rand_pcg`.
- `arm/Cargo.toml`: `core = { path = "../core" }`, `sim = { path = "../sim" }`, plus `nalgebra`.
- `harness/Cargo.toml`: `core = { path = "../core" }`, `sim = { path = "../sim" }`.
- `viz/Cargo.toml`: `sim = { path = "../sim" }`, plus workspace dep `rerun` (optional feature).

**Verify**: `cargo build --workspace` still compiles.

```bash
cargo build --workspace
git add */Cargo.toml && git commit -m "[workspace] Step 0.2: Wire crate dependency arrows"
```

### Step 0.3: Add a baseline `cargo test --workspace` smoke test

**File**: `core/src/lib.rs`.

Add one trivial passing test in `core` to confirm `cargo test` is wired up:

```rust
#[cfg(test)]
mod tests {
    #[test]
    fn workspace_smoke() {
        assert_eq!(2 + 2, 4);
    }
}
```

**Verify**:

```bash
cargo test --workspace
# expect 1 test passed (in core)
git add core/src/lib.rs && git commit -m "[core] Step 0.3: Smoke test for workspace test runner"
```

---

## Phase 1: `core` crate

Each step in this phase: add a module file, write a failing test, run it, write the impl, re-run, commit. Re-export every new public type from `core/src/lib.rs` as part of the same step.

### Step 1.1: `Time` and `Duration` (signed nanos, well-defined arithmetic)

**File**: `core/src/time.rs` (new); update `core/src/lib.rs` with `pub mod time;`.

Failing test (in `core/src/time.rs`):

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duration_from_secs_and_back() {
        let d = Duration::from_secs(2);
        assert_eq!(d.as_nanos(), 2_000_000_000);
        assert_eq!(d.as_secs_f64(), 2.0);
    }

    #[test]
    fn time_subtraction_yields_signed_duration() {
        let t0 = Time::from_nanos(1_000);
        let t1 = Time::from_nanos(500);
        assert_eq!((t0 - t1).as_nanos(),  500);
        assert_eq!((t1 - t0).as_nanos(), -500);
    }

    #[test]
    fn time_plus_duration() {
        let t = Time::from_nanos(100) + Duration::from_nanos(50);
        assert_eq!(t.as_nanos(), 150);
    }
}
```

Run: `cargo test -p core time::tests` → expect compile failure.

Implement: `Time(i64)` and `Duration(i64)` newtypes in nanos, with `Sub`, `Add<Duration>`, `from_nanos`, `from_secs`, `from_millis`, `as_nanos`, `as_secs_f64`. Per design §4 and §10. Both `#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]`.

Re-run: tests pass. Commit:

```bash
git add core/src/time.rs core/src/lib.rs && git commit -m "[core] Step 1.1: Time + Duration with signed nanos"
```

### Step 1.2: `Clock` trait + `FakeClock` test helper

**File**: `core/src/clock.rs` (new); `pub mod clock;` in `lib.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::Time;

    #[test]
    fn fake_clock_advances_under_test_control() {
        let clock = FakeClock::new(Time::from_nanos(0));
        assert_eq!(clock.now(), Time::from_nanos(0));
        clock.advance(crate::time::Duration::from_millis(5));
        assert_eq!(clock.now(), Time::from_millis(5));
    }
}
```

Implement: `pub trait Clock { fn now(&self) -> Time; }`, plus `pub struct FakeClock { ... }` using `Cell<Time>` (single-threaded, exactly what tests need). `FakeClock::new`, `FakeClock::advance`, `Clock for FakeClock`.

Run: `cargo test -p core clock::tests`. Commit:

```bash
git add core/src/clock.rs core/src/lib.rs && git commit -m "[core] Step 1.2: Clock trait + FakeClock"
```

### Step 1.3: `PortId` newtype

**File**: `core/src/port_id.rs` (new); export from `lib.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn port_ids_compare_and_sort() {
        assert!(PortId(1) < PortId(2));
        let mut ids = vec![PortId(3), PortId(1), PortId(2)];
        ids.sort();
        assert_eq!(ids, vec![PortId(1), PortId(2), PortId(3)]);
    }
}
```

Implement: `pub struct PortId(pub u32);` with `#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]`.

Commit:

```bash
git add core/src/port_id.rs core/src/lib.rs && git commit -m "[core] Step 1.3: PortId newtype"
```

### Step 1.4: `port<T>()` + `PortTx` + `PortRx` (LatestWins, single-slot)

**File**: `core/src/port.rs` (new); export from `lib.rs`.

Failing test (covers send-overwrites and latest-vs-take semantics):

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn send_overwrites_then_latest_is_non_destructive() {
        let (tx, rx) = port::<i32>();
        tx.send(1);
        tx.send(2);                       // overwrites 1
        assert_eq!(rx.latest(), Some(2)); // peek
        assert_eq!(rx.latest(), Some(2)); // still there
    }

    #[test]
    fn take_clears_the_slot() {
        let (tx, mut rx) = port::<i32>();
        tx.send(7);
        assert_eq!(rx.take(), Some(7));
        assert_eq!(rx.take(), None);
        assert_eq!(rx.latest(), None);
    }

    #[test]
    fn empty_port_returns_none() {
        let (_tx, rx) = port::<i32>();
        assert_eq!(rx.latest(), None);
    }
}
```

Implement: single-threaded single-slot using `Rc<RefCell<Option<T>>>` (no `Send`/`Sync` cross-thread sharing — design §10 single-threaded guarantee makes this fine). `T: Clone` for `latest()`. Per design §4.

Commit:

```bash
git add core/src/port.rs core/src/lib.rs && git commit -m "[core] Step 1.4: port<T>() + PortTx + PortRx (LatestWins)"
```

### Step 1.5: `Controller` trait + `ControlError`

**File**: `core/src/controller.rs` (new); export from `lib.rs`.

Failing test (a trivial impl that asserts step is callable):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::Time;

    struct Counter { ticks: u32 }
    impl Controller for Counter {
        fn step(&mut self, _t: Time) -> Result<(), ControlError> {
            self.ticks += 1;
            Ok(())
        }
    }

    #[test]
    fn step_increments_state() {
        let mut c = Counter { ticks: 0 };
        c.step(Time::from_nanos(0)).unwrap();
        c.step(Time::from_nanos(1)).unwrap();
        assert_eq!(c.ticks, 2);
    }

    #[test]
    fn unrecoverable_error_propagates() {
        struct Err1;
        impl Controller for Err1 {
            fn step(&mut self, _t: Time) -> Result<(), ControlError> {
                Err(ControlError { kind: ControlErrorKind::Unrecoverable, detail: "bad".into() })
            }
        }
        assert!(Err1.step(Time::from_nanos(0)).is_err());
    }
}
```

Implement per design §4: `Controller` trait, `ControlError { kind, detail: Cow<'static, str> }`, `ControlErrorKind::{Recoverable, Unrecoverable}`.

Commit:

```bash
git add core/src/controller.rs core/src/lib.rs && git commit -m "[core] Step 1.5: Controller trait + ControlError"
```

### Step 1.6: `WorldView` marker trait

**File**: `core/src/world_view.rs` (new); export from `lib.rs`.

Failing test (just type-level):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    struct Empty;
    impl WorldView for Empty {}

    #[test]
    fn marker_trait_is_implementable() {
        fn takes<W: WorldView>(_: &W) {}
        takes(&Empty);
    }
}
```

Implement: `pub trait WorldView {}`. One line; the marker.

Commit:

```bash
git add core/src/world_view.rs core/src/lib.rs && git commit -m "[core] Step 1.6: WorldView marker trait"
```

### Step 1.7: `Score` with optional breakdown

**File**: `core/src/score.rs` (new); export from `lib.rs`. Add `smallvec` dep to `core/Cargo.toml`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_score_is_zero_no_breakdown() {
        let s = Score::default();
        assert_eq!(s.value, 0.0);
        assert!(s.breakdown.is_empty());
    }

    #[test]
    fn score_with_breakdown_round_trips() {
        let s = Score::new(0.85)
            .with_component("accuracy", 0.9)
            .with_component("smoothness", 0.8);
        assert_eq!(s.value, 0.85);
        assert_eq!(s.breakdown.len(), 2);
        assert_eq!(s.breakdown[0], ("accuracy", 0.9));
    }
}
```

Implement per design §4: `pub struct Score { pub value: f64, pub breakdown: SmallVec<[(&'static str, f64); 4]> }` + `Score::new`, `with_component`, `Default`.

Commit:

```bash
git add core/src/score.rs core/Cargo.toml core/src/lib.rs && git commit -m "[core] Step 1.7: Score struct with breakdown field"
```

### Step 1.8: `Goal` trait

**File**: `core/src/goal.rs` (new); export from `lib.rs`.

Failing test (ensures defaults):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::{score::Score, time::Time, world_view::WorldView};

    struct W;
    impl WorldView for W {}

    struct AlwaysOne;
    impl Goal<W> for AlwaysOne {
        fn evaluate(&self, _w: &W) -> Score { Score::new(1.0) }
    }

    #[test]
    fn defaults_for_tick_and_is_complete() {
        let g = AlwaysOne;
        // Default tick is no-op; default is_complete is false.
        assert!(!g.is_complete(&W));
        assert_eq!(g.evaluate(&W).value, 1.0);
    }
}
```

Implement per design §4: trait with `tick`, `is_complete`, `evaluate` (only `evaluate` required).

Commit:

```bash
git add core/src/goal.rs core/src/lib.rs && git commit -m "[core] Step 1.8: Goal trait with default tick + is_complete"
```

### Step 1.9: `SensorReading` marker trait (HAL §9.4)

**File**: `core/src/sensor_reading.rs` (new); export from `lib.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::Time;

    struct Foo { sampled_at: Time }
    impl SensorReading for Foo {
        fn sampled_at(&self) -> Time { self.sampled_at }
    }

    #[test]
    fn sample_timestamp_accessible() {
        let r = Foo { sampled_at: Time::from_millis(42) };
        assert_eq!(r.sampled_at(), Time::from_millis(42));
    }
}
```

Implement: `pub trait SensorReading { fn sampled_at(&self) -> Time; }`. Documented as the HAL §9.4 contract.

Commit:

```bash
git add core/src/sensor_reading.rs core/src/lib.rs && git commit -m "[core] Step 1.9: SensorReading marker trait (HAL §9.4)"
```

### Step 1.10: Final `core/src/lib.rs` re-exports + clippy clean

**File**: `core/src/lib.rs`.

Ensure all public types are re-exported at the crate root. Then:

```bash
cargo clippy -p core -- -D warnings
cargo test -p core
git add core/src/lib.rs && git commit -m "[core] Step 1.10: Public re-exports; clippy clean"
```

---

## Phase 2: `sim` foundations (no gravity yet)

Each step adds a module file in `sim/src/`, with `pub mod ...;` in `sim/src/lib.rs` and a re-export. Same TDD cycle as Phase 1.

### Step 2.1: `EntityId` + `Shape` primitives

**File**: `sim/src/entity.rs` (new); `sim/src/shape.rs` (new); `sim/src/lib.rs`.

Failing test:

```rust
// sim/src/entity.rs
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn entity_ids_distinguish_kinds() {
        assert_ne!(EntityId::Object(1), EntityId::Fixture(1));
    }
}
```

```rust
// sim/src/shape.rs
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    #[test]
    fn aabb_top_z() {
        let s = Shape::Aabb { half_extents: Vector3::new(0.05, 0.05, 0.10) };
        assert!((s.half_height_z() - 0.10).abs() < 1e-9);
    }
    #[test]
    fn sphere_top_z() {
        let s = Shape::Sphere { radius: 0.05 };
        assert!((s.half_height_z() - 0.05).abs() < 1e-9);
    }
}
```

Implement: `pub enum EntityId { Object(u32), Fixture(u32) }` + sort-stable derives. `pub enum Shape { Sphere { radius: f32 }, Aabb { half_extents: Vector3<f32> }, Cylinder { radius: f32, half_height: f32 } }` plus `pub fn half_height_z(&self) -> f32` helper.

Commit:

```bash
git add sim/src/entity.rs sim/src/shape.rs sim/src/lib.rs && git commit -m "[sim] Step 2.1: EntityId + Shape primitives"
```

### Step 2.2: `Object`, `Fixture`, `ObjectState`

**File**: `sim/src/object.rs` (new); `sim/src/fixture.rs` (new); `sim/src/lib.rs`. Add `nalgebra` dep usage.

Failing test (object construction + state transitions):

```rust
// sim/src/object.rs
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Isometry3, Vector3};
    #[test]
    fn free_object_constructed() {
        let o = Object::new(
            ObjectId(1),
            Isometry3::translation(0.0, 0.0, 0.05),
            Shape::Sphere { radius: 0.05 },
            /* mass */ 0.10,
            /* graspable */ true,
        );
        assert_eq!(o.state, ObjectState::Free);
        assert!(o.graspable);
    }
}
```

Implement per design §5.2: `pub struct ObjectId(pub u32);` + `pub enum ObjectState { Free, Grasped { by: ArmRef }, Settled { on: SupportId } }` (define `pub struct ArmRef(pub u32);` + `pub enum SupportId { Fixture(u32), Object(u32) }`). `Object { id, pose, shape, mass, graspable, state, lin_vel: Vector3<f32> }`. `Fixture { id: u32, pose, shape, is_support: bool }`.

Commit:

```bash
git add sim/src/object.rs sim/src/fixture.rs sim/src/lib.rs && git commit -m "[sim] Step 2.2: Object + Fixture + ObjectState"
```

### Step 2.3: `Scene` aggregation + deterministic id assignment

**File**: `sim/src/scene.rs` (new); `sim/src/lib.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Isometry3, Vector3};
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
        let ids: Vec<_> = s.objects().map(|o| o.id).collect();
        assert!(ids.windows(2).all(|w| w[0].0 < w[1].0));
    }
}
```

Implement: `pub struct Scene { objects: BTreeMap<ObjectId, Object>, fixtures: BTreeMap<u32, Fixture>, next_object_id: u32, next_fixture_id: u32, rng: rand_pcg::Pcg64 }` plus accessors. Use `BTreeMap` per design §10. Add `add_object_default` test helper (returns ObjectId).

Commit:

```bash
git add sim/src/scene.rs sim/Cargo.toml sim/src/lib.rs && git commit -m "[sim] Step 2.3: Scene with deterministic id assignment + BTreeMap storage"
```

### Step 2.4: `Color`, `Primitive`, `SceneSnapshot`

**File**: `sim/src/primitive.rs` (new); `sim/src/lib.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Isometry3, Vector3};
    #[test]
    fn snapshot_holds_items_in_insertion_order() {
        let snap = SceneSnapshot {
            t: core::time::Time::from_nanos(0),
            items: vec![
                (sim_entity(1), Primitive::Sphere {
                    pose: Isometry3::identity(), radius: 0.05, color: Color::WHITE
                }),
            ],
        };
        assert_eq!(snap.items.len(), 1);
    }

    fn sim_entity(n: u32) -> crate::entity::EntityId { crate::entity::EntityId::Object(n) }
}
```

Implement per design §7: `pub struct Color { r,g,b,a: u8 }` with `WHITE`, `RED`, etc. constants. `pub enum Primitive { Sphere{...}, Capsule{...}, Box{...}, Line{...}, Label{...} }`. `pub struct SceneSnapshot { pub t: Time, pub items: Vec<(EntityId, Primitive)> }`.

Commit:

```bash
git add sim/src/primitive.rs sim/src/lib.rs && git commit -m "[sim] Step 2.4: Primitive + Color + SceneSnapshot"
```

### Step 2.5: `Visualizable` trait + impl for `Object`

**File**: `sim/src/visualizable.rs` (new); `sim/src/object.rs` (extend); `sim/src/lib.rs`.

Failing test (in `object.rs`):

```rust
#[test]
fn object_appends_one_primitive_matching_shape() {
    use super::*;
    use nalgebra::Isometry3;
    let o = Object::new(ObjectId(1), Isometry3::identity(), Shape::Sphere{radius:0.05}, 0.1, true);
    let mut out = Vec::new();
    o.append_primitives(&mut out);
    assert_eq!(out.len(), 1);
    assert!(matches!(out[0].1, Primitive::Sphere { .. }));
}
```

Implement: `pub trait Visualizable { fn append_primitives(&self, out: &mut Vec<(EntityId, Primitive)>); }` (object-safe — `&self`, no generics, no associated types). `impl Visualizable for Object` matches on `self.shape` to emit one primitive.

Commit:

```bash
git add sim/src/visualizable.rs sim/src/object.rs sim/src/lib.rs && git commit -m "[sim] Step 2.5: Visualizable trait + impl for Object"
```

### Step 2.6: `Visualizable` impl for `Fixture`

**File**: `sim/src/fixture.rs` (extend).

Failing test mirrors 2.5. Implement same shape→Primitive mapping. Commit:

```bash
git add sim/src/fixture.rs && git commit -m "[sim] Step 2.6: Visualizable for Fixture"
```

### Step 2.7: `RateScheduler` (phase accumulator with documented ±1-tick jitter)

**File**: `sim/src/rate_scheduler.rs` (new); `sim/src/lib.rs`.

Failing tests (long-run rate matches; jitter bounded):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn long_run_rate_matches_requested() {
        // 333 Hz against 1 kHz sim: ~1000 ticks should yield ~333 fires (±1).
        let mut sched = RateScheduler::new_hz(333);
        let dt_ns = 1_000_000_i64; // 1 ms
        let mut fires = 0;
        for _ in 0..1_000 { if sched.tick(dt_ns) { fires += 1; } }
        assert!((fires - 333).abs() <= 1, "got {} fires", fires);
    }

    #[test]
    fn divisible_rate_fires_exactly_on_period() {
        let mut sched = RateScheduler::new_hz(100);
        let dt_ns = 1_000_000_i64;
        let mut fires = 0;
        for _ in 0..1_000 { if sched.tick(dt_ns) { fires += 1; } }
        assert_eq!(fires, 100);
    }
}
```

Implement per design §5.6: `pub struct RateScheduler { period_ns: i64, accumulator_ns: i64 }` with `new_hz(u32)` and `fn tick(&mut self, dt_ns: i64) -> bool` that increments accumulator, fires (returns true) when ≥ period, subtracts the period.

Commit:

```bash
git add sim/src/rate_scheduler.rs sim/src/lib.rs && git commit -m "[sim] Step 2.7: RateScheduler (phase accumulator)"
```

### Step 2.8: `SimClock` (sim-controlled clock)

**File**: `sim/src/sim_clock.rs` (new); `sim/src/lib.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use core::clock::Clock;
    use core::time::{Time, Duration};
    #[test]
    fn sim_clock_advances_via_advance() {
        let c = SimClock::new();
        assert_eq!(c.now(), Time::from_nanos(0));
        c.advance(Duration::from_millis(3));
        assert_eq!(c.now(), Time::from_millis(3));
    }
}
```

Implement: `pub struct SimClock { now: Cell<Time> }` + `impl Clock`. (Single-threaded; safe.)

Commit:

```bash
git add sim/src/sim_clock.rs sim/src/lib.rs && git commit -m "[sim] Step 2.8: SimClock"
```

### Step 2.9: `Recorder` trait + `NullRecorder`

**File**: `sim/src/recorder.rs` (new); `sim/src/lib.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use core::time::Time;
    #[test]
    fn null_recorder_is_a_no_op() {
        let mut r = NullRecorder;
        let snap = SceneSnapshot { t: Time::from_nanos(0), items: vec![] };
        r.record(&snap); // should not panic, should compile
    }
}
```

Implement: `pub trait Recorder { fn record(&mut self, snapshot: &SceneSnapshot); fn record_event(&mut self, _event: &ControllerEvent) {} }` + `pub struct NullRecorder; impl Recorder for NullRecorder { fn record(&mut self, _: &SceneSnapshot) {} }`. Define `ControllerEvent` enum per design §7 (just the variants — no implementation).

Commit:

```bash
git add sim/src/recorder.rs sim/src/lib.rs && git commit -m "[sim] Step 2.9: Recorder trait + NullRecorder"
```

### Step 2.10: `RunnableWorld` trait + `SimError`

**File**: `sim/src/runnable_world.rs` (new); `sim/src/lib.rs`.

Failing test (a no-op world impl):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use core::{time::{Time, Duration}, world_view::WorldView};
    struct EmptyWorld { t: Time }
    impl WorldView for EmptyWorld {}
    impl RunnableWorld for EmptyWorld {
        fn tick(&mut self, dt: Duration) -> Result<(), SimError> {
            self.t = self.t + dt;
            Ok(())
        }
        fn snapshot(&self) -> SceneSnapshot { SceneSnapshot { t: self.t, items: vec![] } }
        fn time(&self) -> Time { self.t }
    }

    #[test]
    fn world_advances_time_through_tick() {
        let mut w = EmptyWorld { t: Time::from_nanos(0) };
        w.tick(Duration::from_millis(1)).unwrap();
        assert_eq!(w.time(), Time::from_millis(1));
    }
}
```

Implement per design §7: `pub trait RunnableWorld: WorldView { fn tick(&mut self, dt: Duration) -> Result<(), SimError>; fn snapshot(&self) -> SceneSnapshot; fn time(&self) -> Time; }` + `pub struct SimError { pub detail: Cow<'static, str> }`.

Commit:

```bash
git add sim/src/runnable_world.rs sim/src/lib.rs && git commit -m "[sim] Step 2.10: RunnableWorld trait + SimError"
```

End of Phase 2 verification: `cargo test --workspace && cargo clippy --workspace -- -D warnings`.

---

## Phase 3: `arm` crate

### Step 3.1: `JointSpec`, `GripperSpec`, `ArmSpec`

**File**: `arm/src/spec.rs` (new); `arm/src/lib.rs`.

Failing test (defaulted ArmSpec is empty):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Vector3, Isometry3};
    #[test]
    fn arm_spec_with_two_revolute_joints() {
        let spec = ArmSpec {
            joints: vec![
                JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-3.14, 3.14) },
                JointSpec::Revolute { axis: Vector3::y_axis(), limits: (-1.57, 1.57) },
            ],
            link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.1); 2],
            gripper: GripperSpec { proximity_threshold: 0.02, max_grasp_size: 0.05 },
        };
        assert_eq!(spec.joints.len(), 2);
    }
}
```

Implement per design §5.3: `pub enum JointSpec { Revolute { axis: Unit<Vector3<f32>>, limits: (f32, f32) }, Prismatic { axis: Unit<Vector3<f32>>, limits: (f32, f32) } }`, `pub struct GripperSpec { proximity_threshold: f32, max_grasp_size: f32 }`, `pub struct ArmSpec { joints, link_offsets, gripper }`.

Commit:

```bash
git add arm/src/spec.rs arm/src/lib.rs && git commit -m "[arm] Step 3.1: ArmSpec + JointSpec + GripperSpec"
```

### Step 3.2: `ArmState` runtime state

**File**: `arm/src/state.rs` (new); `arm/src/lib.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn fresh_state_has_zero_q_and_open_gripper() {
        let s = ArmState::zeros(/* n_joints */ 3);
        assert_eq!(s.q, vec![0.0; 3]);
        assert_eq!(s.q_dot, vec![0.0; 3]);
        assert!(!s.gripper_closed);
        assert!(s.grasped.is_none());
    }
}
```

Implement per design §5.3.

Commit:

```bash
git add arm/src/state.rs arm/src/lib.rs && git commit -m "[arm] Step 3.2: ArmState"
```

### Step 3.3: Single-joint forward kinematics

**File**: `arm/src/fk.rs` (new); `arm/src/lib.rs`.

Failing tests (revolute around Z, prismatic along X):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Vector3, Isometry3};
    use crate::spec::JointSpec;

    #[test]
    fn revolute_z_rotates_offset_by_q() {
        let joint = JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-3.2, 3.2) };
        let offset = Isometry3::translation(1.0, 0.0, 0.0);
        let pose = joint_transform(&joint, std::f32::consts::FRAC_PI_2) * offset;
        // After 90° rotation about Z, x-translation maps to +y.
        assert!((pose.translation.x - 0.0).abs() < 1e-5);
        assert!((pose.translation.y - 1.0).abs() < 1e-5);
    }

    #[test]
    fn prismatic_x_translates_by_q() {
        let joint = JointSpec::Prismatic { axis: Vector3::x_axis(), limits: (0.0, 1.0) };
        let pose = joint_transform(&joint, 0.5);
        assert!((pose.translation.x - 0.5).abs() < 1e-9);
    }
}
```

Implement: `pub fn joint_transform(spec: &JointSpec, q: f32) -> Isometry3<f32>`. Use `nalgebra::UnitQuaternion::from_axis_angle` for revolute, `Translation3::from(axis * q)` for prismatic.

Commit:

```bash
git add arm/src/fk.rs arm/src/lib.rs && git commit -m "[arm] Step 3.3: Single-joint forward kinematics"
```

### Step 3.4: Multi-joint chain forward kinematics

**File**: `arm/src/fk.rs` (extend).

Failing test (3-joint planar chain along X):

```rust
#[test]
fn three_joint_chain_at_zero_q_yields_sum_of_offsets() {
    use crate::spec::{ArmSpec, JointSpec, GripperSpec};
    use nalgebra::{Vector3, Isometry3};
    let spec = ArmSpec {
        joints: vec![JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-3.2, 3.2) }; 3],
        link_offsets: vec![
            Isometry3::translation(0.5, 0.0, 0.0),
            Isometry3::translation(0.5, 0.0, 0.0),
            Isometry3::translation(0.5, 0.0, 0.0),
        ],
        gripper: GripperSpec { proximity_threshold: 0.01, max_grasp_size: 0.05 },
    };
    let q = vec![0.0; 3];
    let ee = forward_kinematics(&spec, &q);
    assert!((ee.translation.x - 1.5).abs() < 1e-5);
    assert!(ee.translation.y.abs() < 1e-5);
}
```

Implement: `pub fn forward_kinematics(spec: &ArmSpec, q: &[f32]) -> Isometry3<f32>` — fold over joints, multiplying `joint_transform(joint, q[i]) * link_offsets[i]` against an accumulating `Isometry3<f32>`.

Commit:

```bash
git add arm/src/fk.rs && git commit -m "[arm] Step 3.4: Multi-joint chain FK"
```

### Step 3.5: Port type definitions (with `sampled_at`)

**File**: `arm/src/ports.rs` (new); `arm/src/lib.rs`.

Failing test (each reading impls `SensorReading`):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use core::{sensor_reading::SensorReading, time::Time};
    #[test]
    fn joint_encoder_reading_exposes_sample_time() {
        let r = JointEncoderReading {
            joint: JointId(0), q: 0.5, q_dot: 0.1, sampled_at: Time::from_millis(7),
        };
        assert_eq!(r.sampled_at(), Time::from_millis(7));
    }
}
```

Implement: `pub struct JointId(pub u32);` then four reading/command types per design §5.6 and §6:

```rust
pub struct JointEncoderReading { pub joint: JointId, pub q: f32, pub q_dot: f32, pub sampled_at: Time }
pub struct EePoseReading       { pub pose: Isometry3<f32>, pub sampled_at: Time }
pub struct JointVelocityCommand{ pub joint: JointId, pub q_dot_target: f32 }
pub struct GripperCommand      { pub close: bool }
```

`impl SensorReading for JointEncoderReading` and `for EePoseReading`. (Commands don't impl `SensorReading`.) All `#[derive(Clone, Debug)]`.

Commit:

```bash
git add arm/src/ports.rs arm/src/lib.rs && git commit -m "[arm] Step 3.5: Arm port type definitions (encoder, EE pose, velocity cmd, gripper cmd)"
```

### Step 3.6: `Visualizable` for the arm (capsules per link, box for gripper)

**File**: `arm/src/viz.rs` (new); `arm/src/lib.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    // For an N-joint arm, expect N capsules (links) + 1 box (gripper).
    use crate::{spec::*, state::*};
    use nalgebra::{Vector3, Isometry3};
    use sim::{primitive::Primitive, entity::EntityId, visualizable::Visualizable};

    #[test]
    fn appends_n_capsules_and_one_box() {
        let spec = ArmSpec {
            joints: vec![JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-3.2, 3.2) }; 3],
            link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.1); 3],
            gripper: GripperSpec { proximity_threshold: 0.02, max_grasp_size: 0.05 },
        };
        let state = ArmState::zeros(3);
        let arm = Arm { spec, state, id: 0 };
        let mut out = Vec::new();
        arm.append_primitives(&mut out);
        let n_capsules = out.iter().filter(|(_, p)| matches!(p, Primitive::Capsule { .. })).count();
        let n_boxes    = out.iter().filter(|(_, p)| matches!(p, Primitive::Box { .. })).count();
        assert_eq!(n_capsules, 3);
        assert_eq!(n_boxes, 1);
    }
}
```

Implement: define `pub struct Arm { pub spec: ArmSpec, pub state: ArmState, pub id: u32 }` (move into a new `arm/src/arm.rs` module if not present). `impl Visualizable for Arm` walks the chain using `joint_transform` accumulator, emits `Primitive::Capsule` per link and one `Primitive::Box` at the EE pose.

Commit:

```bash
git add arm/src/viz.rs arm/src/arm.rs arm/src/lib.rs && git commit -m "[arm] Step 3.6: Visualizable impl for Arm"
```

### Step 3.7: `ArmWorld` skeleton (no ports yet)

**File**: `arm/src/world.rs` (new); `arm/src/lib.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use sim::scene::Scene;
    #[test]
    fn arm_world_is_constructible() {
        let world = ArmWorld::new(Scene::new(0), simple_spec());
        assert_eq!(world.arm.state.q.len(), simple_spec().joints.len());
        assert_eq!(world.time(), core::time::Time::from_nanos(0));
    }
    fn simple_spec() -> crate::spec::ArmSpec { /* 2-joint helper */ }
}
```

Implement skeleton:

```rust
pub struct ArmWorld {
    pub scene: Scene,
    pub arm: Arm,
    sim_time: core::time::Time,
    next_port_id: u32,
    sensors_joint_encoder:    BTreeMap<core::port_id::PortId, EncoderPublisher>,
    sensors_ee_pose:          BTreeMap<core::port_id::PortId, EePosePublisher>,
    actuators_joint_velocity: BTreeMap<core::port_id::PortId, JointVelocityConsumer>,
    actuators_gripper:        BTreeMap<core::port_id::PortId, GripperConsumer>,
}
```

Define stub `EncoderPublisher`, `EePosePublisher`, `JointVelocityConsumer`, `GripperConsumer` as `pub(crate) struct`s holding the corresponding `PortTx`/`PortRx` and a `RateScheduler` (for sensors). `ArmWorld::new(scene, spec)` constructs everything zeroed.

Commit:

```bash
git add arm/src/world.rs arm/src/lib.rs && git commit -m "[arm] Step 3.7: ArmWorld skeleton with port registries"
```

### Step 3.8: `attach_joint_encoder_sensor`

**File**: `arm/src/world.rs` (extend).

Failing test (sensor returns Rx; world holds Tx; ID is monotonic):

```rust
#[test]
fn attaching_two_encoders_yields_distinct_port_ids() {
    let mut world = ArmWorld::new(Scene::new(0), simple_spec());
    let _rx_a = world.attach_joint_encoder_sensor(JointId(0), RateHz::new(1000));
    let _rx_b = world.attach_joint_encoder_sensor(JointId(1), RateHz::new(1000));
    assert_eq!(world.sensors_joint_encoder.len(), 2);
}
```

Implement: `pub fn attach_joint_encoder_sensor(&mut self, joint: JointId, rate: RateHz) -> PortRx<JointEncoderReading>`. Construct `port::<JointEncoderReading>()`, take next `PortId`, build `EncoderPublisher { joint, tx, scheduler: RateScheduler::new_hz(rate.0) }`, insert into map, return `rx`. Also add `pub struct RateHz(pub u32); impl RateHz { pub fn new(hz: u32) -> Self }`.

Commit:

```bash
git add arm/src/world.rs && git commit -m "[arm] Step 3.8: attach_joint_encoder_sensor"
```

### Step 3.9: `attach_joint_velocity_actuator`

**File**: `arm/src/world.rs` (extend). Same pattern; consumer holds `PortRx`. Commit `[arm] Step 3.9: attach_joint_velocity_actuator`.

### Step 3.10: `attach_gripper_actuator` and `attach_ee_pose_sensor`

**File**: `arm/src/world.rs` (extend). Both follow the established pattern. Commit `[arm] Step 3.10: attach_gripper_actuator + attach_ee_pose_sensor`.

### Step 3.11: `ArmWorld::tick` (no gravity yet)

**File**: `arm/src/world.rs` (extend).

Failing test (encoder publishes after one tick at 1 kHz; commanded velocity advances joint position):

```rust
#[test]
fn tick_publishes_encoder_and_advances_q_on_velocity_command() {
    use core::time::Duration;
    let mut world = ArmWorld::new(Scene::new(0), simple_spec());
    let rx = world.attach_joint_encoder_sensor(JointId(0), RateHz::new(1000));
    let tx = world.attach_joint_velocity_actuator(JointId(0));
    tx.send(JointVelocityCommand { joint: JointId(0), q_dot_target: 1.0 });
    world.tick(Duration::from_millis(1)).unwrap();
    assert!(world.arm.state.q[0] > 0.0);
    assert!(rx.latest().is_some());
}
```

Implement per design §5.7 (skipping §5.5 gravity for now):

1. Iterate `sensors_*` BTreeMaps in PortId order; for each, scheduler.tick(dt_ns); if true, build reading from arm state + send.
2. Iterate `actuators_*` BTreeMaps in PortId order; for each, take latest from rx and apply (e.g., update `arm.state.q_dot[joint]`).
3. Compute FK; update grasp state (closing+nothing held+nearby graspable → attach; opening → detach).
4. Integrate `q += q_dot * dt`. Advance `sim_time`.

Note: in the current ordering, sensors publish *before* the controller step. Since `harness::run` is what calls `step` between sim steps (Phase 4), `tick` here just integrates state; publishing belongs in a separate call. Refactor: split into `pub fn publish_sensors(&mut self)` and `pub fn consume_actuators_and_integrate(&mut self, dt)`. Adjust the test accordingly.

Commit:

```bash
git add arm/src/world.rs && git commit -m "[arm] Step 3.11: ArmWorld::publish_sensors + consume_actuators_and_integrate (no gravity)"
```

### Step 3.12: `impl RunnableWorld for ArmWorld` + `ArmWorldView`

**File**: `arm/src/world.rs` (extend).

Failing test:

```rust
#[test]
fn arm_world_implements_runnable_world() {
    use sim::runnable_world::RunnableWorld;
    let mut world = ArmWorld::new(Scene::new(0), simple_spec());
    let snap = world.snapshot();
    assert_eq!(snap.t, world.time());
    // Snapshot has at least the arm primitives (3 capsules + 1 box for 3-joint arm).
    assert!(snap.items.len() >= 4);
}
```

Implement:

- `impl WorldView for ArmWorld {}` (and define accessor methods like `pub fn ee_pose(&self) -> Isometry3<f32>`, `pub fn object(&self, id: ObjectId) -> Option<&Object>` — these are used by goals).
- `impl RunnableWorld for ArmWorld { fn tick(&mut self, dt) { self.publish_sensors(); /* in run-loop the controller step happens here */ self.consume_actuators_and_integrate(dt); }, fn snapshot(&self) -> SceneSnapshot { /* walk: scene fixtures, scene objects, arm */ }, fn time(&self) -> Time { self.sim_time } }`.

Note: `RunnableWorld::tick` calls publish + consume in one shot; `harness` is responsible for inserting the controller step between them — see Phase 4 Step 4.3 for how `harness::run` orchestrates this with explicit calls to `world.publish_sensors()` then `controller.step()` then `world.consume_actuators_and_integrate(dt)`.

Commit:

```bash
git add arm/src/world.rs && git commit -m "[arm] Step 3.12: impl RunnableWorld + WorldView for ArmWorld"
```

End of Phase 3 verification: `cargo test --workspace && cargo clippy --workspace -- -D warnings`.

---

## Phase 4: `harness` crate

### Step 4.1: `RunConfig`, `RunResult`, `Termination`

**File**: `harness/src/types.rs` (new); `harness/src/lib.rs`.

Failing test (defaults sane):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use sim::recorder::NullRecorder;
    use core::time::Duration;
    #[test]
    fn run_config_defaults() {
        let cfg: RunConfig<NullRecorder> = RunConfig::default();
        assert_eq!(cfg.tick_rate_hz, 1000);
        assert!(cfg.gravity);
        assert_eq!(cfg.deadline, Duration::from_secs(10));
    }
}
```

Implement per design §6:

```rust
pub struct RunConfig<R: sim::recorder::Recorder = sim::recorder::NullRecorder> {
    pub tick_rate_hz: u32,
    pub deadline: core::time::Duration,
    pub seed: u64,
    pub recorder: R,
    pub gravity: bool,
}

pub struct RunResult {
    pub score: core::score::Score,
    pub final_time: core::time::Time,
    pub terminated_by: Termination,
}

pub enum Termination {
    GoalComplete,
    Deadline,
    ControllerError(core::controller::ControlError),
}

impl Default for RunConfig<sim::recorder::NullRecorder> {
    fn default() -> Self { /* tick=1000, deadline=10s, seed=0, NullRecorder, gravity=true */ }
}
```

Add builder methods: `with_deadline`, `with_seed`, `with_recorder`, `with_gravity`, `with_tick_rate`.

Commit:

```bash
git add harness/src/types.rs harness/src/lib.rs && git commit -m "[harness] Step 4.1: RunConfig + RunResult + Termination"
```

### Step 4.2: `harness::run` skeleton (immediate-deadline = no ticks)

**File**: `harness/src/run.rs` (new); `harness/src/lib.rs`.

Failing test (zero-deadline run terminates immediately):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use sim::{recorder::NullRecorder, runnable_world::RunnableWorld};
    use core::{time::{Time, Duration}, world_view::WorldView, score::Score, controller::*};
    use core::goal::Goal;

    struct W { t: Time }
    impl WorldView for W {}
    impl RunnableWorld for W {
        fn tick(&mut self, dt: Duration) -> Result<(), sim::SimError> { self.t = self.t + dt; Ok(()) }
        fn snapshot(&self) -> sim::primitive::SceneSnapshot { sim::primitive::SceneSnapshot { t: self.t, items: vec![] } }
        fn time(&self) -> Time { self.t }
    }

    struct Noop;
    impl Controller for Noop { fn step(&mut self, _: Time) -> Result<(), ControlError> { Ok(()) } }

    struct AlwaysHalf;
    impl Goal<W> for AlwaysHalf {
        fn evaluate(&self, _: &W) -> Score { Score::new(0.5) }
    }

    #[test]
    fn zero_deadline_terminates_immediately_with_evaluated_score() {
        let cfg = RunConfig::default().with_deadline(Duration::from_nanos(0));
        let res = run(W { t: Time::from_nanos(0) }, Noop, AlwaysHalf, cfg);
        assert!(matches!(res.terminated_by, Termination::Deadline));
        assert_eq!(res.score.value, 0.5);
    }
}
```

Implement: a `pub fn run<W, C, G, R>(mut world: W, mut controller: C, mut goal: G, cfg: RunConfig<R>) -> RunResult` that returns immediately with `Deadline` termination + `goal.evaluate(&world)` score. Loop body comes in 4.3.

Commit:

```bash
git add harness/src/run.rs harness/src/lib.rs && git commit -m "[harness] Step 4.2: harness::run skeleton (deadline-only)"
```

### Step 4.3: Tick loop with controller `step` between publish and consume

**File**: `harness/src/run.rs` (extend). Requires `RunnableWorld` to expose `publish_sensors` / `consume_actuators_and_integrate` directly so `run` can interleave the controller step. Add these as additional trait methods on `RunnableWorld` *or* keep them as `ArmWorld`-specific and have `run` be generic over a sub-trait `TickStaged: RunnableWorld { ... }`. Recommended: add to `RunnableWorld` as `fn publish_sensors(&mut self)` + `fn consume_actuators_and_integrate(&mut self, dt: Duration)`, with `tick` as a default that calls both with no controller in between (kept for the no-op world test).

Failing test:

```rust
#[test]
fn n_ticks_advance_world_time_by_n_dt() {
    let cfg = RunConfig::default()
        .with_deadline(Duration::from_millis(5))
        .with_tick_rate(1000);
    let res = run(W { t: Time::from_nanos(0) }, Noop, AlwaysHalf, cfg);
    assert_eq!(res.final_time, Time::from_millis(5));
}
```

Implement: compute `dt = Duration::from_nanos(1_000_000_000 / cfg.tick_rate_hz as i64)`. Loop: while `world.time() < cfg.deadline_time` and `!goal.is_complete(&world)`:

1. `world.publish_sensors();`
2. `goal.tick(world.time(), &world);`
3. `match controller.step(world.time()) { Err(ControlError { kind: Unrecoverable, .. }) => return RunResult { terminated_by: Termination::ControllerError(...), .. }, _ => {} }`
4. `world.consume_actuators_and_integrate(dt);`

After loop: evaluate `goal`, return `RunResult`.

Commit:

```bash
git add harness/src/run.rs sim/src/runnable_world.rs && git commit -m "[harness] Step 4.3: Tick loop with controller step between publish/consume"
```

### Step 4.4: Goal completion short-circuits the loop

**File**: `harness/src/run.rs` (extend).

Failing test (goal becomes complete after 3 ms):

```rust
#[test]
fn run_terminates_on_goal_complete_before_deadline() {
    struct DoneAt(Time);
    impl Goal<W> for DoneAt {
        fn is_complete(&self, w: &W) -> bool { w.time() >= self.0 }
        fn evaluate(&self, _: &W) -> Score { Score::new(1.0) }
    }
    let cfg = RunConfig::default().with_deadline(Duration::from_millis(10));
    let res = run(W { t: Time::from_nanos(0) }, Noop, DoneAt(Time::from_millis(3)), cfg);
    assert!(matches!(res.terminated_by, Termination::GoalComplete));
    assert!(res.final_time >= Time::from_millis(3));
}
```

Already covered by 4.3's loop condition; this test just ensures it. Commit `[harness] Step 4.4: Verify goal-complete short-circuit`.

### Step 4.5: Recorder is invoked once per tick

**File**: `harness/src/run.rs` (extend).

Failing test (a recording-counter recorder):

```rust
#[test]
fn recorder_is_called_once_per_tick() {
    use std::cell::RefCell;
    struct Counter(RefCell<u32>);
    impl sim::recorder::Recorder for Counter {
        fn record(&mut self, _: &sim::primitive::SceneSnapshot) { *self.0.borrow_mut() += 1; }
    }
    let cfg = RunConfig::default()
        .with_deadline(Duration::from_millis(5))
        .with_tick_rate(1000)
        .with_recorder(Counter(RefCell::new(0)));
    let res = run(W { t: Time::from_nanos(0) }, Noop, AlwaysHalf, cfg);
    // Counter is consumed; we'd need to check externally. Easier: use Rc<Cell<u32>>.
    let _ = res;
}
```

(Refactor the test to share the counter via `Rc<Cell<u32>>` since `recorder` is owned by `cfg`.)

Implement: between sub-steps 1 and 2 of the per-tick loop, after publishing sensors but before the controller step, call `cfg.recorder.record(&world.snapshot())`. Document this position (snapshot reflects post-publish, pre-control state at time `world.time()`).

Commit:

```bash
git add harness/src/run.rs && git commit -m "[harness] Step 4.5: Per-tick recorder integration"
```

End of Phase 4 verification: `cargo test --workspace`.

---

## Phase 5: First end-to-end test (no gravity)

This phase wires Phases 1–4 together and proves the pipeline end-to-end. The goal is `ReachPose` (no grasping, no falling — pure servoing).

### Step 5.1: `ReachPose` goal in `arm::goals`

**File**: `arm/src/goals/mod.rs` (new); `arm/src/goals/reach_pose.rs` (new); update `arm/src/lib.rs`.

Failing test (in `reach_pose.rs`, using a stub `ArmWorld` with a known EE pose):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Isometry3, Translation3};
    use crate::world::ArmWorld;
    use crate::spec::*;
    use sim::scene::Scene;

    #[test]
    fn score_is_one_when_at_target_within_tolerance() {
        let world = ArmWorld::new(Scene::new(0), simple_spec());
        let target = world.ee_pose();             // arm starts at identity-ish pose
        let goal = ReachPose::new(target, /* tolerance */ 0.01);
        assert!(goal.is_complete(&world));
        assert!(goal.evaluate(&world).value > 0.99);
    }
    fn simple_spec() -> ArmSpec { /* same helper as Step 3.7 */ }
}
```

Implement: `pub struct ReachPose { target: Isometry3<f32>, tolerance: f32 }`. `evaluate` computes `dist = (world.ee_pose().translation.vector - self.target.translation.vector).norm()` and returns `Score::new((1.0 - dist / self.tolerance).clamp(0.0, 1.0) as f64)`. `is_complete` is `dist <= self.tolerance`.

Commit:

```bash
git add arm/src/goals/ arm/src/lib.rs && git commit -m "[arm] Step 5.1: ReachPose goal"
```

### Step 5.2: Hand-rolled PD controller

**File**: `arm/examples/reach_pose_pd.rs` (new) — using `arm`'s `examples/` so it has access to internal port types via the public API.

```rust
//! Minimal PD controller that drives joint velocities from joint position error
//! against a precomputed target q*.

use core::{controller::*, port::*, time::Time};
use arm::ports::*;
use std::collections::HashMap;

pub struct PdJointController {
    target_q: Vec<f32>,
    kp: f32, kd: f32,
    encoder_rxs: Vec<PortRx<JointEncoderReading>>,    // one per joint, in joint-index order
    velocity_txs: Vec<PortTx<JointVelocityCommand>>,  // ditto
}

impl PdJointController {
    pub fn new(target_q: Vec<f32>,
               encoder_rxs: Vec<PortRx<JointEncoderReading>>,
               velocity_txs: Vec<PortTx<JointVelocityCommand>>) -> Self {
        Self { target_q, kp: 4.0, kd: 1.5, encoder_rxs, velocity_txs }
    }
}

impl Controller for PdJointController {
    fn step(&mut self, _t: Time) -> Result<(), ControlError> {
        for (i, rx) in self.encoder_rxs.iter_mut().enumerate() {
            let Some(reading) = rx.latest() else { continue };
            let err = self.target_q[i] - reading.q;
            let q_dot_target = self.kp * err - self.kd * reading.q_dot;
            self.velocity_txs[i].send(JointVelocityCommand {
                joint: JointId(i as u32), q_dot_target,
            });
        }
        Ok(())
    }
}

fn main() {} // empty; this file is consumed by tests below
```

Add a unit test inside the example:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use core::port::port;
    use arm::ports::*;
    use core::time::Time;

    #[test]
    fn pd_emits_velocity_command_proportional_to_error() {
        let (enc_tx, enc_rx) = port::<JointEncoderReading>();
        let (vel_tx, mut vel_rx) = port::<JointVelocityCommand>();
        let mut c = PdJointController::new(vec![1.0], vec![enc_rx], vec![vel_tx]);
        enc_tx.send(JointEncoderReading {
            joint: JointId(0), q: 0.0, q_dot: 0.0, sampled_at: Time::from_nanos(0),
        });
        c.step(Time::from_nanos(0)).unwrap();
        let cmd = vel_rx.take().unwrap();
        assert!(cmd.q_dot_target > 0.0); // moves toward +q
    }
}
```

Commit:

```bash
git add arm/examples/reach_pose_pd.rs && git commit -m "[arm] Step 5.2: PdJointController example"
```

### Step 5.3: `build_simple_arm_world` test helper

**File**: `arm/src/world.rs` (extend, behind `#[cfg(any(test, feature = "test-helpers"))]`) or new `arm/src/test_helpers.rs`.

```rust
pub fn build_simple_arm_world(n_joints: usize) -> ArmWorld {
    let spec = ArmSpec {
        joints: vec![JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-3.14, 3.14) }; n_joints],
        link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.2); n_joints],
        gripper: GripperSpec { proximity_threshold: 0.02, max_grasp_size: 0.05 },
    };
    ArmWorld::new(Scene::new(0), spec)
}

pub struct StandardArmPorts {
    pub encoder_rxs:  Vec<PortRx<JointEncoderReading>>,
    pub velocity_txs: Vec<PortTx<JointVelocityCommand>>,
    pub gripper_tx:   PortTx<GripperCommand>,
}

impl ArmWorld {
    pub fn attach_standard_arm_ports(&mut self) -> StandardArmPorts {
        let n = self.arm.spec.joints.len();
        let encoder_rxs = (0..n).map(|i| self.attach_joint_encoder_sensor(JointId(i as u32), RateHz::new(1000))).collect();
        let velocity_txs = (0..n).map(|i| self.attach_joint_velocity_actuator(JointId(i as u32))).collect();
        let gripper_tx = self.attach_gripper_actuator();
        StandardArmPorts { encoder_rxs, velocity_txs, gripper_tx }
    }
}
```

No new test for the helper itself; it's exercised in 5.4. Commit `[arm] Step 5.3: build_simple_arm_world + attach_standard_arm_ports`.

### Step 5.4: End-to-end `#[test]` — PD reaches target

**File**: `arm/tests/reach_pose_e2e.rs` (new — integration test).

```rust
use arm::{world::*, goals::reach_pose::ReachPose, examples_::PdJointController /* re-export from examples in lib */};
use core::time::{Duration, Time};
use harness::{run, types::{RunConfig, Termination}};

#[test]
fn pd_reaches_target_pose_within_2_seconds() {
    let mut world = build_simple_arm_world(3);
    let ports = world.attach_standard_arm_ports();

    // Aim for q = [0.5, 0.5, 0.5] rad → some non-identity EE pose.
    let target_q = vec![0.5_f32; 3];
    let controller = PdJointController::new(target_q.clone(), ports.encoder_rxs, ports.velocity_txs);

    // Compute target EE pose for the goal:
    let target_ee = arm::fk::forward_kinematics(&world.arm.spec, &target_q);
    let goal = ReachPose::new(target_ee, 0.01);

    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(2))
        .with_tick_rate(1000)
        .with_seed(42)
        .with_gravity(false);

    let res = run(world, controller, goal, cfg);
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "did not converge in time; final score = {}", res.score.value
    );
}
```

Note: `PdJointController` lives in `arm/examples/`; to use it from a `#[test]`, either move it to `arm/src/examples_.rs` (a public submodule) or keep it as `examples/` and have the test load it via `#[path = ...] mod` (less clean). Pick the cleaner path: move to `arm/src/examples_.rs` as a public module gated by a `with_examples` Cargo feature; the integration test enables that feature in `dev-dependencies`-style.

Run: `cargo test -p arm --test reach_pose_e2e -- --nocapture`. If the test fails (controller doesn't converge), tune `kp`, `kd`, or `deadline` — but don't loosen the tolerance below 0.01 m.

Commit:

```bash
git add arm/tests/reach_pose_e2e.rs arm/src/examples_.rs arm/Cargo.toml && git commit -m "[arm] Step 5.4: End-to-end ReachPose test (no gravity)"
```

End of Phase 5 verification: a single `#[test]` proves the entire pipeline (port wiring → tick loop → controller step → FK → goal evaluation → termination) works.

---

## Phase 6: Gravity-fall in `sim`

Per design §5.5. Lives in a new `sim::gravity` module. `ArmWorld::consume_actuators_and_integrate` calls into it before integrating arm joint state.

### Step 6.1: Gravity constants + `Object` lin-vel access

**File**: `sim/src/gravity.rs` (new, mostly empty stub); confirm `Object` already has `lin_vel: Vector3<f32>` field from Step 2.2.

Failing test:

```rust
#[cfg(test)]
mod tests {
    #[test]
    fn gravity_constant_is_9_81() {
        assert!((super::GRAVITY_M_PER_S2 - 9.81).abs() < 1e-6);
    }
}
```

Implement: `pub const GRAVITY_M_PER_S2: f32 = 9.81;` and `pub const SETTLE_EPSILON_M: f32 = 1e-5;`.

Commit:

```bash
git add sim/src/gravity.rs sim/src/lib.rs && git commit -m "[sim] Step 6.1: Gravity constants"
```

### Step 6.2: `find_support_beneath` helper

**File**: `sim/src/gravity.rs` (extend).

Failing test (object directly above a fixture finds it; object off-edge falls to ground):

```rust
#[cfg(test)]
mod tests_support {
    use super::*;
    use crate::{scene::Scene, fixture::*, object::*, shape::Shape};
    use nalgebra::{Isometry3, Vector3};

    #[test]
    fn object_above_fixture_finds_it() {
        let mut scene = Scene::new(0);
        let table_id = scene.add_fixture(Fixture {
            id: 0, pose: Isometry3::translation(0.0, 0.0, 0.05),
            shape: Shape::Aabb { half_extents: Vector3::new(0.5, 0.5, 0.05) },
            is_support: true,
        });
        let support = find_support_beneath(&scene, /*xy*/ (0.0, 0.0), /*z_above*/ 1.0, /*ignore*/ None);
        assert_eq!(support.unwrap().0, SupportId::Fixture(table_id));
    }

    #[test]
    fn object_off_edge_finds_no_support() {
        let mut scene = Scene::new(0);
        scene.add_fixture(Fixture {
            id: 0, pose: Isometry3::translation(0.0, 0.0, 0.05),
            shape: Shape::Aabb { half_extents: Vector3::new(0.1, 0.1, 0.05) },
            is_support: true,
        });
        let support = find_support_beneath(&scene, (5.0, 5.0), 1.0, None);
        assert!(support.is_none()); // no fixture under (5,5); ground plane is added separately
    }
}
```

Implement:

```rust
pub fn find_support_beneath(
    scene: &Scene,
    xy: (f32, f32),
    z_above: f32,
    ignore: Option<ObjectId>,
) -> Option<(SupportId, /*top_z*/ f32)> {
    let mut best: Option<(SupportId, f32)> = None;
    for (id, fix) in scene.fixtures() {
        if !fix.is_support { continue; }
        if !shape_xy_contains(fix.pose, &fix.shape, xy) { continue; }
        let top_z = fix.pose.translation.z + fix.shape.half_height_z();
        if top_z <= z_above && best.map_or(true, |(_, z)| top_z > z) {
            best = Some((SupportId::Fixture(*id), top_z));
        }
    }
    for (id, obj) in scene.objects() {
        if Some(*id) == ignore { continue; }
        if !matches!(obj.state, ObjectState::Settled { .. }) { continue; }
        if !shape_xy_contains(obj.pose, &obj.shape, xy) { continue; }
        let top_z = obj.pose.translation.z + obj.shape.half_height_z();
        if top_z <= z_above && best.map_or(true, |(_, z)| top_z > z) {
            best = Some((SupportId::Object(id.0), top_z));
        }
    }
    best
}

fn shape_xy_contains(pose: Isometry3<f32>, shape: &Shape, xy: (f32, f32)) -> bool { /* AABB/sphere/cyl test */ }
```

Add `Scene::fixtures()` and `Scene::objects()` iterator accessors if not present. Add a "ground plane" fixture by convention (id = u32::MAX, infinite extents, is_support = true) constructed by `Scene::with_ground()`.

Commit:

```bash
git add sim/src/gravity.rs sim/src/scene.rs && git commit -m "[sim] Step 6.2: find_support_beneath helper"
```

### Step 6.3: Single-object gravity integration

**File**: `sim/src/gravity.rs` (extend).

Failing test:

```rust
#[test]
fn free_object_falls_until_it_meets_a_fixture() {
    use crate::{scene::Scene, fixture::*, object::*, shape::Shape};
    use nalgebra::{Isometry3, Vector3};
    let mut scene = Scene::with_ground();
    let _table = scene.add_fixture(Fixture {
        id: 0, pose: Isometry3::translation(0.0, 0.0, 0.5),
        shape: Shape::Aabb { half_extents: Vector3::new(0.2, 0.2, 0.01) },
        is_support: true,
    });
    let oid = scene.add_object(Object::new(
        ObjectId(1),
        Isometry3::translation(0.0, 0.0, 1.0),
        Shape::Sphere { radius: 0.05 },
        0.1, true,
    ));
    // Step gravity for a generous duration:
    for _ in 0..2000 { gravity_step(&mut scene, 1_000_000 /* 1ms in ns */); }
    let o = &scene.objects().next().unwrap().1;
    assert!(matches!(o.state, ObjectState::Settled { .. }));
    let table_top_z = 0.5 + 0.01;
    assert!((o.pose.translation.z - (table_top_z + 0.05)).abs() < 1e-3);
}
```

Implement: `pub fn gravity_step(scene: &mut Scene, dt_ns: i64)` that walks all `Free` objects (sorted by ascending `q.z` then `id` for determinism), decrements `lin_vel.z` by `g * dt_s`, integrates `pose.translation.z`, calls `find_support_beneath`, and if `bottom_z <= top_z + SETTLE_EPSILON_M`, snaps z and transitions to `Settled { on: ... }`.

Commit:

```bash
git add sim/src/gravity.rs && git commit -m "[sim] Step 6.3: gravity_step for Free objects"
```

### Step 6.4: Stacking — object falls onto another settled object

**File**: `sim/src/gravity.rs` (test only — implementation is reused from 6.3).

Failing test:

```rust
#[test]
fn object_falls_onto_settled_object_below() {
    use crate::{scene::Scene, fixture::*, object::*, shape::Shape};
    use nalgebra::{Isometry3, Vector3};
    let mut scene = Scene::with_ground();
    scene.add_fixture(Fixture {
        id: 0, pose: Isometry3::translation(0.0, 0.0, 0.5),
        shape: Shape::Aabb { half_extents: Vector3::new(0.5, 0.5, 0.01) },
        is_support: true,
    });
    let bottom = scene.add_object(Object::new(
        ObjectId(1),
        Isometry3::translation(0.0, 0.0, 0.6),
        Shape::Aabb { half_extents: Vector3::new(0.05, 0.05, 0.05) },
        0.1, true,
    ));
    let top = scene.add_object(Object::new(
        ObjectId(2),
        Isometry3::translation(0.0, 0.0, 1.0),
        Shape::Aabb { half_extents: Vector3::new(0.05, 0.05, 0.05) },
        0.1, true,
    ));
    for _ in 0..2000 { super::gravity_step(&mut scene, 1_000_000); }
    let top_obj = scene.object(top).unwrap();
    let bottom_obj = scene.object(bottom).unwrap();
    let bottom_top = bottom_obj.pose.translation.z + bottom_obj.shape.half_height_z();
    assert!(matches!(top_obj.state, ObjectState::Settled { on: SupportId::Object(_) }));
    assert!((top_obj.pose.translation.z - (bottom_top + top_obj.shape.half_height_z())).abs() < 1e-3);
}
```

Implementation already exists from 6.3. Commit `[sim] Step 6.4: Stack-on-stack test`.

### Step 6.5: Re-evaluate displaced settled objects

**File**: `sim/src/gravity.rs` (extend).

Failing test (when a settled object's support is removed, it becomes Free):

```rust
#[test]
fn settled_object_becomes_free_when_support_disappears() {
    use crate::{scene::Scene, fixture::*, object::*, shape::Shape};
    use nalgebra::{Isometry3, Vector3};
    let mut scene = Scene::with_ground();
    let table = scene.add_fixture(Fixture {
        id: 0, pose: Isometry3::translation(0.0, 0.0, 0.5),
        shape: Shape::Aabb { half_extents: Vector3::new(0.5, 0.5, 0.01) },
        is_support: true,
    });
    let oid = scene.add_object(Object::new(
        ObjectId(1),
        Isometry3::translation(0.0, 0.0, 0.51 + 0.05),
        Shape::Sphere { radius: 0.05 },
        0.1, true,
    ));
    super::gravity_step(&mut scene, 1_000_000);
    assert!(matches!(scene.object(oid).unwrap().state, ObjectState::Settled { .. }));
    // Remove the table:
    scene.remove_fixture(table);
    super::reevaluate_settled(&mut scene);
    assert!(matches!(scene.object(oid).unwrap().state, ObjectState::Free));
}
```

Implement: `pub fn reevaluate_settled(scene: &mut Scene)` walks `Settled { on: X }` objects; if `X` no longer exists (`Scene::has_support(X)`) or no longer occupies the object's xy, transition to `Free`.

Add `Scene::remove_fixture` and `Scene::has_support` accessors as needed.

Commit:

```bash
git add sim/src/gravity.rs sim/src/scene.rs && git commit -m "[sim] Step 6.5: reevaluate_settled (handles vanished supports)"
```

### Step 6.6: Hook gravity into `ArmWorld::consume_actuators_and_integrate`

**File**: `arm/src/world.rs` (extend).

Failing test:

```rust
#[test]
fn arm_world_with_gravity_lets_object_fall_onto_table() {
    let mut world = build_pick_and_place_world();   // helper introduced in 7.x; for now inline
    // ... place an object above the table, tick a few hundred times, assert settled.
}
```

Implement: at the top of `consume_actuators_and_integrate(dt)`, call `sim::gravity::reevaluate_settled(&mut self.scene); sim::gravity::gravity_step(&mut self.scene, dt.as_nanos());` *if* `self.gravity_enabled` is true. Add the `gravity_enabled: bool` field to `ArmWorld`, default `true`. Plumb `RunConfig::with_gravity(false)` to set this via a `pub fn set_gravity(&mut self, on: bool)`.

The `harness::run` function reads `cfg.gravity` once before the loop and calls `world.set_gravity(cfg.gravity)`.

Commit:

```bash
git add arm/src/world.rs harness/src/run.rs && git commit -m "[sim,arm,harness] Step 6.6: Wire gravity into ArmWorld + RunConfig"
```

### Step 6.7: Released-object integration test (full pipeline)

**File**: `arm/tests/release_falls.rs` (new).

```rust
#[test]
fn object_grasped_then_released_falls_to_table() {
    // Build world, grasp an object, move EE somewhere over the table, open gripper,
    // let physics run. Assert: object lands on the table, settled.
}
```

Concrete steps in the test: construct world with table at z=0.5 + an object held by the arm at z=1.0; controller is a tiny scripted state machine ("close gripper for 100ms, then move toward target xy via velocity command, then open gripper"); `goal` is a stub `AlwaysScore(1.0)` since we just want the deadline to fire; assert at the end that `world.object(BLOCK).unwrap().state` is `Settled` and `pose.translation.z` ≈ `table_top_z + block.half_height_z`.

Commit:

```bash
git add arm/tests/release_falls.rs && git commit -m "[arm] Step 6.7: Released-object falls to table integration test"
```

End of Phase 6 verification: `cargo test --workspace` — gravity tests pass; `reach_pose_e2e` from Phase 5 still passes (regression check).

---

## Phase 7: Goals — `PickObject`, `PlaceInBin`, `Stack`

These three goals are independent. Build any in parallel after Phase 6 is done.

### Step 7.1: `PickObject` goal

**File**: `arm/src/goals/pick_object.rs` (new); `arm/src/goals/mod.rs` (re-export).

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::build_simple_arm_world;
    use sim::object::ObjectId;

    #[test]
    fn complete_when_target_object_is_grasped() {
        let mut world = build_simple_arm_world(3);
        let block = world.scene.add_object_default(); // helper sets graspable=true
        // Force the grasp condition manually for test:
        world.arm.state.grasped = Some(block);
        let goal = PickObject::new(block);
        assert!(goal.is_complete(&world));
        assert!(goal.evaluate(&world).value > 0.99);
    }

    #[test]
    fn evaluate_falls_off_with_distance_when_not_held() {
        let mut world = build_simple_arm_world(3);
        let block = world.scene.add_object_default();
        let goal = PickObject::new(block);
        assert!(!goal.is_complete(&world));
        let s = goal.evaluate(&world).value;
        assert!(s >= 0.0 && s < 1.0);
    }
}
```

Implement: `pub struct PickObject { target: ObjectId }`. `is_complete`: `world.arm.state.grasped == Some(self.target)`. `evaluate`: 1.0 if grasped, else shaped on `(world.ee_pose().translation.vector - obj.pose.translation.vector).norm()` clamped to [0, 1].

Commit `[arm] Step 7.1: PickObject goal`.

### Step 7.2: `PlaceInBin` goal (uses gravity-fall settled state)

**File**: `arm/src/goals/place_in_bin.rs` (new); `arm/src/goals/mod.rs`.

Failing tests:

```rust
#[test]
fn complete_when_object_is_settled_inside_bin() {
    let mut world = build_simple_arm_world(3);
    let bin = world.scene.add_fixture(/* bin at (0.0, 0.5, 0.0), half-extents (0.1, 0.1, 0.05), is_support=true */);
    let block = world.scene.add_object(/* at (0.0, 0.5, 0.05+0.05), graspable=true */);
    // Manually set the block to Settled on the bin:
    world.scene.object_mut(block).unwrap().state =
        sim::object::ObjectState::Settled { on: sim::object::SupportId::Fixture(bin) };
    let goal = PlaceInBin::new(block, bin);
    assert!(goal.is_complete(&world));
    assert!(goal.evaluate(&world).value > 0.99);
}

#[test]
fn not_complete_if_object_settled_elsewhere() {
    /* settled on a different fixture → 0 score */
}
```

Implement: `pub struct PlaceInBin { target: ObjectId, bin: u32 /* fixture id */ }`. `is_complete`: object is `Settled { on: SupportId::Fixture(bin) }` AND its xy lies within the bin's xy footprint. `evaluate`: 1.0 if complete, else shaped (function of horizontal distance from bin center).

Commit `[arm] Step 7.2: PlaceInBin goal`.

### Step 7.3: `Stack` goal

**File**: `arm/src/goals/stack.rs` (new); `arm/src/goals/mod.rs`.

Failing test:

```rust
#[test]
fn complete_when_a_settled_on_b_for_at_least_100ms() {
    let mut world = build_simple_arm_world(3);
    let a = world.scene.add_object_default();
    let b = world.scene.add_object_default();
    world.scene.object_mut(a).unwrap().state =
        sim::object::ObjectState::Settled { on: sim::object::SupportId::Object(b.0) };
    let mut goal = Stack::new(a, b);
    // Tick the goal for 100ms of sim time (each tick advances the goal's internal "settled-for" counter).
    for ms in 0..100 {
        goal.tick(core::time::Time::from_millis(ms), &world);
        assert!(!goal.is_complete(&world), "should not be complete yet at {} ms", ms);
    }
    goal.tick(core::time::Time::from_millis(100), &world);
    assert!(goal.is_complete(&world));
    assert!(goal.evaluate(&world).value > 0.99);
}
```

Implement per design §6: `pub struct Stack { a: ObjectId, b: ObjectId, settled_since: Option<Time> }`. `tick` updates `settled_since` based on current state. `is_complete`: `(now - settled_since) >= 100ms`.

Commit `[arm] Step 7.3: Stack goal`.

### Step 7.4: End-to-end PickPlace test

**File**: `arm/tests/pick_place_e2e.rs` (new).

Implement a simple state-machine "PickPlace" controller in `arm/src/examples_.rs` (alongside `PdJointController`). State machine: `Approach → CloseGripper → MoveToBin → OpenGripper → Done`. Each state emits velocity commands and a gripper command per tick; transitions on geometric conditions (EE near object, object grasped, EE near bin).

Then the test:

```rust
#[test]
fn state_machine_picks_block_and_drops_in_bin() {
    let mut world = build_pick_and_place_world(); // table + bin + block, gravity ON
    let ports = world.attach_standard_arm_ports();
    let block = arm::test_helpers::block_id(&world);
    let bin   = arm::test_helpers::bin_id(&world);
    let controller = arm::examples_::PickPlace::new(ports, block, bin);
    let goal = arm::goals::place_in_bin::PlaceInBin::new(block, bin);
    let cfg = harness::types::RunConfig::default()
        .with_deadline(core::time::Duration::from_secs(15))
        .with_seed(42);
    let res = harness::run(world, controller, goal, cfg);
    assert!(
        matches!(res.terminated_by, harness::types::Termination::GoalComplete),
        "score={}, time={:?}", res.score.value, res.final_time
    );
    assert!(res.score.value > 0.9);
}
```

This is the headline acceptance test for v1. Commit `[arm] Step 7.4: End-to-end PickPlace test`.

End of Phase 7 verification: full workspace test suite passes; the headline E2E completes in well under the 15 s deadline.

---

## Phase 8: `viz` crate (rerun-backed visualizer)

Independent of Phases 6 & 7 — can run in parallel after Phase 5.

### Step 8.1: `viz` skeleton with `rerun` feature flag

**File**: `viz/src/lib.rs`; `viz/Cargo.toml`.

Add an optional `rerun` feature in `viz/Cargo.toml`:

```toml
[features]
default = []
rerun = ["dep:rerun"]

[dependencies]
sim = { path = "../sim" }
rerun = { workspace = true, optional = true }
```

Failing test (compiles without the feature):

```rust
#[cfg(test)]
mod tests {
    #[test]
    fn viz_compiles_without_rerun_feature() {
        // No-op test; success means the crate compiles in default config.
    }
}
```

Commit:

```bash
git add viz/Cargo.toml viz/src/lib.rs && git commit -m "[viz] Step 8.1: viz crate skeleton with optional rerun feature"
```

### Step 8.2: `FileRecorder` (JSON, no external deps)

**File**: `viz/src/file_recorder.rs` (new); `viz/src/lib.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use sim::{primitive::*, recorder::Recorder, entity::EntityId};
    use core::time::Time;
    use nalgebra::Isometry3;
    use std::io::{BufRead, BufReader};
    use std::fs::File;

    #[test]
    fn appends_one_line_per_snapshot() {
        let path = std::env::temp_dir().join("viz_test.jsonl");
        let _ = std::fs::remove_file(&path);
        let mut rec = FileRecorder::create(&path).unwrap();
        rec.record(&SceneSnapshot { t: Time::from_nanos(0), items: vec![] });
        rec.record(&SceneSnapshot { t: Time::from_millis(1), items: vec![] });
        drop(rec);
        let lines: Vec<_> = BufReader::new(File::open(&path).unwrap()).lines().collect();
        assert_eq!(lines.len(), 2);
    }
}
```

Implement: `pub struct FileRecorder { writer: BufWriter<File> }` + `pub fn create<P: AsRef<Path>>(path: P) -> io::Result<Self>` + `impl Recorder` that serializes each snapshot as one JSON line. Use a tiny hand-rolled serializer (no `serde`) to keep deps light, or add `serde` + `serde_json` as a `viz` dep — author's choice; serde is the simpler route.

Commit:

```bash
git add viz/src/file_recorder.rs viz/Cargo.toml viz/src/lib.rs && git commit -m "[viz] Step 8.2: FileRecorder (JSON-lines format)"
```

### Step 8.3: `RerunRecorder` (behind `rerun` feature)

**File**: `viz/src/rerun_recorder.rs` (new; `#[cfg(feature = "rerun")]` everywhere); `viz/src/lib.rs`.

Failing test (only with the feature):

```rust
#[cfg(all(test, feature = "rerun"))]
mod tests {
    use super::*;
    use sim::{primitive::*, recorder::Recorder};
    use core::time::Time;
    #[test]
    fn rerun_recorder_constructs_in_memory() {
        let mut rec = RerunRecorder::new_in_memory("test").unwrap();
        rec.record(&SceneSnapshot { t: Time::from_nanos(0), items: vec![] });
    }
}
```

Implement: wrap `rerun::RecordingStream`. For each `Primitive` variant, map to the closest rerun archetype:

- `Primitive::Sphere` → `rerun::archetypes::Points3D` with one point + radius (or `Mesh3D` for filled spheres).
- `Primitive::Capsule` → `rerun::archetypes::Capsules3D` (if available in your rerun version) or fall back to `Mesh3D`.
- `Primitive::Box` → `rerun::archetypes::Boxes3D`.
- `Primitive::Line` → `rerun::archetypes::LineStrips3D`.
- `Primitive::Label` → `rerun::archetypes::TextLog` or `Points3D` with label.

Use `rec_stream.set_time_seconds("sim", snapshot.t.as_secs_f64())` to put each snapshot on the timeline.

Run: `cargo test -p viz --features rerun`.

Commit:

```bash
git add viz/src/rerun_recorder.rs viz/src/lib.rs && git commit -m "[viz] Step 8.3: RerunRecorder behind 'rerun' feature"
```

### Step 8.4: Smoke test — opt RerunRecorder into the headline E2E

**File**: `arm/tests/pick_place_e2e.rs` (extend with a feature-gated variant).

Add a parallel test that runs the same scenario but with `RerunRecorder` enabled (using `#[cfg(all(test, feature = "viz-rerun"))]`). Add a `viz-rerun` feature in `arm/Cargo.toml` that pulls in `viz/rerun`.

```rust
#[cfg(feature = "viz-rerun")]
#[test]
fn state_machine_picks_block_and_drops_in_bin_with_rerun() {
    // identical to the non-viz test but with .with_recorder(viz::rerun_recorder::RerunRecorder::spawn_viewer("pick_place").unwrap())
}
```

Run:

```bash
cargo test -p arm --features viz-rerun --test pick_place_e2e
# Then open the rerun viewer manually (or use `RerunRecorder::spawn_viewer`).
```

Commit:

```bash
git add arm/tests/pick_place_e2e.rs arm/Cargo.toml && git commit -m "[arm] Step 8.4: Optional rerun-recorded variant of E2E test"
```

---

## Phase 9: `sim::faults` — channel and reading combinators

Independent of Phases 6, 7, 8 — can run after Phase 2.

Each combinator wraps a `PortRx<T>` (or in some cases a `PortTx<T>`) and is itself a `PortRx<T>` (or `PortTx<T>`). Determinism is guaranteed by an explicit seed.

**Implementation pattern.** Each wrapper is a struct that owns the inner endpoint plus its state (RNG, buffer, latency clock). It exposes the same `latest()`/`take()` API as `PortRx<T>` — but to allow controllers to consume them transparently, we need a uniform receiver type. Two options:

- **Option A (object-safe trait)**: define `pub trait PortReader<T> { fn latest(&self) -> Option<T>; fn take(&mut self) -> Option<T>; }` in `core::port` and have both raw `PortRx<T>` and every fault wrapper implement it. Controllers take `Box<dyn PortReader<T>>` (or generic `R: PortReader<T>`).
- **Option B (concrete `PortRx<T>`)**: have each fault wrapper construct a fresh `(PortTx, PortRx)` and run a small "pump" inside its `Drop`/scheduler, copying from the inner port to the outer. More machinery; more allocation; concrete type stays simple.

**Recommendation: Option A**. Add the `PortReader<T>` trait in `core::port` *now* (back-port to Phase 1.4 if you've already started). Update controllers to be generic over `R: PortReader<T>`.

If Option A is impractical mid-implementation, defer Option B and deliver fault wrappers as standalone types that controllers explicitly type against (e.g., `controller takes faults::Delay<JointEncoderReading>` directly).

### Step 9.1: `sim::faults` module + `PortReader` trait (if not yet)

**File**: `core/src/port.rs` (extend, back-fill from 1.4); `sim/src/faults/mod.rs` (new); `sim/src/lib.rs`.

Failing test (raw PortRx implements PortReader):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn raw_port_rx_implements_port_reader() {
        let (tx, rx) = port::<i32>();
        tx.send(42);
        fn assert_reader<R: PortReader<i32>>(_: &R) {}
        assert_reader(&rx);
        assert_eq!(rx.latest(), Some(42));
    }
}
```

Implement `PortReader<T>` trait in `core::port` and impl for `PortRx<T>`. Update Phase 5/7 controllers to be generic over `R: PortReader<T>` if not yet.

Commit:

```bash
git add core/src/port.rs sim/src/faults/mod.rs sim/src/lib.rs && git commit -m "[core,sim] Step 9.1: PortReader trait + sim::faults skeleton"
```

### Step 9.2: `Delay` wrapper (latency)

**File**: `sim/src/faults/delay.rs` (new); `sim/src/faults/mod.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use core::{port::*, time::{Time, Duration}};
    use core::clock::Clock;
    use sim::sim_clock::SimClock;

    #[test]
    fn delay_releases_value_only_after_latency_elapses() {
        let clock = SimClock::new();
        let (tx, rx) = port::<i32>();
        let delayed = Delay::new(rx, Duration::from_millis(2), &clock as &dyn Clock);
        tx.send(7);
        // Time hasn't advanced — latest should be None.
        assert!(delayed.latest().is_none());
        clock.advance(Duration::from_millis(2));
        assert_eq!(delayed.latest(), Some(7));
    }
}
```

Implement: `pub struct Delay<R: PortReader<T>, T> { inner: R, buffer: VecDeque<(Time, T)>, latency: Duration, clock: ... }`. On every `latest()`/`take()`, drain new arrivals from `inner`, stamp them with `clock.now() + latency`, then return the head iff its release time `<= clock.now()`.

Note: Delay needs a `Clock` reference. Either pass `&dyn Clock` (lifetime-bound) or `Rc<dyn Clock>`. `SimClock` is `Rc`-friendly. Pick `Rc<SimClock>` and have `ArmWorld::sim_clock() -> Rc<SimClock>` for the wiring.

Commit:

```bash
git add sim/src/faults/delay.rs && git commit -m "[sim] Step 9.2: Delay fault wrapper"
```

### Step 9.3: `DropMessages` wrapper (lossy channel, deterministic)

**File**: `sim/src/faults/drop_messages.rs` (new).

Failing test:

```rust
#[test]
fn drop_messages_drops_about_the_specified_fraction() {
    use super::*;
    use core::port::*;
    let (tx, rx) = port::<i32>();
    let mut dropper = DropMessages::new(rx, /* drop_rate */ 0.5, /* seed */ 42);
    let mut received = 0;
    for v in 0..1000 {
        tx.send(v);
        if dropper.take().is_some() { received += 1; }
    }
    assert!((received - 500).abs() < 80, "received {} (expected ≈500)", received);
}
```

Implement: holds an inner reader + `rand_pcg::Pcg64` seeded by `seed`. On each `take()`, draw a uniform; if `< drop_rate`, discard the inner value and return `None`; else pass through.

Commit:

```bash
git add sim/src/faults/drop_messages.rs && git commit -m "[sim] Step 9.3: DropMessages fault wrapper"
```

### Step 9.4: `StaleData` wrapper (rejects readings older than max_age)

**File**: `sim/src/faults/stale_data.rs` (new).

Requires `T: SensorReading` (so `sampled_at` is exposed).

Failing test:

```rust
#[test]
fn stale_data_rejects_old_readings() {
    use super::*;
    use core::{port::*, time::{Time, Duration}, sensor_reading::SensorReading};
    use sim::sim_clock::SimClock;
    use std::rc::Rc;

    #[derive(Clone)]
    struct R { sampled_at: Time }
    impl SensorReading for R { fn sampled_at(&self) -> Time { self.sampled_at } }

    let clock = Rc::new(SimClock::new());
    let (tx, rx) = port::<R>();
    let stale = StaleData::new(rx, Duration::from_millis(50), Rc::clone(&clock));
    tx.send(R { sampled_at: Time::from_millis(0) });
    clock.advance(Duration::from_millis(60));
    assert!(stale.latest().is_none());     // older than 50ms
    tx.send(R { sampled_at: Time::from_millis(60) });
    assert!(stale.latest().is_some());
}
```

Implement: on `latest()`/`take()`, fetch from inner; if `clock.now() - r.sampled_at() > max_age`, return None.

Commit:

```bash
git add sim/src/faults/stale_data.rs && git commit -m "[sim] Step 9.4: StaleData fault wrapper"
```

### Step 9.5: `GaussianNoise` wrapper

**File**: `sim/src/faults/gaussian_noise.rs` (new).

Requires `T` to expose mutable noise points. Easiest: add a `Noise` trait that types like `JointEncoderReading` implement to perturb their own fields.

```rust
pub trait Noise {
    fn apply_noise(&mut self, rng: &mut rand_pcg::Pcg64, stddev: f32);
}
```

For `JointEncoderReading`: perturb `q` and `q_dot` by `stddev * randn`. Add this in `arm::ports`.

Failing test:

```rust
#[test]
fn gaussian_noise_perturbs_q_field() {
    use super::*;
    use core::port::*;
    use arm::ports::*;
    let (tx, rx) = port::<JointEncoderReading>();
    let mut noisy = GaussianNoise::new(rx, /* stddev */ 0.01_f32, /* seed */ 0);
    tx.send(JointEncoderReading { joint: JointId(0), q: 1.0, q_dot: 0.0, sampled_at: core::time::Time::from_nanos(0) });
    let r = noisy.take().unwrap();
    assert!((r.q - 1.0).abs() < 0.1);   // perturbed by O(stddev)
    assert_ne!(r.q, 1.0);                // but not exactly 1.0
}
```

Implement `GaussianNoise<R, T> where T: Clone + Noise`.

Commit:

```bash
git add sim/src/faults/gaussian_noise.rs arm/src/ports.rs && git commit -m "[sim,arm] Step 9.5: GaussianNoise fault wrapper + Noise trait"
```

### Step 9.6: Robustness test — PD reaches pose under encoder delay + noise

**File**: `arm/tests/reach_pose_with_faults.rs` (new).

```rust
#[test]
fn pd_still_reaches_target_with_2ms_encoder_delay_and_noise() {
    use std::rc::Rc;
    let mut world = build_simple_arm_world(3);
    let clock = world.sim_clock_handle(); // Rc<SimClock>
    let raw_rxs = (0..3).map(|i| world.attach_joint_encoder_sensor(JointId(i), RateHz::new(1000))).collect::<Vec<_>>();
    let velocity_txs = (0..3).map(|i| world.attach_joint_velocity_actuator(JointId(i))).collect::<Vec<_>>();

    let noisy_delayed: Vec<Box<dyn PortReader<JointEncoderReading>>> = raw_rxs.into_iter().map(|rx| {
        let noisy = sim::faults::GaussianNoise::new(rx, 0.005, 7);
        let delayed = sim::faults::Delay::new(noisy, Duration::from_millis(2), Rc::clone(&clock));
        Box::new(delayed) as _
    }).collect();

    let target_q = vec![0.5_f32; 3];
    let target_ee = arm::fk::forward_kinematics(&world.arm.spec, &target_q);
    let controller = arm::examples_::PdJointController::new_dyn(target_q, noisy_delayed, velocity_txs);
    let goal = arm::goals::reach_pose::ReachPose::new(target_ee, 0.02);
    let cfg = harness::types::RunConfig::default()
        .with_deadline(Duration::from_secs(3))
        .with_seed(42)
        .with_gravity(false);
    let res = harness::run(world, controller, goal, cfg);
    assert!(matches!(res.terminated_by, harness::types::Termination::GoalComplete),
        "did not converge under faults; score={}", res.score.value);
}
```

Note: `PdJointController::new_dyn` takes `Vec<Box<dyn PortReader<JointEncoderReading>>>` instead of concrete `Vec<PortRx<...>>`. Add this constructor in `arm::examples_::PdJointController`.

Commit:

```bash
git add arm/tests/reach_pose_with_faults.rs arm/src/examples_.rs && git commit -m "[arm] Step 9.6: Robustness test (encoder delay + noise)"
```

---

## Final End-to-End Verification

Run after all 9 phases land. This is the gate for declaring v1 complete.

### V.1 Full test suite passes (deterministic)

```bash
cargo test --workspace --release
```

Run twice; assert identical pass count. (Determinism guarantee — design §10.)

### V.2 Clippy clean across the workspace

```bash
cargo clippy --workspace --all-targets -- -D warnings
```

Zero warnings. If any new lint fires for the determinism invariants (`HashMap` use in tick path, `Instant::now`, `thread::spawn`), fix at the source — do not allow.

### V.3 Headline acceptance test

```bash
cargo test -p arm --test pick_place_e2e -- --nocapture
```

Expect: `state_machine_picks_block_and_drops_in_bin` passes with `score > 0.9` and termination `GoalComplete` within 15 s of sim time.

### V.4 Robustness test passes

```bash
cargo test -p arm --test reach_pose_with_faults -- --nocapture
```

PD reaches pose under 2 ms encoder delay + 0.005 rad noise.

### V.5 Visualizer round-trip (manual)

```bash
cargo test -p arm --features viz-rerun --test pick_place_e2e -- --nocapture
```

Open the rerun viewer, confirm: arm capsules visible, block sphere visible, bin box visible, timeline scrubbable, block visibly drops into bin near the end.

### V.6 Documentation snapshot

Confirm the design doc, review file, and plan file are all checked in:

```bash
git log --oneline docs/plans/
```

Expect entries for: v1 design, review-1, v2 design, this implementation plan, plus per-step commits.

---

## Notes on parallelism within phases

- **Phase 7** (PickObject, PlaceInBin, Stack goals): the three goals are entirely independent — separate files, separate tests. Can be implemented in parallel by separate sessions/agents and merged.
- **Phase 8** (viz) and **Phase 9** (faults): both depend only on Phase 5 being done. Can run alongside Phases 6 and 7.
- **Within other phases**: steps build on each other (each step's test imports types from prior steps), so execute serially within a phase.

## Where to deviate

- If the SNN integration story needs concrete validation earlier than v1, add an SNN controller step after Phase 5 — design §8.2 has the canonical drift-free `step` template.
- If `RunnableWorld::publish_sensors` / `consume_actuators_and_integrate` cannot cleanly live on the trait (e.g., signature gymnastics), fall back to having `harness::run` accept a more specific bound or a helper function that the world type provides — keep the trait minimal.

## Out of scope for v1 (do not silently expand)

Per design §11: physics dynamics, lateral motion, joint torques, mesh geometry, multi-robot scenes, non-arm modalities, hardware adapters, data-defined scenarios, multi-component score aggregation, realistic-by-default sim, composite goal types (`Sequence`/`Choice`), cross-platform bit-identical determinism. If a step in this plan is tempted to add any of these, stop and re-validate against §11 first.


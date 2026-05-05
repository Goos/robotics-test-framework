# Plan: Robotics Test Framework — v1 Implementation (plan v2)

**Date:** 2026-05-05 (revised after plan review round 1)
**Supersedes:** [`2026-05-05-robotics-test-framework-implementation-plan.md`](2026-05-05-robotics-test-framework-implementation-plan.md)
**Review addressed:** [`2026-05-05-robotics-test-framework-implementation-plan-review-1.md`](2026-05-05-robotics-test-framework-implementation-plan-review-1.md)
**Authoritative design:** [`2026-05-05-robotics-test-framework-design-v2.md`](2026-05-05-robotics-test-framework-design-v2.md) (amended same-day to add `PortReader<T>` and drop `T: Send`)

**Goal**: Build a Rust workspace implementing the v1 design — a kinematic-with-gravity robotics simulator with HAL-style port abstraction, deterministic test harness, code-defined `#[test]` scenarios, and out-of-process visualization. End state: a passing `#[test]` in which a hand-rolled state-machine controller picks up a block and places it in a bin, scored ≥ 0.9.

**Architecture**: 5-crate Cargo workspace (`rtf_core`, `rtf_sim`, `rtf_arm`, `rtf_harness`, `rtf_viz`) with strict one-way dependency arrows enforced by the workspace. `rtf_core` depends on nothing. The controller-under-test depends only on `rtf_core` + `rtf_arm` port type definitions, never on `rtf_sim` or `rtf_harness` — the HAL boundary made structural.

**Tech Stack**: Rust 2021, `nalgebra` (math), `smallvec` (Score breakdown), `rand` + `rand_pcg` (deterministic RNG), `rerun` SDK (visualizer backend, optional). No async, no threads in the tick path.

---

## Changes from plan v1

### Naming
- **All five workspace crates renamed to `rtf_*` prefix** (`rtf_core`, `rtf_sim`, `rtf_arm`, `rtf_harness`, `rtf_viz`). v1 had `core`, which collided with Rust's stdlib `core` crate (in the extern prelude in edition 2021) — every `use core::time::Time;` from a sibling crate would resolve to stdlib. Other crate names didn't collide, but uniformity is preferred.

### Architecture (consequential)
- **`PortReader<T>` trait added to `rtf_core::port` from Step 1.4** — was a back-fill in v1 Step 9.1. Cleaner: introduced when ports are first defined; both `PortRx<T>` and every fault wrapper implement it; controllers are generic over `R: PortReader<T>` from the start. Design v2 has been amended to include this trait.
- **`PortRx<T>` is now `!Clone`** — single-consumer guarantee. Documented at Step 1.4. Wrapping in fault combinators consumes the receiver.
- **`port<T>` no longer requires `T: Send`** — framework is single-threaded by design §10. Documented at Step 1.4.
- **`RunnableWorld` trait surface defined fully up front in Step 2.10** — was incrementally amended in v1 Step 4.3, with a circular default-method bug. Now: `publish_sensors`, `consume_actuators_and_integrate`, `snapshot`, `time` are required; `tick` is a default that calls `publish_sensors` then `consume_actuators_and_integrate`. The `EmptyWorld` test impls all four.
- **`set_gravity` is `ArmWorld`-specific, not on `RunnableWorld`** — was an unbacked claim in v1 Step 6.6. Now: gravity is configured via `ArmWorld::new(scene, spec, gravity_enabled)` (or `with_gravity` builder); `harness::run` does not see it.
- **`PdJointController` lives in `rtf_arm/src/examples_.rs` from Step 5.2** — was first committed under `examples/` then moved in v1 Step 5.4. Now placed correctly the first time.
- **`controller_events` cargo feature plumbed in Step 2.9** — design §7 requires "zero overhead unless enabled"; v1 plan defined the enum unconditionally. Now: `[features] controller_events = []` in `rtf_sim/Cargo.toml`; the enum and the `Recorder::record_event` default body are both gated.

### Step splits (each was over the 2–5 minute budget)
- **Step 3.11** (entire `ArmWorld::tick`) → **3.11a–e**: publish encoders, publish EE pose, consume velocity, gripper actuator + grasp transitions, integrate + time advance.
- **Step 6.3** (gravity_step) → **6.3a–c**: gravity-only integration, support snap, stable sort by `(q.z, id)`.
- **Step 8.3** (RerunRecorder) → **8.3a–e**: stream init + Sphere, Box, Capsule (or Mesh3D), Line, Label.
- **Step 7.4** (PickPlace state machine + e2e) → **7.4a–b**: state machine + unit tests, then e2e.

### Test correctness
- **Step 4.5 recorder test** uses `Rc<Cell<u32>>` to count `record` calls — v1's test was admittedly broken (counter consumed; assertion was `let _ = res;`).
- **Step 9.2 / 9.4 fault wrapper tests** use `Rc<dyn Clock>` (or `Rc<SimClock>`) consistently with the implementation. v1's tests passed `&dyn Clock` to a function specced to take `Rc<...>` — wouldn't compile.
- **Step 1.4 `take()` signature** is `&mut self` everywhere (consistent with `PortReader::take(&mut self)`).
- **Step 3.4 FK test** adds a non-zero-q assertion (`q = [PI/2, 0, 0]` → end-effector swings to +y) so a wrong composition order doesn't pass silently.
- **Step 5.2** `use std::collections::HashMap;` removed (unused; primes the implementer to use a forbidden type per design §10.2).
- **Step 9.5 `GaussianNoise` test** seeds a known-non-zero draw (no vanishing-probability flake).

### Coverage
- **New Step 0.4** adds workspace `[lints]` table (`clippy::disallowed_types` for `HashMap`, `Instant`, `thread::spawn`) — mechanizes the §10.2 audit.
- **New Step 7.0** creates `rtf_arm/src/test_helpers.rs` with `build_pick_and_place_world`, `block_id`, `bin_id` — referenced by Step 7.4 and several earlier integration tests but never defined in v1 plan.
- **New Step 3.7.5** adds `ArmWorld::sim_clock_handle() -> Rc<SimClock>` — needed by fault wrappers in Phase 9. v1 added it implicitly in 9.6.
- **New Step 2.5b** (one-line) tests that `Visualizable` is object-safe.
- **Final Verification** adds `cargo doc --workspace --no-deps`, `cargo fmt --check`, and explicit instructions for launching the rerun viewer.

---

## Conventions

- **Test commands** — every test step uses `cargo test -p <crate> <test_name> -- --exact` from the workspace root, or `cargo test --workspace` for the full suite. Run from `/Users/goos/AAI/robotics-test-framework`. Note: `<crate>` is the package name (`rtf_core`, `rtf_sim`, etc.), not the directory name.
- **Commit messages** — `[<crate-short-name>] Step N.M: <description>` using short names without the `rtf_` prefix (`[core]`, `[sim]`, `[arm]`, `[harness]`, `[viz]`) for brevity. Use git (`git add` + `git commit`); no `--no-verify`.
- **TDD discipline** — every implementation step is preceded by a failing test that targets exactly the new behavior; verify failure before writing the implementation, verify pass after.
- **Per-step time budget** — 2–5 minutes of focused work. If a step balloons, split it.
- **Determinism invariants** — `BTreeMap` over `HashMap` in tick path; no `std::time::Instant::now()` in `rtf_core`/`rtf_sim`; no thread spawning; explicit seeded RNG. (Design §10; mechanized via Step 0.4 lints.)
- **API spec** — when a step says "implement <Type>", consult the corresponding section of the design v2 doc for the canonical signature; this plan repeats only the load-bearing details.
- **Crate directory vs package name** — directories are `core/`, `sim/`, `arm/`, `harness/`, `viz/` for tidiness; package names in `Cargo.toml` are `rtf_core`, `rtf_sim`, `rtf_arm`, `rtf_harness`, `rtf_viz`. The `path = "../core"` style of dependency declaration is unaffected.

---

## Task Dependencies

| Phase | Steps | Depends On | Can Parallelize Within Phase? |
|-------|-------|------------|-------------------------------|
| 0. Workspace setup | 0.1–0.4 | — | No (serial) |
| 1. `rtf_core` crate | 1.1–1.10 | Phase 0 | Mostly no (each builds on prior types) |
| 2. `rtf_sim` foundations (no gravity) | 2.1–2.10 | Phase 1 | No |
| 3. `rtf_arm` crate | 3.1–3.12 (3.7.5 inserted; 3.11 split into 3.11a–e) | Phase 2 | No |
| 4. `rtf_harness` crate | 4.1–4.5 | Phase 3 | No |
| 5. First end-to-end test (no gravity) | 5.1–5.4 | Phase 4 | No |
| 6. Gravity-fall in `rtf_sim` | 6.1–6.7 (6.3 split into 6.3a–c) | Phase 5 | No |
| 7. Goals: `PickObject`, `PlaceInBin`, `Stack` | 7.0–7.4 (7.0 helpers added; 7.4 split into 7.4a–b) | Phase 6 | Yes (7.1, 7.2, 7.3 are independent goals) |
| 8. `rtf_viz` crate (rerun) | 8.1–8.4 (8.3 split into 8.3a–e) | Phase 5 | Independent of 6/7 |
| 9. `rtf_sim::faults` | 9.1–9.6 | Phase 2 | Independent of 6/7/8 |

Phases 8 and 9 can run in parallel with 6 and 7 once Phase 5 is done.

## Phase 0: Workspace setup

### Step 0.1: Initialize Cargo workspace with `rtf_*` package names

**Files**: `Cargo.toml` (new, workspace root), `core/Cargo.toml`, `sim/Cargo.toml`, `arm/Cargo.toml`, `harness/Cargo.toml`, `viz/Cargo.toml`, `core/src/lib.rs`, `sim/src/lib.rs`, `arm/src/lib.rs`, `harness/src/lib.rs`, `viz/src/lib.rs`.

```bash
cargo new --lib core --name rtf_core
cargo new --lib sim  --name rtf_sim
cargo new --lib arm  --name rtf_arm
cargo new --lib harness --name rtf_harness
cargo new --lib viz  --name rtf_viz
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

In each crate's `Cargo.toml`, set `edition.workspace = true`. Confirm the `name = "rtf_<x>"` line is set by `cargo new --name`. Do not add per-crate deps yet — added per crate in their own steps.

**Verify**: `cargo build --workspace` compiles five empty libs.

```bash
cargo build --workspace
git add Cargo.toml */Cargo.toml */src/lib.rs && git commit -m "[workspace] Step 0.1: Init Cargo workspace with rtf_* package names"
```

### Step 0.2: Configure dependency arrows

**Files**: `core/Cargo.toml`, `sim/Cargo.toml`, `arm/Cargo.toml`, `harness/Cargo.toml`, `viz/Cargo.toml`.

Wire the dependency arrows from design §3 (workspace deps via path). Use the package name as the dependency key:

- `core/Cargo.toml`: no internal deps.
- `sim/Cargo.toml`: `rtf_core = { path = "../core" }`, plus workspace deps `nalgebra`, `rand`, `rand_pcg`.
- `arm/Cargo.toml`: `rtf_core = { path = "../core" }`, `rtf_sim = { path = "../sim" }`, plus `nalgebra`.
- `harness/Cargo.toml`: `rtf_core = { path = "../core" }`, `rtf_sim = { path = "../sim" }`.
- `viz/Cargo.toml`: `rtf_sim = { path = "../sim" }`, plus workspace dep `rerun` (optional feature, see Step 8.1).

Sibling crate imports use the package name: `use rtf_core::time::Time;` from `rtf_sim`, etc.

**Verify**: `cargo build --workspace` still compiles.

```bash
cargo build --workspace
git add */Cargo.toml && git commit -m "[workspace] Step 0.2: Wire crate dependency arrows (rtf_*)"
```

### Step 0.3: Smoke test for `cargo test --workspace`

**File**: `core/src/lib.rs`.

Add one trivial passing test in `rtf_core` to confirm the test runner works:

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
# expect 1 test passed (in rtf_core)
git add core/src/lib.rs && git commit -m "[core] Step 0.3: Smoke test for workspace test runner"
```

### Step 0.4: Mechanize the determinism audit via workspace `[lints]`

**File**: workspace-root `Cargo.toml` (extend).

Add a workspace-level lints table that enforces design §10.2's forbidden-types list:

```toml
[workspace.lints.clippy]
disallowed_types = { level = "deny", priority = 1 }

[workspace.lints.rust]
# Reserve room for any future workspace-wide deny lints.

# Per-clippy.toml at the workspace root:
```

And `clippy.toml` (new, workspace root):

```toml
disallowed-types = [
    { path = "std::collections::HashMap", reason = "use BTreeMap; HashMap iteration is nondeterministic (design §10.2)" },
    { path = "std::collections::HashSet", reason = "use BTreeSet (design §10.2)" },
    { path = "std::time::Instant", reason = "inject Clock; Instant::now() is forbidden in core/sim (design §10.2)" },
]

disallowed-methods = [
    { path = "std::thread::spawn", reason = "single-threaded only (design §10.2)" },
]
```

In each crate's `lib.rs`, add `#![deny(clippy::disallowed_types, clippy::disallowed_methods)]` (or rely on the workspace-lints inheritance — Rust 1.74+ supports `[lints] workspace = true` in each crate's `Cargo.toml`).

In each crate's `Cargo.toml`, add:

```toml
[lints]
workspace = true
```

**Verify** (failing test: deliberately try to use `HashMap` and watch clippy fail):

Create a temporary `core/src/_lint_test.rs`:

```rust
// Should FAIL clippy with disallowed-types.
use std::collections::HashMap;
pub fn _bad() -> HashMap<u32, u32> { HashMap::new() }
```

Run:

```bash
cargo clippy --workspace -- -D warnings
# expect failure citing disallowed_types HashMap
```

Then **delete** `core/src/_lint_test.rs` and re-run; expect success. Commit:

```bash
rm core/src/_lint_test.rs
cargo clippy --workspace -- -D warnings
git add Cargo.toml clippy.toml */Cargo.toml && git commit -m "[workspace] Step 0.4: Workspace lints enforce determinism invariants"
```

---

## Phase 1: `rtf_core` crate

Each step adds a module file in `core/src/`, with `pub mod ...;` in `core/src/lib.rs` and a re-export. Re-export every new public type from `core/src/lib.rs` as part of the same step. **Inside `core`, intra-crate paths use `crate::` and `super::`, not `rtf_core::`.** Sibling crates (in Phase 2+) use `rtf_core::`.

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

Run: `cargo test -p rtf_core time::tests` → expect compile failure.

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

Implement: `pub trait Clock { fn now(&self) -> Time; }`, plus `pub struct FakeClock { now: Cell<Time> }`. `FakeClock::new`, `FakeClock::advance(&self, dt: Duration)`, `impl Clock for FakeClock`. (Single-threaded; `Cell` is sufficient.)

Run: `cargo test -p rtf_core clock::tests`. Commit:

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

### Step 1.4: `port<T>()` + `PortTx` + `PortRx` + `PortReader<T>` trait (LatestWins, single-slot)

**File**: `core/src/port.rs` (new); export from `lib.rs`.

This step combines two design v2 elements: the concrete `(PortTx, PortRx)` types AND the `PortReader<T>` trait that fault wrappers will implement (design v2 §4 amendment). They land together so controllers from Phase 5 onward can be generic over `R: PortReader<T>` from day one.

**Design v2 deviations to be aware of (and enforced by these signatures):**
- `port<T: Clone + 'static>()` — `T: Send` is intentionally omitted (design §10 single-threaded guarantee).
- `PortRx<T>` is intentionally **not** `Clone` (single-consumer guarantee). Wrapping a port in a fault combinator consumes the receiver.

Failing test (covers send-overwrites, latest-vs-take semantics, AND `PortReader<T>` impl on `PortRx<T>`):

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

    #[test]
    fn port_rx_implements_port_reader_trait() {
        let (tx, mut rx) = port::<i32>();
        tx.send(42);
        // Polymorphic over PortReader<T>:
        fn pull<R: PortReader<i32>>(r: &mut R) -> Option<i32> { r.take() }
        assert_eq!(pull(&mut rx), Some(42));
    }
}
```

(Optional: a compile-fail doc-test or note that `PortRx<T>` is `!Clone` — `let _: () = (|| { let (_, rx) = port::<i32>(); let _ = rx.clone(); })();` should not compile. Easiest to enforce via not-deriving `Clone`; no separate test needed.)

Implement: single-threaded, single-slot using `Rc<RefCell<Option<T>>>`. `T: Clone` for `latest()`.

```rust
use std::cell::RefCell;
use std::rc::Rc;

pub trait PortReader<T> {
    fn latest(&self) -> Option<T>;
    fn take(&mut self) -> Option<T>;
}

pub struct PortTx<T> { slot: Rc<RefCell<Option<T>>> }
pub struct PortRx<T> { slot: Rc<RefCell<Option<T>>> }
// Intentionally NOT deriving Clone for PortRx<T>.

pub fn port<T: Clone + 'static>() -> (PortTx<T>, PortRx<T>) {
    let slot = Rc::new(RefCell::new(None));
    (PortTx { slot: Rc::clone(&slot) }, PortRx { slot })
}

impl<T> PortTx<T> {
    pub fn send(&self, v: T) { *self.slot.borrow_mut() = Some(v); }
}

impl<T: Clone> PortRx<T> {
    pub fn latest(&self) -> Option<T> { self.slot.borrow().clone() }
    pub fn take(&mut self) -> Option<T> { self.slot.borrow_mut().take() }
}

impl<T: Clone> PortReader<T> for PortRx<T> {
    fn latest(&self) -> Option<T> { Self::latest(self) }
    fn take(&mut self) -> Option<T> { Self::take(self) }
}
```

Note: both `PortRx::take` and `PortReader::take` use `&mut self`. Although `RefCell::borrow_mut()` only requires `&self`, taking `&mut self` enforces the single-consumer invariant at the borrow checker level (cannot have two outstanding `&mut` references to the same `PortRx`).

Commit:

```bash
git add core/src/port.rs core/src/lib.rs && git commit -m "[core] Step 1.4: port<T>() + PortTx + PortRx + PortReader<T> trait"
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

Implement: `pub trait WorldView {}`. Commit `[core] Step 1.6: WorldView marker trait`.

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

Implement per design §4: `Score { value: f64, breakdown: SmallVec<[(&'static str, f64); 4]> }` + `Score::new`, `with_component`, `Default`.

Commit:

```bash
git add core/src/score.rs core/Cargo.toml core/src/lib.rs && git commit -m "[core] Step 1.7: Score struct with breakdown field"
```

### Step 1.8: `Goal` trait

**File**: `core/src/goal.rs` (new); export from `lib.rs`.

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::{score::Score, time::Time, world_view::WorldView};
    struct W;
    impl WorldView for W {}
    struct AlwaysOne;
    impl Goal<W> for AlwaysOne { fn evaluate(&self, _w: &W) -> Score { Score::new(1.0) } }
    #[test]
    fn defaults_for_tick_and_is_complete() {
        let g = AlwaysOne;
        assert!(!g.is_complete(&W));
        assert_eq!(g.evaluate(&W).value, 1.0);
    }
}
```

Implement per design §4. Commit `[core] Step 1.8: Goal trait with default tick + is_complete`.

### Step 1.9: `SensorReading` marker trait (HAL §9.4)

**File**: `core/src/sensor_reading.rs` (new); export from `lib.rs`.

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::Time;
    struct Foo { sampled_at: Time }
    impl SensorReading for Foo { fn sampled_at(&self) -> Time { self.sampled_at } }
    #[test]
    fn sample_timestamp_accessible() {
        let r = Foo { sampled_at: Time::from_millis(42) };
        assert_eq!(r.sampled_at(), Time::from_millis(42));
    }
}
```

Implement: `pub trait SensorReading { fn sampled_at(&self) -> Time; }`. Commit `[core] Step 1.9: SensorReading marker trait (HAL §9.4)`.

### Step 1.10: Final `core/src/lib.rs` re-exports + clippy clean

**File**: `core/src/lib.rs`.

Ensure all public types are re-exported at the crate root. Then:

```bash
cargo clippy -p rtf_core -- -D warnings
cargo test -p rtf_core
git add core/src/lib.rs && git commit -m "[core] Step 1.10: Public re-exports; clippy clean"
```

---

## Phase 2: `rtf_sim` foundations (no gravity yet)

Each step adds a module file in `sim/src/`, with `pub mod ...;` in `sim/src/lib.rs` and a re-export. Sibling-crate paths use `rtf_core::...`.

### Step 2.1: `EntityId` + `Shape` primitives

**File**: `sim/src/entity.rs` (new); `sim/src/shape.rs` (new); `sim/src/lib.rs`.

Failing test (entity):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn entity_ids_distinguish_kinds() {
        assert_ne!(EntityId::Object(1), EntityId::Fixture(1));
    }
}
```

Failing test (shape):

```rust
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

Implement: `pub enum EntityId { Object(u32), Fixture(u32) }` with sort-stable derives. `pub enum Shape { Sphere { radius: f32 }, Aabb { half_extents: Vector3<f32> }, Cylinder { radius: f32, half_height: f32 } }` + `pub fn half_height_z(&self) -> f32`.

Commit `[sim] Step 2.1: EntityId + Shape primitives`.

### Step 2.2: `Object`, `Fixture`, `ObjectState`

**File**: `sim/src/object.rs` (new); `sim/src/fixture.rs` (new); `sim/src/lib.rs`.

```rust
// sim/src/object.rs
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Isometry3;
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

Implement per design §5.2: `ObjectId(u32)` + `ObjectState::{Free, Grasped { by: ArmRef }, Settled { on: SupportId }}` + `ArmRef(u32)` + `SupportId::{Fixture(u32), Object(u32)}`. `Object { id, pose, shape, mass, graspable, state, lin_vel: Vector3<f32> }`. `Fixture { id: u32, pose, shape, is_support: bool }`.

Commit `[sim] Step 2.2: Object + Fixture + ObjectState`.

### Step 2.3: `Scene` aggregation + deterministic id assignment

**File**: `sim/src/scene.rs` (new); `sim/src/lib.rs`.

```rust
#[cfg(test)]
mod tests {
    use super::*;
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
        let ids: Vec<_> = s.objects().map(|(_, o)| o.id).collect();
        assert!(ids.windows(2).all(|w| w[0].0 < w[1].0));
    }
}
```

Implement: `Scene { objects: BTreeMap<ObjectId, Object>, fixtures: BTreeMap<u32, Fixture>, next_object_id: u32, next_fixture_id: u32, rng: rand_pcg::Pcg64 }` + accessors (`objects(&self) -> impl Iterator<Item = (&ObjectId, &Object)>`, `fixtures(&self) -> impl Iterator<...>`, `add_object_default` test helper).

Commit `[sim] Step 2.3: Scene with deterministic id assignment + BTreeMap storage`.

### Step 2.4: `Color`, `Primitive`, `SceneSnapshot`

**File**: `sim/src/primitive.rs` (new); `sim/src/lib.rs`.

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Isometry3;
    use crate::entity::EntityId;
    use rtf_core::time::Time;
    #[test]
    fn snapshot_holds_items_in_insertion_order() {
        let snap = SceneSnapshot {
            t: Time::from_nanos(0),
            items: vec![
                (EntityId::Object(1), Primitive::Sphere {
                    pose: Isometry3::identity(), radius: 0.05, color: Color::WHITE
                }),
            ],
        };
        assert_eq!(snap.items.len(), 1);
    }
}
```

Implement per design §7. Commit `[sim] Step 2.4: Primitive + Color + SceneSnapshot`.

### Step 2.5: `Visualizable` trait + impl for `Object`

**File**: `sim/src/visualizable.rs` (new); `sim/src/object.rs` (extend); `sim/src/lib.rs`.

Failing test (in `object.rs`):

```rust
#[test]
fn object_appends_one_primitive_matching_shape() {
    use super::*;
    use nalgebra::Isometry3;
    use crate::primitive::Primitive;
    use crate::visualizable::Visualizable;
    let o = Object::new(ObjectId(1), Isometry3::identity(), Shape::Sphere{radius:0.05}, 0.1, true);
    let mut out = Vec::new();
    o.append_primitives(&mut out);
    assert_eq!(out.len(), 1);
    assert!(matches!(out[0].1, Primitive::Sphere { .. }));
}
```

Implement: `pub trait Visualizable { fn append_primitives(&self, out: &mut Vec<(EntityId, Primitive)>); }` (no `Self: Sized`, no generic methods, no associated types — object-safe). `impl Visualizable for Object` matches on `self.shape` to emit one primitive.

Commit `[sim] Step 2.5: Visualizable trait + impl for Object`.

### Step 2.5b: Object-safety check for `Visualizable` (one-line guard)

**File**: `sim/src/visualizable.rs` (extend tests).

```rust
#[test]
fn visualizable_is_object_safe() {
    fn _it_compiles(_: &dyn Visualizable) {}
    // Body intentionally empty; success is the call type-checking.
}
```

If anyone later adds a generic method or `Self: Sized` to `Visualizable`, this fails to compile with a clear error. Commit `[sim] Step 2.5b: object-safety guard test for Visualizable`.

### Step 2.6: `Visualizable` impl for `Fixture`

**File**: `sim/src/fixture.rs` (extend). Mirror Step 2.5. Commit `[sim] Step 2.6: Visualizable for Fixture`.

### Step 2.7: `RateScheduler` (phase accumulator with documented ±1-tick jitter)

**File**: `sim/src/rate_scheduler.rs` (new); `sim/src/lib.rs`.

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

Implement per design §5.6: `RateScheduler { period_ns: i64, accumulator_ns: i64 }` + `new_hz(u32)` + `tick(&mut self, dt_ns: i64) -> bool`.

Commit `[sim] Step 2.7: RateScheduler (phase accumulator)`.

### Step 2.8: `SimClock` (sim-controlled clock)

**File**: `sim/src/sim_clock.rs` (new); `sim/src/lib.rs`.

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use rtf_core::clock::Clock;
    use rtf_core::time::{Time, Duration};
    #[test]
    fn sim_clock_advances_via_advance() {
        let c = SimClock::new();
        assert_eq!(c.now(), Time::from_nanos(0));
        c.advance(Duration::from_millis(3));
        assert_eq!(c.now(), Time::from_millis(3));
    }
}
```

Implement: `pub struct SimClock { now: Cell<Time> }` + `impl Clock`. (Single-threaded; safe.) Also expose `pub fn handle(self: &Rc<Self>) -> Rc<Self> { Rc::clone(self) }` for fault wrappers in Phase 9.

Commit `[sim] Step 2.8: SimClock`.

### Step 2.9: `Recorder` trait + `NullRecorder` + `controller_events` feature

**File**: `sim/src/recorder.rs` (new); `sim/src/lib.rs`; `sim/Cargo.toml`.

Add a `controller_events` feature in `sim/Cargo.toml`:

```toml
[features]
default = []
controller_events = []
```

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use rtf_core::time::Time;
    use crate::primitive::SceneSnapshot;
    #[test]
    fn null_recorder_is_a_no_op() {
        let mut r = NullRecorder;
        let snap = SceneSnapshot { t: Time::from_nanos(0), items: vec![] };
        r.record(&snap);
    }
}
```

Implement:

```rust
pub trait Recorder {
    fn record(&mut self, snapshot: &SceneSnapshot);
    #[cfg(feature = "controller_events")]
    fn record_event(&mut self, _event: &ControllerEvent) {}
}

pub struct NullRecorder;
impl Recorder for NullRecorder {
    fn record(&mut self, _: &SceneSnapshot) {}
    #[cfg(feature = "controller_events")]
    fn record_event(&mut self, _: &ControllerEvent) {}
}

#[cfg(feature = "controller_events")]
pub enum ControllerEvent {
    PortSend  { port_id: rtf_core::port_id::PortId, t: rtf_core::time::Time, type_name: &'static str },
    PortRecv  { port_id: rtf_core::port_id::PortId, t: rtf_core::time::Time, type_name: &'static str },
    ControllerError { t: rtf_core::time::Time, kind: rtf_core::controller::ControlErrorKind, detail: String },
}
```

Per design §7: zero overhead unless the feature is enabled. The tick-loop instrumentation that calls `record_event` (Phase 4) must be `#[cfg(feature = "controller_events")]` too.

Commit `[sim] Step 2.9: Recorder trait + NullRecorder + controller_events feature gate`.

### Step 2.10: `RunnableWorld` trait — full surface up front

**File**: `sim/src/runnable_world.rs` (new); `sim/src/lib.rs`.

This step defines the entire `RunnableWorld` trait surface so `harness::run` (Phase 4) does not need to amend it. Required methods: `publish_sensors`, `consume_actuators_and_integrate`, `snapshot`, `time`. `tick` is provided as a default that calls `publish_sensors` then `consume_actuators_and_integrate(dt)` — useful for direct callers (no controller in between) but not used by `harness::run`.

Failing test (no-op world implements all four required methods + uses default `tick`):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use rtf_core::{time::{Time, Duration}, world_view::WorldView};
    use crate::primitive::SceneSnapshot;

    struct EmptyWorld { t: Time }
    impl WorldView for EmptyWorld {}
    impl RunnableWorld for EmptyWorld {
        fn publish_sensors(&mut self) { /* no sensors */ }
        fn consume_actuators_and_integrate(&mut self, dt: Duration) {
            self.t = self.t + dt;
        }
        fn snapshot(&self) -> SceneSnapshot { SceneSnapshot { t: self.t, items: vec![] } }
        fn time(&self) -> Time { self.t }
    }

    #[test]
    fn default_tick_advances_time_via_consume() {
        let mut w = EmptyWorld { t: Time::from_nanos(0) };
        w.tick(Duration::from_millis(1)).unwrap();   // default impl
        assert_eq!(w.time(), Time::from_millis(1));
    }
}
```

Implement:

```rust
pub trait RunnableWorld: rtf_core::world_view::WorldView {
    fn publish_sensors(&mut self);
    fn consume_actuators_and_integrate(&mut self, dt: rtf_core::time::Duration);
    fn snapshot(&self) -> SceneSnapshot;
    fn time(&self) -> rtf_core::time::Time;

    /// Default tick: publish, then consume. Use when there's no controller in the loop
    /// (e.g., for tests of pure world dynamics). `harness::run` interleaves a controller
    /// step between these two calls instead of using this default.
    fn tick(&mut self, dt: rtf_core::time::Duration) -> Result<(), SimError> {
        self.publish_sensors();
        self.consume_actuators_and_integrate(dt);
        Ok(())
    }
}

pub struct SimError { pub detail: std::borrow::Cow<'static, str> }
```

Commit `[sim] Step 2.10: RunnableWorld trait with full surface (publish/consume/snapshot/time + default tick)`.

End of Phase 2 verification: `cargo test --workspace && cargo clippy --workspace -- -D warnings`.

---

## Phase 3: `rtf_arm` crate

### Step 3.1: `JointSpec`, `GripperSpec`, `ArmSpec`

**File**: `arm/src/spec.rs` (new); `arm/src/lib.rs`.

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

Implement per design §5.3. Commit `[arm] Step 3.1: ArmSpec + JointSpec + GripperSpec`.

### Step 3.2: `ArmState`

**File**: `arm/src/state.rs` (new); `arm/src/lib.rs`. Failing test ensures `ArmState::zeros(n)` returns zeroed q/q_dot, open gripper, no grasp. Commit `[arm] Step 3.2: ArmState`.

### Step 3.3: Single-joint forward kinematics

**File**: `arm/src/fk.rs` (new); `arm/src/lib.rs`.

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

Implement: `pub fn joint_transform(spec: &JointSpec, q: f32) -> Isometry3<f32>`.

Commit `[arm] Step 3.3: Single-joint forward kinematics`.

### Step 3.4: Multi-joint chain forward kinematics — including non-zero-q test

**File**: `arm/src/fk.rs` (extend).

Two failing tests — the non-zero-q test is what catches wrong composition order:

```rust
#[test]
fn three_joint_chain_at_zero_q_yields_sum_of_offsets() {
    use crate::spec::{ArmSpec, JointSpec, GripperSpec};
    use nalgebra::{Vector3, Isometry3};
    let spec = ArmSpec {
        joints: vec![JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-3.2, 3.2) }; 3],
        link_offsets: vec![Isometry3::translation(0.5, 0.0, 0.0); 3],
        gripper: GripperSpec { proximity_threshold: 0.01, max_grasp_size: 0.05 },
    };
    let q = vec![0.0; 3];
    let ee = forward_kinematics(&spec, &q);
    assert!((ee.translation.x - 1.5).abs() < 1e-5);
    assert!(ee.translation.y.abs() < 1e-5);
}

#[test]
fn first_joint_pi_over_2_swings_chain_to_plus_y() {
    use crate::spec::{ArmSpec, JointSpec, GripperSpec};
    use nalgebra::{Vector3, Isometry3};
    let spec = ArmSpec {
        joints: vec![JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-3.2, 3.2) }; 3],
        link_offsets: vec![Isometry3::translation(0.5, 0.0, 0.0); 3],
        gripper: GripperSpec { proximity_threshold: 0.01, max_grasp_size: 0.05 },
    };
    // Only the first joint rotated 90°: the entire chain swings to +y axis.
    let q = vec![std::f32::consts::FRAC_PI_2, 0.0, 0.0];
    let ee = forward_kinematics(&spec, &q);
    assert!(ee.translation.x.abs() < 1e-4, "x should be ~0, got {}", ee.translation.x);
    assert!((ee.translation.y - 1.5).abs() < 1e-4, "y should be ~1.5, got {}", ee.translation.y);
}
```

Implement: `pub fn forward_kinematics(spec: &ArmSpec, q: &[f32]) -> Isometry3<f32>` — fold over joints, accumulating `acc = acc * joint_transform(joint, q[i]) * link_offsets[i]`. The non-zero-q test will catch a wrong composition order (e.g., applying `link_offset` before `joint_transform`).

Commit `[arm] Step 3.4: Multi-joint chain FK + non-zero-q test`.

### Step 3.5: Port type definitions (with `sampled_at`)

**File**: `arm/src/ports.rs` (new); `arm/src/lib.rs`.

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use rtf_core::{sensor_reading::SensorReading, time::Time};
    #[test]
    fn joint_encoder_reading_exposes_sample_time() {
        let r = JointEncoderReading {
            joint: JointId(0), q: 0.5, q_dot: 0.1, sampled_at: Time::from_millis(7),
        };
        assert_eq!(r.sampled_at(), Time::from_millis(7));
    }
}
```

Implement: `JointId(u32)` + four reading/command types per design §5.6 and §6:

```rust
pub struct JointEncoderReading { pub joint: JointId, pub q: f32, pub q_dot: f32, pub sampled_at: Time }
pub struct EePoseReading       { pub pose: Isometry3<f32>, pub sampled_at: Time }
pub struct JointVelocityCommand{ pub joint: JointId, pub q_dot_target: f32 }
pub struct GripperCommand      { pub close: bool }
```

`impl SensorReading` for the two reading types. All `#[derive(Clone, Debug)]`.

Commit `[arm] Step 3.5: Arm port type definitions`.

### Step 3.6: `Arm` struct + `Visualizable` impl (capsules per link, box for gripper)

**File**: `arm/src/arm.rs` (new); `arm/src/viz.rs` (new); `arm/src/lib.rs`.

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::{spec::*, state::*, arm::Arm};
    use nalgebra::{Vector3, Isometry3};
    use rtf_sim::{primitive::Primitive, entity::EntityId, visualizable::Visualizable};

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

Implement: `pub struct Arm { pub spec: ArmSpec, pub state: ArmState, pub id: u32 }`. `impl Visualizable for Arm` walks the chain using `joint_transform` accumulator, emits `Primitive::Capsule` per link and one `Primitive::Box` at the EE pose.

Commit `[arm] Step 3.6: Arm struct + Visualizable impl`.

### Step 3.7: `ArmWorld` skeleton with gravity flag

**File**: `arm/src/world.rs` (new); `arm/src/lib.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use rtf_sim::scene::Scene;
    #[test]
    fn arm_world_is_constructible_with_gravity_default_on() {
        let world = ArmWorld::new(Scene::new(0), simple_spec(), /* gravity */ true);
        assert!(world.gravity_enabled);
        assert_eq!(world.arm.state.q.len(), simple_spec().joints.len());
        assert_eq!(world.time(), rtf_core::time::Time::from_nanos(0));
    }
    fn simple_spec() -> crate::spec::ArmSpec { /* 2-joint helper */ }
}
```

Implement skeleton:

```rust
use std::rc::Rc;

pub struct ArmWorld {
    pub scene: Scene,
    pub arm: Arm,
    pub gravity_enabled: bool,
    sim_time: rtf_core::time::Time,
    sim_clock: Rc<rtf_sim::sim_clock::SimClock>,
    next_port_id: u32,
    sensors_joint_encoder:    BTreeMap<rtf_core::port_id::PortId, EncoderPublisher>,
    sensors_ee_pose:          BTreeMap<rtf_core::port_id::PortId, EePosePublisher>,
    actuators_joint_velocity: BTreeMap<rtf_core::port_id::PortId, JointVelocityConsumer>,
    actuators_gripper:        BTreeMap<rtf_core::port_id::PortId, GripperConsumer>,
}

impl ArmWorld {
    pub fn new(scene: Scene, spec: ArmSpec, gravity_enabled: bool) -> Self { /* ... */ }
}
```

Define stub `EncoderPublisher`, `EePosePublisher`, `JointVelocityConsumer`, `GripperConsumer` as `pub(crate) struct`s holding the corresponding `PortTx`/`PortRx` and a `RateScheduler` (for sensors).

Commit `[arm] Step 3.7: ArmWorld skeleton with port registries + gravity flag`.

### Step 3.7.5: `ArmWorld::sim_clock_handle()` accessor

**File**: `arm/src/world.rs` (extend).

```rust
#[test]
fn sim_clock_handle_returns_sharable_rc() {
    let world = ArmWorld::new(Scene::new(0), simple_spec(), true);
    let h1 = world.sim_clock_handle();
    let h2 = world.sim_clock_handle();
    assert!(Rc::ptr_eq(&h1, &h2));
}
```

Implement: `pub fn sim_clock_handle(&self) -> Rc<rtf_sim::sim_clock::SimClock> { Rc::clone(&self.sim_clock) }`. Used by Phase 9 fault wrappers (Delay, StaleData) that need shared access to sim time.

Commit `[arm] Step 3.7.5: sim_clock_handle accessor`.

### Step 3.8: `attach_joint_encoder_sensor`

**File**: `arm/src/world.rs` (extend).

```rust
#[test]
fn attaching_two_encoders_yields_distinct_port_ids() {
    let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
    let _rx_a = world.attach_joint_encoder_sensor(JointId(0), RateHz::new(1000));
    let _rx_b = world.attach_joint_encoder_sensor(JointId(1), RateHz::new(1000));
    assert_eq!(world.sensors_joint_encoder.len(), 2);
}
```

Implement: `pub fn attach_joint_encoder_sensor(&mut self, joint: JointId, rate: RateHz) -> PortRx<JointEncoderReading>` — construct port, take next PortId, build EncoderPublisher, insert, return rx. Add `pub struct RateHz(pub u32); impl RateHz { pub fn new(hz: u32) -> Self }`.

Commit `[arm] Step 3.8: attach_joint_encoder_sensor`.

### Step 3.9: `attach_joint_velocity_actuator`

**File**: `arm/src/world.rs` (extend). Same pattern; consumer holds `PortRx`. Failing test asserts a fresh `PortTx` is returned and the world's actuator map has a new entry. Commit `[arm] Step 3.9: attach_joint_velocity_actuator`.

### Step 3.10: `attach_gripper_actuator` and `attach_ee_pose_sensor`

**File**: `arm/src/world.rs` (extend). Both follow the established pattern. Commit `[arm] Step 3.10: attach_gripper_actuator + attach_ee_pose_sensor`.

### Step 3.11a: `publish_sensors` for joint encoders

**File**: `arm/src/world.rs` (extend).

```rust
#[test]
fn publish_sensors_emits_encoder_at_rate() {
    use rtf_sim::sim_clock::SimClock;
    let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
    let rx = world.attach_joint_encoder_sensor(JointId(0), RateHz::new(1000));
    world.publish_sensors();    // first tick: scheduler should fire (period = 1ms, accumulator starts at 0 — depends on scheduler init)
    // Either way, after publishing for 1ms of accumulated time, the encoder should have fired once.
    // For determinism, prime the scheduler so it fires on the first publish_sensors call.
    assert!(rx.latest().is_some());
}
```

Implement: iterate `sensors_joint_encoder` in PortId order; for each `EncoderPublisher`, call `scheduler.tick(dt_since_last_publish)` (need to track this — or pass dt explicitly; cleanest: `publish_sensors(&mut self, dt: Duration)` — but trait says no arg. Resolution: store last-publish time on the world; compute dt internally). Build a `JointEncoderReading` from `arm.state.q[joint]` and `q_dot[joint]`, send via tx.

Commit `[arm] Step 3.11a: publish_sensors for joint encoders`.

### Step 3.11b: publish EE pose

**File**: `arm/src/world.rs` (extend).

```rust
#[test]
fn publish_sensors_emits_ee_pose() {
    let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
    let rx = world.attach_ee_pose_sensor(RateHz::new(100));
    world.publish_sensors();
    let r = rx.latest().expect("ee pose published");
    assert_eq!(r.sampled_at, world.time());
}
```

Implement: extend `publish_sensors` to also iterate `sensors_ee_pose`. Compute `forward_kinematics(&self.arm.spec, &self.arm.state.q)` once; cache for the tick. Send to each EE-pose publisher whose scheduler fires.

Commit `[arm] Step 3.11b: publish EE pose`.

### Step 3.11c: `consume_actuators_and_integrate` — joint velocity command application

**File**: `arm/src/world.rs` (extend).

```rust
#[test]
fn velocity_command_advances_joint_position() {
    use rtf_core::time::Duration;
    let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
    let tx = world.attach_joint_velocity_actuator(JointId(0));
    tx.send(JointVelocityCommand { joint: JointId(0), q_dot_target: 1.0 });
    world.consume_actuators_and_integrate(Duration::from_millis(10));
    assert!(world.arm.state.q[0] > 0.0);
}
```

Implement: iterate `actuators_joint_velocity` in PortId order; for each, take latest, set `arm.state.q_dot[joint]`. Then integrate: `arm.state.q[i] += arm.state.q_dot[i] * dt_s for each joint`. Advance `sim_time`. **Do NOT yet do gravity or grasp updates here — separate sub-steps.**

Commit `[arm] Step 3.11c: consume_actuators (velocity) + integrate joint state`.

### Step 3.11d: gripper actuator + grasp transitions

**File**: `arm/src/world.rs` (extend).

```rust
#[test]
fn closing_gripper_near_graspable_object_attaches_it() {
    let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
    // Place a graspable object at the EE pose
    let block = world.scene.add_object_default_at(world.ee_pose());
    let g_tx = world.attach_gripper_actuator();
    g_tx.send(GripperCommand { close: true });
    world.consume_actuators_and_integrate(rtf_core::time::Duration::from_millis(1));
    assert_eq!(world.arm.state.grasped, Some(block));
}
```

Implement: in `consume_actuators_and_integrate`, after velocity application, iterate `actuators_gripper`; take latest gripper command. Update `arm.state.gripper_closed`. Run grasp logic per design §5.4: if closing + nothing held + a graspable object's pose is within `gripper.proximity_threshold` of EE → attach (set `arm.state.grasped`, set `object.state = Grasped { by: ArmRef(arm.id) }`). If gripper opens → detach (set `arm.state.grasped = None`, set `object.state = Free`).

Commit `[arm] Step 3.11d: gripper consume + grasp transitions`.

### Step 3.11e: grasped-object pose follows EE

**File**: `arm/src/world.rs` (extend).

```rust
#[test]
fn grasped_object_pose_tracks_ee() {
    let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
    let block = world.scene.add_object_default_at(world.ee_pose());
    let v_tx = world.attach_joint_velocity_actuator(JointId(0));
    let g_tx = world.attach_gripper_actuator();
    g_tx.send(GripperCommand { close: true });
    world.consume_actuators_and_integrate(rtf_core::time::Duration::from_millis(1));
    let pose_before = world.scene.object(block).unwrap().pose;
    v_tx.send(JointVelocityCommand { joint: JointId(0), q_dot_target: 1.0 });
    world.consume_actuators_and_integrate(rtf_core::time::Duration::from_millis(50));
    let pose_after = world.scene.object(block).unwrap().pose;
    assert_ne!(pose_before.translation.vector, pose_after.translation.vector);
    // Object pose equals EE pose:
    let ee = world.ee_pose();
    assert!((pose_after.translation.vector - ee.translation.vector).norm() < 1e-4);
}
```

Implement: at the end of `consume_actuators_and_integrate`, if `arm.state.grasped == Some(id)`, set `scene.object_mut(id).pose = self.ee_pose()`.

Commit `[arm] Step 3.11e: grasped object follows EE`.

### Step 3.12: `impl RunnableWorld for ArmWorld` + `WorldView`

**File**: `arm/src/world.rs` (extend).

The trait surface is already correctly defined in Step 2.10, so this step just provides the impl that delegates to the methods built in 3.11a–e and adds `snapshot` + `time` + `WorldView`.

```rust
#[test]
fn arm_world_implements_runnable_world() {
    use rtf_sim::runnable_world::RunnableWorld;
    let world = ArmWorld::new(Scene::new(0), simple_spec(), true);
    let snap = world.snapshot();
    assert_eq!(snap.t, world.time());
    // Snapshot has at least the arm primitives (3 capsules + 1 box for 3-joint arm).
    assert!(snap.items.len() >= 4);
}
```

Implement:

```rust
impl rtf_core::world_view::WorldView for ArmWorld {}

impl RunnableWorld for ArmWorld {
    fn publish_sensors(&mut self) { /* delegates to inherent methods from 3.11a-b */ }
    fn consume_actuators_and_integrate(&mut self, dt: Duration) {
        /* from 3.11c-e */
    }
    fn snapshot(&self) -> SceneSnapshot {
        let mut items = Vec::new();
        for (_, fix) in self.scene.fixtures() { fix.append_primitives(&mut items); }
        for (_, obj) in self.scene.objects()  { obj.append_primitives(&mut items); }
        self.arm.append_primitives(&mut items);
        SceneSnapshot { t: self.sim_time, items }
    }
    fn time(&self) -> Time { self.sim_time }
}
```

Add domain-specific accessors to `ArmWorld` itself (not in `WorldView` — that's a marker): `pub fn ee_pose(&self) -> Isometry3<f32>`, `pub fn object(&self, id: ObjectId) -> Option<&Object>`. Goals use these via `&ArmWorld`.

Commit `[arm] Step 3.12: impl RunnableWorld + WorldView for ArmWorld`.

End of Phase 3 verification: `cargo test --workspace && cargo clippy --workspace -- -D warnings`.

---

## Phase 4: `rtf_harness` crate

### Step 4.1: `RunConfig`, `RunResult`, `Termination`

**File**: `harness/src/types.rs` (new); `harness/src/lib.rs`.

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use rtf_sim::recorder::NullRecorder;
    use rtf_core::time::Duration;
    #[test]
    fn run_config_defaults() {
        let cfg: RunConfig<NullRecorder> = RunConfig::default();
        assert_eq!(cfg.tick_rate_hz, 1000);
        assert_eq!(cfg.deadline, Duration::from_secs(10));
    }
}
```

Implement per design §6:

```rust
pub struct RunConfig<R: rtf_sim::recorder::Recorder = rtf_sim::recorder::NullRecorder> {
    pub tick_rate_hz: u32,
    pub deadline: rtf_core::time::Duration,
    pub seed: u64,
    pub recorder: R,
}

pub struct RunResult {
    pub score: rtf_core::score::Score,
    pub final_time: rtf_core::time::Time,
    pub terminated_by: Termination,
}

pub enum Termination {
    GoalComplete,
    Deadline,
    ControllerError(rtf_core::controller::ControlError),
}

impl Default for RunConfig<rtf_sim::recorder::NullRecorder> {
    fn default() -> Self { Self {
        tick_rate_hz: 1000,
        deadline: rtf_core::time::Duration::from_secs(10),
        seed: 0,
        recorder: rtf_sim::recorder::NullRecorder,
    }}
}
```

Builder methods: `with_deadline`, `with_seed`, `with_recorder`, `with_tick_rate`. **Note:** there is no `with_gravity` here — gravity is `ArmWorld`-specific and configured at world construction (Step 3.7). `harness::run` is robot-agnostic.

Commit `[harness] Step 4.1: RunConfig + RunResult + Termination`.

### Step 4.2: `harness::run` skeleton (immediate-deadline = no ticks)

**File**: `harness/src/run.rs` (new); `harness/src/lib.rs`.

Failing test:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use rtf_sim::{recorder::NullRecorder, runnable_world::RunnableWorld, primitive::SceneSnapshot};
    use rtf_core::{time::{Time, Duration}, world_view::WorldView, score::Score, controller::*, goal::Goal};

    struct W { t: Time }
    impl WorldView for W {}
    impl RunnableWorld for W {
        fn publish_sensors(&mut self) {}
        fn consume_actuators_and_integrate(&mut self, dt: Duration) { self.t = self.t + dt; }
        fn snapshot(&self) -> SceneSnapshot { SceneSnapshot { t: self.t, items: vec![] } }
        fn time(&self) -> Time { self.t }
    }
    struct Noop;
    impl Controller for Noop { fn step(&mut self, _: Time) -> Result<(), ControlError> { Ok(()) } }
    struct AlwaysHalf;
    impl Goal<W> for AlwaysHalf { fn evaluate(&self, _: &W) -> Score { Score::new(0.5) } }

    #[test]
    fn zero_deadline_terminates_immediately_with_evaluated_score() {
        let cfg = RunConfig::default().with_deadline(Duration::from_nanos(0));
        let res = run(W { t: Time::from_nanos(0) }, Noop, AlwaysHalf, cfg);
        assert!(matches!(res.terminated_by, Termination::Deadline));
        assert_eq!(res.score.value, 0.5);
    }
}
```

Implement: `pub fn run<W, C, G, R>(...) -> RunResult` that returns immediately with `Deadline` termination + `goal.evaluate(&world)` score. Loop body comes in 4.3.

Commit `[harness] Step 4.2: harness::run skeleton (deadline-only)`.

### Step 4.3: Tick loop with controller `step` between publish and consume

**File**: `harness/src/run.rs` (extend). The `RunnableWorld` trait already has `publish_sensors` and `consume_actuators_and_integrate` from Step 2.10 — no trait changes needed.

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

Implement:

```rust
pub fn run<W, C, G, R>(mut world: W, mut controller: C, mut goal: G, cfg: RunConfig<R>) -> RunResult
where W: rtf_sim::runnable_world::RunnableWorld, C: Controller, G: Goal<W>, R: rtf_sim::recorder::Recorder
{
    let dt_ns = 1_000_000_000_i64 / cfg.tick_rate_hz as i64;
    let dt = Duration::from_nanos(dt_ns);
    let mut cfg = cfg; // owned; we may mutate the recorder
    let deadline_time = Time::from_nanos(cfg.deadline.as_nanos());

    let terminated_by;
    loop {
        if world.time() >= deadline_time { terminated_by = Termination::Deadline; break; }
        if goal.is_complete(&world) { terminated_by = Termination::GoalComplete; break; }

        world.publish_sensors();
        cfg.recorder.record(&world.snapshot());
        goal.tick(world.time(), &world);

        match controller.step(world.time()) {
            Ok(()) => {}
            Err(e) if e.kind == ControlErrorKind::Recoverable => { /* logged via record_event when feature on */ }
            Err(e) => { terminated_by = Termination::ControllerError(e); break; }
        }

        world.consume_actuators_and_integrate(dt);
    }

    RunResult {
        score: goal.evaluate(&world),
        final_time: world.time(),
        terminated_by,
    }
}
```

Commit `[harness] Step 4.3: Tick loop with controller step between publish/consume`.

### Step 4.4: Goal completion short-circuits the loop

**File**: `harness/src/run.rs` (verification).

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

The loop already short-circuits per 4.3; this test only verifies. Commit `[harness] Step 4.4: Verify goal-complete short-circuit`.

### Step 4.5: Recorder is invoked once per tick (using `Rc<Cell<u32>>` pattern)

**File**: `harness/src/run.rs` (test only).

```rust
#[test]
fn recorder_is_called_once_per_tick() {
    use std::rc::Rc;
    use std::cell::Cell;
    use rtf_sim::recorder::Recorder;
    use rtf_sim::primitive::SceneSnapshot;

    struct Counter(Rc<Cell<u32>>);
    impl Recorder for Counter {
        fn record(&mut self, _: &SceneSnapshot) { self.0.set(self.0.get() + 1); }
    }

    let counter = Rc::new(Cell::new(0u32));
    let cfg = RunConfig::default()
        .with_deadline(Duration::from_millis(5))
        .with_tick_rate(1000)
        .with_recorder(Counter(Rc::clone(&counter)));
    let _ = run(W { t: Time::from_nanos(0) }, Noop, AlwaysHalf, cfg);
    // 5 ticks at 1 kHz with 5 ms deadline → 5 record calls (loop body runs while t < deadline).
    assert_eq!(counter.get(), 5);
}
```

The implementation already calls `cfg.recorder.record(&world.snapshot())` in the per-tick loop (Step 4.3 §). This test merely verifies. The `Rc<Cell<u32>>` pattern lets the test inspect the counter after the run consumes the recorder.

Commit `[harness] Step 4.5: Verify per-tick recorder invocation (Rc<Cell> counter)`.

End of Phase 4 verification: `cargo test --workspace`.

---

## Phase 5: First end-to-end test (no gravity)

This phase wires Phases 1–4 together and proves the pipeline end-to-end. The goal is `ReachPose` (no grasping, no falling — pure servoing). Gravity is disabled by passing `false` to `ArmWorld::new`.

### Step 5.1: `ReachPose` goal in `rtf_arm::goals`

**File**: `arm/src/goals/mod.rs` (new); `arm/src/goals/reach_pose.rs` (new); `arm/src/lib.rs`.

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::ArmWorld;
    use crate::spec::*;
    use rtf_sim::scene::Scene;

    #[test]
    fn score_is_one_when_at_target_within_tolerance() {
        let world = ArmWorld::new(Scene::new(0), simple_spec(), /* gravity */ false);
        let target = world.ee_pose();
        let goal = ReachPose::new(target, /* tolerance */ 0.01);
        assert!(goal.is_complete(&world));
        assert!(goal.evaluate(&world).value > 0.99);
    }
    fn simple_spec() -> ArmSpec { /* same helper as Step 3.7 */ }
}
```

Implement: `pub struct ReachPose { target: Isometry3<f32>, tolerance: f32 }`. `evaluate`: `dist = (world.ee_pose().translation.vector - self.target.translation.vector).norm()`; `Score::new((1.0 - dist as f64 / self.tolerance as f64).clamp(0.0, 1.0))`. `is_complete`: `dist <= self.tolerance`.

Commit `[arm] Step 5.1: ReachPose goal`.

### Step 5.2: Hand-rolled PD controller — in `arm/src/examples_.rs` from the start

**File**: `arm/src/examples_.rs` (new); `arm/src/lib.rs`; `arm/Cargo.toml` (add `examples` feature).

In `arm/Cargo.toml`:

```toml
[features]
default = []
examples = []
```

Then `arm/src/lib.rs`:

```rust
#[cfg(any(test, feature = "examples"))]
pub mod examples_;
```

The module is gated so that bare `cargo build -p rtf_arm` does not compile `examples_` into a release artifact.

`arm/src/examples_.rs`:

```rust
//! Example controllers, gated behind `examples` feature (or test).
//! Generic over `R: PortReader<JointEncoderReading>` so they accept either raw `PortRx<_>`
//! (Phase 5 happy path) or fault-wrapped readers (Phase 9.6).

use rtf_core::{controller::*, port::*, time::Time};
use crate::ports::*;

pub struct PdJointController<R: PortReader<JointEncoderReading>> {
    target_q: Vec<f32>,
    kp: f32, kd: f32,
    encoder_rxs: Vec<R>,
    velocity_txs: Vec<PortTx<JointVelocityCommand>>,
}

impl<R: PortReader<JointEncoderReading>> PdJointController<R> {
    pub fn new(target_q: Vec<f32>, encoder_rxs: Vec<R>, velocity_txs: Vec<PortTx<JointVelocityCommand>>) -> Self {
        Self { target_q, kp: 4.0, kd: 1.5, encoder_rxs, velocity_txs }
    }
}

impl<R: PortReader<JointEncoderReading>> Controller for PdJointController<R> {
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
```

(No `use std::collections::HashMap;` — joints are indexed by `i: usize` into `Vec`s; design §10.2 forbids `HashMap` in tick path.)

Failing test (in the same file):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use rtf_core::port::port;

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
        assert!(cmd.q_dot_target > 0.0);
    }
}
```

Commit `[arm] Step 5.2: PdJointController in examples_ (generic over PortReader)`.

### Step 5.3: `build_simple_arm_world` + `attach_standard_arm_ports` helpers

**File**: `arm/src/test_helpers.rs` (new); `arm/src/lib.rs`.

```rust
#[cfg(any(test, feature = "examples"))]
pub mod test_helpers;
```

```rust
//! Test helpers — gated like examples_.

use crate::{spec::*, world::ArmWorld, ports::*};
use rtf_sim::scene::Scene;
use rtf_core::port::PortRx;
use nalgebra::{Vector3, Isometry3};

pub fn build_simple_arm_world(n_joints: usize) -> ArmWorld {
    let spec = ArmSpec {
        joints: vec![JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-3.14, 3.14) }; n_joints],
        link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.2); n_joints],
        gripper: GripperSpec { proximity_threshold: 0.02, max_grasp_size: 0.05 },
    };
    ArmWorld::new(Scene::new(0), spec, /* gravity */ false)   // Phase 5 doesn't use gravity
}

pub struct StandardArmPorts<R = PortRx<JointEncoderReading>>
where R: rtf_core::port::PortReader<JointEncoderReading> {
    pub encoder_rxs:  Vec<R>,
    pub velocity_txs: Vec<rtf_core::port::PortTx<JointVelocityCommand>>,
    pub gripper_tx:   rtf_core::port::PortTx<GripperCommand>,
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

Commit `[arm] Step 5.3: build_simple_arm_world + attach_standard_arm_ports helpers`.

### Step 5.4: End-to-end `#[test]` — PD reaches target

**File**: `arm/tests/reach_pose_e2e.rs` (new — integration test). Requires the `examples` feature on `rtf_arm`'s `[dev-dependencies]` self-import (or just add `[features] e2e = ["examples"]` and use `--features e2e`).

```rust
use rtf_arm::{world::*, goals::reach_pose::ReachPose, examples_::PdJointController, test_helpers::*};
use rtf_core::time::{Duration, Time};
use rtf_harness::{run, types::{RunConfig, Termination}};

#[test]
fn pd_reaches_target_pose_within_2_seconds() {
    let mut world = build_simple_arm_world(3);
    let ports = world.attach_standard_arm_ports();

    let target_q = vec![0.5_f32; 3];
    let controller = PdJointController::new(target_q.clone(), ports.encoder_rxs, ports.velocity_txs);
    let target_ee = rtf_arm::fk::forward_kinematics(&world.arm.spec, &target_q);
    let goal = ReachPose::new(target_ee, 0.01);

    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(2))
        .with_tick_rate(1000)
        .with_seed(42);
    // Note: gravity is OFF — set in build_simple_arm_world (Phase 5 only).

    let res = run(world, controller, goal, cfg);
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "did not converge in time; final score = {}", res.score.value
    );
}
```

Run: `cargo test -p rtf_arm --features examples --test reach_pose_e2e -- --nocapture`. If it fails (controller doesn't converge), tune `kp`/`kd` or the deadline — but don't loosen tolerance below 0.01 m.

Commit `[arm] Step 5.4: End-to-end ReachPose test (no gravity)`.

End of Phase 5 verification: a single `#[test]` proves the entire pipeline works.

---

## Phase 6: Gravity-fall in `rtf_sim`

Per design §5.5. Lives in a new `rtf_sim::gravity` module. `ArmWorld::consume_actuators_and_integrate` calls into it before integrating arm joint state, **only when** `self.gravity_enabled == true`.

### Step 6.1: Gravity constants + `Object::lin_vel` access

**File**: `sim/src/gravity.rs` (new); `sim/src/lib.rs`. Confirm `Object` already has `lin_vel: Vector3<f32>` field from Step 2.2.

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

Commit `[sim] Step 6.1: Gravity constants`.

### Step 6.2: `find_support_beneath` helper

**File**: `sim/src/gravity.rs` (extend).

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

Add `Scene::with_ground()` constructor that pre-inserts an "infinite" ground-plane fixture (e.g., huge AABB at z=0). Add `Scene::add_fixture`, `Scene::object_mut`, `Scene::object` accessors as needed.

Commit `[sim] Step 6.2: find_support_beneath helper + Scene::with_ground`.

### Step 6.3a: `gravity_step` — integration only (no settling, single object)

**File**: `sim/src/gravity.rs` (extend).

```rust
#[test]
fn free_object_falls_under_gravity_with_no_supports() {
    use crate::{scene::Scene, object::*, shape::Shape};
    use nalgebra::Isometry3;
    let mut scene = Scene::new(0); // no with_ground; nothing to land on
    let oid = scene.add_object(Object::new(
        ObjectId(1),
        Isometry3::translation(0.0, 0.0, 1.0),
        Shape::Sphere { radius: 0.05 },
        0.1, true,
    ));
    let z_before = scene.object(oid).unwrap().pose.translation.z;
    super::gravity_step(&mut scene, 1_000_000); // 1ms
    let z_after = scene.object(oid).unwrap().pose.translation.z;
    assert!(z_after < z_before, "object should have fallen; z_before={}, z_after={}", z_before, z_after);
}
```

Implement first version — apply gravity to all `Free` objects, no support check, no settle:

```rust
pub fn gravity_step(scene: &mut Scene, dt_ns: i64) {
    let dt_s = dt_ns as f32 / 1e9;
    for (_id, obj) in scene.objects_mut() {
        if !matches!(obj.state, ObjectState::Free) { continue; }
        obj.lin_vel.z -= GRAVITY_M_PER_S2 * dt_s;
        obj.pose.translation.z += obj.lin_vel.z * dt_s;
    }
}
```

Commit `[sim] Step 6.3a: gravity_step (integration only)`.

### Step 6.3b: `gravity_step` — snap to support

**File**: `sim/src/gravity.rs` (extend the same function).

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
    for _ in 0..2000 { super::gravity_step(&mut scene, 1_000_000); }
    let o = scene.object(oid).unwrap();
    assert!(matches!(o.state, ObjectState::Settled { .. }));
    let table_top_z = 0.5 + 0.01;
    assert!((o.pose.translation.z - (table_top_z + 0.05)).abs() < 1e-3);
}
```

Implement: extend `gravity_step` so that after integrating each `Free` object's z, it calls `find_support_beneath` and, if `bottom_z <= top_z + SETTLE_EPSILON_M`, snaps z and sets `state = Settled { on: support_id }`, zeroes `lin_vel.z`.

Commit `[sim] Step 6.3b: gravity_step snaps to support`.

### Step 6.3c: deterministic iteration order — sort by `(q.z, id)`

**File**: `sim/src/gravity.rs` (extend).

```rust
#[test]
fn stack_on_stack_settles_correctly() {
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

Implement: in `gravity_step`, before integrating, build a sorted vec of (ObjectId, current z) for all `Free` objects, sorted by `(z.partial_cmp(...), id)` ascending. Process in that order so that lower objects settle before upper ones consider them as supports. (BTreeMap iteration is already by id; the explicit sort by `(z, id)` adds the z-primary ordering required by design §5.5.)

Note: deterministic equality of `f32` is acceptable for the sort key here because the same scenario + seed produces the same `f32` values. Use `f32::total_cmp` (Rust 1.62+) to handle NaN deterministically.

Commit `[sim] Step 6.3c: gravity_step deterministic order (z, id)`.

### Step 6.4: Re-evaluate displaced settled objects

**File**: `sim/src/gravity.rs` (extend).

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
    scene.remove_fixture(table);
    super::reevaluate_settled(&mut scene);
    assert!(matches!(scene.object(oid).unwrap().state, ObjectState::Free));
}
```

Implement: `pub fn reevaluate_settled(scene: &mut Scene)` walks `Settled { on: X }` objects; if support `X` no longer exists or no longer occupies the object's xy, transition to `Free`.

Add `Scene::remove_fixture` and `Scene::has_support` accessors.

Commit `[sim] Step 6.4: reevaluate_settled`.

### Step 6.5: Hook gravity into `ArmWorld::consume_actuators_and_integrate`

**File**: `arm/src/world.rs` (extend).

```rust
#[test]
fn arm_world_with_gravity_lets_object_fall_onto_table() {
    use rtf_core::time::Duration;
    use rtf_sim::{scene::Scene, fixture::*, object::*, shape::Shape};
    use nalgebra::{Isometry3, Vector3};
    let mut scene = Scene::with_ground();
    scene.add_fixture(Fixture {
        id: 0, pose: Isometry3::translation(0.0, 0.0, 0.5),
        shape: Shape::Aabb { half_extents: Vector3::new(0.5, 0.5, 0.01) },
        is_support: true,
    });
    let block = scene.add_object(Object::new(
        ObjectId(1),
        Isometry3::translation(0.0, 0.0, 1.0),
        Shape::Sphere { radius: 0.05 },
        0.1, true,
    ));
    let mut world = ArmWorld::new(scene, simple_spec(), /* gravity */ true);
    for _ in 0..2000 { world.consume_actuators_and_integrate(Duration::from_millis(1)); }
    assert!(matches!(world.scene.object(block).unwrap().state, ObjectState::Settled { .. }));
}
```

Implement: at the **top** of `consume_actuators_and_integrate(dt)`, if `self.gravity_enabled`, call:

```rust
rtf_sim::gravity::reevaluate_settled(&mut self.scene);
rtf_sim::gravity::gravity_step(&mut self.scene, dt.as_nanos());
```

Commit `[sim,arm] Step 6.5: Wire gravity into ArmWorld`.

### Step 6.6: Released-object integration test (full pipeline)

**File**: `arm/tests/release_falls.rs` (new).

```rust
#[test]
fn object_grasped_then_released_falls_to_table() {
    // Build world (gravity on), grasp an object, move EE somewhere over the table,
    // open gripper, run for a bit. Assert: object lands on the table, settled.
}
```

Concrete steps in the test: construct world (gravity on) with table at z=0.5 + a graspable object held by the arm at z=1.0; **scripted controller** (in the test file, since it's specific) — `Tick<5: gripper close; 5≤Tick<200: send velocity command toward target xy via simple per-joint velocity; 200≤Tick<210: gripper open; rest: noop`; goal is `AlwaysScore(1.0)` (run-to-deadline); assert at the end `world.object(BLOCK).unwrap().state` is `Settled` and z ≈ table_top + half_height.

Commit `[arm] Step 6.6: Released-object falls to table integration test`.

### Step 6.7: Sanity — Phase 5's ReachPose test still passes with gravity disabled

**File**: (no new file — re-run existing test).

```bash
cargo test -p rtf_arm --features examples --test reach_pose_e2e
```

Confirms no regressions from Phase 6 changes. Commit `[arm] Step 6.7: Verify Phase 5 regression-free` (no code change; commit message documents the verification).

End of Phase 6 verification: full suite + clippy.

---

## Phase 7: Goals — `PickObject`, `PlaceInBin`, `Stack`

`Step 7.0` adds the helpers that 7.1–7.4 depend on. `7.1`, `7.2`, `7.3` are independent of each other (separate goal files). `7.4` depends on `7.0`, `7.1`, `7.2`, and the example PickPlace controller.

### Step 7.0: `build_pick_and_place_world` + `block_id` / `bin_id` helpers

**File**: `arm/src/test_helpers.rs` (extend the file from Step 5.3).

```rust
use rtf_sim::{scene::Scene, fixture::*, object::*, shape::Shape};
use nalgebra::{Isometry3, Vector3};
use crate::ports::*;

/// Conventional ids for the pick-and-place scenario.
pub const BLOCK_OBJECT_ID: ObjectId = ObjectId(1);
pub const BIN_FIXTURE_ID:  u32      = 2;
// (id 0 reserved for the table; ground plane uses u32::MAX.)

pub fn build_pick_and_place_world() -> ArmWorld {
    let mut scene = Scene::with_ground();

    // Table at z = 0.5
    scene.add_fixture(Fixture {
        id: 0,
        pose: Isometry3::translation(0.5, 0.0, 0.475),  // top at z = 0.5
        shape: Shape::Aabb { half_extents: Vector3::new(0.4, 0.4, 0.025) },
        is_support: true,
    });

    // Bin at z = 0.5..0.6, footprint 0.2 x 0.2 m
    scene.add_fixture(Fixture {
        id: BIN_FIXTURE_ID,
        pose: Isometry3::translation(0.0, 0.5, 0.55),
        shape: Shape::Aabb { half_extents: Vector3::new(0.1, 0.1, 0.05) },
        is_support: true,
    });

    // Block on the table, graspable
    scene.add_object(Object {
        id: BLOCK_OBJECT_ID,
        pose: Isometry3::translation(0.5, 0.0, 0.55),
        shape: Shape::Aabb { half_extents: Vector3::new(0.025, 0.025, 0.025) },
        mass: 0.1,
        graspable: true,
        state: ObjectState::Settled { on: SupportId::Fixture(0) },
        lin_vel: Vector3::zeros(),
    });

    let spec = crate::spec::ArmSpec { /* a 6-DOF arm reaching from origin to ~1m */ };
    ArmWorld::new(scene, spec, /* gravity */ true)
}

pub fn block_id(_world: &ArmWorld) -> ObjectId { BLOCK_OBJECT_ID }
pub fn bin_id(_world: &ArmWorld) -> u32 { BIN_FIXTURE_ID }
```

(The arm spec details are flexible; pick reach / link lengths so the EE can reach both the table block xy and the bin xy.)

Commit `[arm] Step 7.0: build_pick_and_place_world + block/bin id helpers`.

### Step 7.1: `PickObject` goal

**File**: `arm/src/goals/pick_object.rs` (new); `arm/src/goals/mod.rs`.

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;
    #[test]
    fn complete_when_target_object_is_grasped() {
        let mut world = build_pick_and_place_world();
        let block = block_id(&world);
        world.arm.state.grasped = Some(block);
        let goal = PickObject::new(block);
        assert!(goal.is_complete(&world));
        assert!(goal.evaluate(&world).value > 0.99);
    }
}
```

Implement: `PickObject { target: ObjectId }`. `is_complete`: `world.arm.state.grasped == Some(target)`. `evaluate`: 1.0 if grasped, else shaped on EE-to-object distance.

Commit `[arm] Step 7.1: PickObject goal`.

### Step 7.2: `PlaceInBin` goal

**File**: `arm/src/goals/place_in_bin.rs` (new); `arm/src/goals/mod.rs`.

```rust
#[test]
fn complete_when_object_is_settled_inside_bin() {
    use crate::test_helpers::*;
    let mut world = build_pick_and_place_world();
    let block = block_id(&world);
    let bin = bin_id(&world);
    // Place block above bin and let gravity settle it (or set state directly):
    world.scene.object_mut(block).unwrap().state =
        rtf_sim::object::ObjectState::Settled { on: rtf_sim::object::SupportId::Fixture(bin) };
    world.scene.object_mut(block).unwrap().pose.translation = nalgebra::Translation3::new(0.0, 0.5, 0.55 + 0.025);
    let goal = PlaceInBin::new(block, bin);
    assert!(goal.is_complete(&world));
    assert!(goal.evaluate(&world).value > 0.99);
}
```

Implement: `PlaceInBin { target: ObjectId, bin: u32 }`. `is_complete`: object is `Settled { on: SupportId::Fixture(bin) }` AND its xy lies within the bin's xy footprint. `evaluate`: 1.0 if complete, else shaped on horizontal distance from bin center.

Commit `[arm] Step 7.2: PlaceInBin goal`.

### Step 7.3: `Stack` goal

**File**: `arm/src/goals/stack.rs` (new); `arm/src/goals/mod.rs`.

```rust
#[test]
fn complete_when_a_settled_on_b_for_at_least_100ms() {
    use crate::test_helpers::*;
    let mut world = build_pick_and_place_world();
    let a = world.scene.add_object_default();
    let b = world.scene.add_object_default();
    world.scene.object_mut(a).unwrap().state =
        rtf_sim::object::ObjectState::Settled { on: rtf_sim::object::SupportId::Object(b.0) };
    let mut goal = Stack::new(a, b);
    for ms in 0..100 {
        goal.tick(rtf_core::time::Time::from_millis(ms), &world);
        assert!(!goal.is_complete(&world), "should not be complete yet at {} ms", ms);
    }
    goal.tick(rtf_core::time::Time::from_millis(100), &world);
    assert!(goal.is_complete(&world));
    assert!(goal.evaluate(&world).value > 0.99);
}
```

Implement: `Stack { a: ObjectId, b: ObjectId, settled_since: Option<Time> }`. `tick` updates `settled_since`. `is_complete`: `(now - settled_since) >= 100ms`.

Commit `[arm] Step 7.3: Stack goal`.

### Step 7.4a: `PickPlace` state-machine controller in `examples_`

**File**: `arm/src/examples_.rs` (extend).

State machine: `Approach → CloseGripper → MoveToBin → OpenGripper → Done`. Transitions on geometric conditions (EE near object xy/z, object grasped, EE near bin xy/z).

```rust
pub struct PickPlace<R: PortReader<JointEncoderReading>, P: PortReader<EePoseReading>> {
    state: PickPlaceState,
    target_block_xy: (f32, f32),
    target_bin_xy:   (f32, f32),
    encoder_rxs: Vec<R>,
    ee_pose_rx: P,
    velocity_txs: Vec<PortTx<JointVelocityCommand>>,
    gripper_tx: PortTx<GripperCommand>,
}

enum PickPlaceState { Approach, CloseGripper(u32 /*ticks held*/), MoveToBin, OpenGripper(u32), Done }
```

Failing tests for transitions:

```rust
#[test]
fn approach_transitions_to_close_when_ee_within_threshold_of_block() { /* ... */ }
#[test]
fn close_gripper_holds_for_n_ticks_then_moves_to_bin() { /* ... */ }
#[test]
fn open_gripper_completes_after_n_ticks() { /* ... */ }
```

Implement the state machine. Use a simple inverse-kinematics-free approach: command joint velocities proportional to the xy-error projected onto the appropriate joint directions. (The arm spec from Step 7.0 should be chosen so this naive approach works.) For more difficult arms, the controller can be replaced — this is just an example.

Commit `[arm] Step 7.4a: PickPlace state-machine controller + transition unit tests`.

### Step 7.4b: End-to-end PickPlace `#[test]`

**File**: `arm/tests/pick_place_e2e.rs` (new).

```rust
use rtf_arm::{world::*, goals::place_in_bin::PlaceInBin, examples_::PickPlace, test_helpers::*};
use rtf_core::time::{Duration, Time};
use rtf_harness::{run, types::{RunConfig, Termination}};

#[test]
fn state_machine_picks_block_and_drops_in_bin() {
    let mut world = build_pick_and_place_world();
    let ports = world.attach_standard_arm_ports();
    let ee_pose_rx = world.attach_ee_pose_sensor(rtf_arm::ports::RateHz::new(100));
    let block = block_id(&world);
    let bin = bin_id(&world);

    let controller = PickPlace::new(
        ports.encoder_rxs, ee_pose_rx, ports.velocity_txs, ports.gripper_tx,
        /* block xy */ (0.5, 0.0),
        /* bin xy */ (0.0, 0.5),
    );
    let goal = PlaceInBin::new(block, bin);

    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(15))
        .with_seed(42);

    let res = run(world, controller, goal, cfg);
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "score={}, time={:?}", res.score.value, res.final_time
    );
    assert!(res.score.value > 0.9);
}
```

Run: `cargo test -p rtf_arm --features examples --test pick_place_e2e -- --nocapture`. **This is the headline acceptance test for v1.**

Commit `[arm] Step 7.4b: End-to-end PickPlace acceptance test`.

End of Phase 7 verification: headline E2E completes well under the 15 s deadline; `cargo test --workspace` is green.

---

## Phase 8: `rtf_viz` crate (rerun-backed visualizer)

Independent of Phases 6 & 7 — can run in parallel after Phase 5.

### Step 8.1: `rtf_viz` skeleton with `rerun` feature flag

**File**: `viz/src/lib.rs`; `viz/Cargo.toml`.

```toml
[features]
default = []
rerun = ["dep:rerun"]

[dependencies]
rtf_sim = { path = "../sim" }
rerun = { workspace = true, optional = true }
```

```rust
#[cfg(test)]
mod tests {
    #[test]
    fn viz_compiles_without_rerun_feature() { /* no-op */ }
}
```

Commit `[viz] Step 8.1: rtf_viz crate skeleton with optional rerun feature`.

### Step 8.2: `FileRecorder` (JSON-lines, no rerun dep)

**File**: `viz/src/file_recorder.rs` (new); `viz/src/lib.rs`. Add `serde = "1"` and `serde_json = "1"` deps.

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use rtf_sim::{primitive::*, recorder::Recorder};
    use rtf_core::time::Time;
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

Implement: `pub struct FileRecorder { writer: BufWriter<File> }` + `pub fn create<P: AsRef<Path>>(path: P) -> io::Result<Self>` + `impl Recorder` that serializes each snapshot via `serde_json::to_writer` + `\n`. Add `#[derive(serde::Serialize)]` to `Primitive`, `Color`, `SceneSnapshot`, `EntityId` in `rtf_sim`.

Commit `[viz] Step 8.2: FileRecorder (JSON-lines)`.

### Step 8.3a: `RerunRecorder` skeleton — stream init + `Sphere` only

**File**: `viz/src/rerun_recorder.rs` (new, `#[cfg(feature = "rerun")]`); `viz/src/lib.rs`.

```rust
#[cfg(all(test, feature = "rerun"))]
mod tests {
    use super::*;
    use rtf_sim::{primitive::*, recorder::Recorder, entity::EntityId};
    use rtf_core::time::Time;
    use nalgebra::Isometry3;
    #[test]
    fn rerun_recorder_records_a_sphere() {
        let mut rec = RerunRecorder::in_memory("test").unwrap();
        rec.record(&SceneSnapshot {
            t: Time::from_nanos(0),
            items: vec![(EntityId::Object(1), Primitive::Sphere {
                pose: Isometry3::identity(), radius: 0.05, color: Color::WHITE,
            })],
        });
    }
}
```

Implement: `pub struct RerunRecorder { stream: rerun::RecordingStream }` + `pub fn in_memory(name: &str) -> Result<Self, ...>` (uses `RecordingStreamBuilder::memory()`). `impl Recorder for RerunRecorder` — `record` sets `rec_stream.set_time_seconds("sim", snapshot.t.as_secs_f64())` then iterates items; for `Primitive::Sphere`, emits via `rerun::archetypes::Points3D::new([pose.translation.vector.into()]).with_radii([radius])` (or the closest 0.21-API equivalent).

Commit `[viz] Step 8.3a: RerunRecorder + Sphere mapping`.

### Step 8.3b: `Box` mapping

**File**: `viz/src/rerun_recorder.rs` (extend). Map `Primitive::Box` to `rerun::archetypes::Boxes3D::from_centers_and_half_sizes(...)`. Failing test variant emits a Box and constructs without panic. Commit `[viz] Step 8.3b: RerunRecorder + Box mapping`.

### Step 8.3c: `Capsule` mapping (or Mesh3D fallback)

**File**: `viz/src/rerun_recorder.rs` (extend). Use `rerun::archetypes::Capsules3D` if available in 0.21; otherwise emit a `Mesh3D` for a procedurally-generated capsule. Document the choice in a code comment. Commit `[viz] Step 8.3c: RerunRecorder + Capsule mapping`.

### Step 8.3d: `Line` mapping

**File**: `viz/src/rerun_recorder.rs` (extend). Map `Primitive::Line` to `rerun::archetypes::LineStrips3D`. Commit `[viz] Step 8.3d: RerunRecorder + Line mapping`.

### Step 8.3e: `Label` mapping (text overlay at pose)

**File**: `viz/src/rerun_recorder.rs` (extend). Map `Primitive::Label` to `rerun::archetypes::Points3D::new([pose])` with a label string (or `TextDocument` variant — pick what looks best). Commit `[viz] Step 8.3e: RerunRecorder + Label mapping`.

### Step 8.4: Smoke test — opt `RerunRecorder` into the headline E2E

**File**: `arm/tests/pick_place_e2e.rs` (extend with feature-gated variant); `arm/Cargo.toml`.

In `arm/Cargo.toml`:

```toml
[features]
default = []
examples = []
viz-rerun = ["rtf_viz/rerun"]

[dev-dependencies]
rtf_viz = { path = "../viz" }
```

Test:

```rust
#[cfg(feature = "viz-rerun")]
#[test]
fn state_machine_picks_block_and_drops_in_bin_with_rerun() {
    use rtf_viz::rerun_recorder::RerunRecorder;
    // Spawn the rerun viewer in a child process; the SDK's `serve_grpc` exposes a port the
    // viewer reads from. Alternative: write to a `.rrd` file and open it later with `rerun foo.rrd`.
    let mut rec = RerunRecorder::serve_grpc("pick_place").unwrap(); // listens on default rerun port
    let mut world = build_pick_and_place_world();
    /* ... same as 7.4b but with .with_recorder(rec) ... */
}
```

**Launching the viewer manually** (one-time install + per-run):

```bash
# Install once:
cargo install rerun-cli --version 0.21
# Then either:
#   (a) Live: in another terminal, `rerun` (opens viewer, listens on default port);
#       then run the test — frames stream in real time.
#   (b) Offline: write to file, then `rerun /tmp/pick_place.rrd`.
```

Commit `[arm,viz] Step 8.4: Optional rerun-recorded variant of E2E + viewer launch docs`.

---

## Phase 9: `rtf_sim::faults` — channel and reading combinators

Independent of Phases 6, 7, 8 — can run after Phase 2.

The `PortReader<T>` trait was introduced in Step 1.4 (back-fill done in plan v2), so all controllers from Phase 5 onward already accept any `R: PortReader<T>`. Phase 9 adds wrapper types that themselves impl `PortReader<T>`. Each is deterministic given an explicit seed.

### Step 9.1: `rtf_sim::faults` module + blanket `PortReader for Box<R>`

**Files**: `core/src/port.rs` (extend); `sim/src/faults/mod.rs` (new); `sim/src/lib.rs`.

Add a blanket impl in `core/src/port.rs` so `Box<dyn PortReader<T>>` satisfies the bound:

```rust
impl<T, R: ?Sized + PortReader<T>> PortReader<T> for Box<R> {
    fn latest(&self) -> Option<T> { (**self).latest() }
    fn take(&mut self) -> Option<T> { (**self).take() }
}
```

Failing test:

```rust
#[test]
fn boxed_dyn_port_reader_satisfies_bound() {
    use std::boxed::Box;
    let (tx, rx) = port::<i32>();
    tx.send(42);
    let mut boxed: Box<dyn PortReader<i32>> = Box::new(rx);
    fn pull<R: PortReader<i32>>(r: &mut R) -> Option<i32> { r.take() }
    assert_eq!(pull(&mut boxed), Some(42));
}
```

In `sim/src/faults/mod.rs`, just `pub mod delay;` etc. as the wrappers land.

Commit `[core,sim] Step 9.1: Box<dyn PortReader<T>> blanket impl + sim::faults module`.

### Step 9.2: `Delay` wrapper (latency, uses `Rc<dyn Clock>`)

**File**: `sim/src/faults/delay.rs` (new); `sim/src/faults/mod.rs`.

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use rtf_core::{port::*, time::{Time, Duration}, clock::Clock};
    use rtf_sim::sim_clock::SimClock;
    use std::rc::Rc;

    #[test]
    fn delay_releases_value_only_after_latency_elapses() {
        let clock: Rc<dyn Clock> = Rc::new(SimClock::new());
        let (tx, rx) = port::<i32>();
        let mut delayed = Delay::new(rx, Duration::from_millis(2), Rc::clone(&clock));
        tx.send(7);
        // Time hasn't advanced — latest should be None (value not yet released).
        assert!(delayed.latest().is_none());

        // Need to be able to advance the clock; downcast back to SimClock for test.
        // (In production code, sim/world owns the SimClock and ticks it.)
        let sim_clock_rc: Rc<SimClock> = clock.clone().downcast_rc().unwrap_or_else(|_| panic!("downcast"));
        // Easier: keep a separate Rc<SimClock> handle alongside the Rc<dyn Clock>.

        // Practically: write the test using two Rcs to the same SimClock,
        // one as Rc<SimClock> for advance(), one as Rc<dyn Clock> for the wrapper:
        let raw_clock = Rc::new(SimClock::new());
        let clock_dyn: Rc<dyn Clock> = raw_clock.clone();
        let (tx2, rx2) = port::<i32>();
        let mut delayed2 = Delay::new(rx2, Duration::from_millis(2), clock_dyn);
        tx2.send(7);
        assert!(delayed2.latest().is_none());
        raw_clock.advance(Duration::from_millis(2));
        assert_eq!(delayed2.latest(), Some(7));
    }
}
```

Implement:

```rust
use std::rc::Rc;
use std::collections::VecDeque;
use rtf_core::{port::PortReader, time::{Time, Duration}, clock::Clock};

pub struct Delay<R: PortReader<T>, T: Clone> {
    inner: R,
    buffer: VecDeque<(Time, T)>,
    latency: Duration,
    clock: Rc<dyn Clock>,
    _phantom: std::marker::PhantomData<T>,
}

impl<R: PortReader<T>, T: Clone> Delay<R, T> {
    pub fn new(inner: R, latency: Duration, clock: Rc<dyn Clock>) -> Self { /* ... */ }
}

impl<R: PortReader<T>, T: Clone> PortReader<T> for Delay<R, T> {
    fn latest(&self) -> Option<T> {
        // Drain inner into the buffer, stamped with release_time = now + latency.
        // Then return the clone of the head iff its release_time <= clock.now().
        // ... but `latest` is &self; drain via interior mutability if needed.
        unimplemented!()
    }
    fn take(&mut self) -> Option<T> { /* same logic, take from buffer */ }
}
```

Note: making `latest(&self)` work cleanly while also draining the inner port into a buffer requires interior mutability on the buffer. Wrap the buffer in `RefCell<VecDeque<...>>`. Acceptable per the single-threaded story.

Commit `[sim] Step 9.2: Delay fault wrapper (Rc<dyn Clock>)`.

### Step 9.3: `DropMessages` wrapper (lossy channel, deterministic)

**File**: `sim/src/faults/drop_messages.rs` (new).

```rust
#[test]
fn drop_messages_drops_about_the_specified_fraction() {
    use super::*;
    use rtf_core::port::*;
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

Implement: `DropMessages<R, T>` with `inner: R`, `rng: rand_pcg::Pcg64`, `drop_rate: f32`. On each `take()`, draw a uniform; if `< drop_rate`, fetch from inner and discard, return None; else pass through.

Commit `[sim] Step 9.3: DropMessages fault wrapper`.

### Step 9.4: `StaleData` wrapper (rejects readings older than max_age, uses `Rc<dyn Clock>`)

**File**: `sim/src/faults/stale_data.rs` (new). Requires `T: SensorReading`.

```rust
#[test]
fn stale_data_rejects_old_readings() {
    use super::*;
    use rtf_core::{port::*, time::{Time, Duration}, sensor_reading::SensorReading, clock::Clock};
    use rtf_sim::sim_clock::SimClock;
    use std::rc::Rc;

    #[derive(Clone)]
    struct R { sampled_at: Time }
    impl SensorReading for R { fn sampled_at(&self) -> Time { self.sampled_at } }

    let raw_clock = Rc::new(SimClock::new());
    let clock_dyn: Rc<dyn Clock> = raw_clock.clone();
    let (tx, rx) = port::<R>();
    let mut stale = StaleData::new(rx, Duration::from_millis(50), clock_dyn);

    tx.send(R { sampled_at: Time::from_millis(0) });
    raw_clock.advance(Duration::from_millis(60));
    assert!(stale.latest().is_none());     // older than 50ms
    tx.send(R { sampled_at: Time::from_millis(60) });
    assert!(stale.latest().is_some());
}
```

Implement: on `latest()`/`take()`, fetch from inner; if `clock.now() - r.sampled_at() > max_age`, return None.

Commit `[sim] Step 9.4: StaleData fault wrapper (Rc<dyn Clock>)`.

### Step 9.5: `GaussianNoise` wrapper (with seeded non-zero draw)

**File**: `sim/src/faults/gaussian_noise.rs` (new); `arm/src/ports.rs` (add `Noise` impl).

`Noise` trait in `rtf_core::sensor_reading` (next to `SensorReading`):

```rust
pub trait Noise {
    fn apply_noise(&mut self, rng: &mut rand_pcg::Pcg64, stddev: f32);
}
```

`impl Noise for JointEncoderReading` perturbs `q` and `q_dot` by `stddev * standard_normal()`. Use `rand_distr::StandardNormal`.

```rust
#[test]
fn gaussian_noise_perturbs_q_field_with_known_seed() {
    use super::*;
    use rtf_core::{port::*, time::Time};
    use rtf_arm::ports::*;

    let (tx, rx) = port::<JointEncoderReading>();
    // Seed picked to produce a non-zero standard-normal draw on the first call:
    let mut noisy = GaussianNoise::new(rx, /* stddev */ 0.01_f32, /* seed */ 7);
    tx.send(JointEncoderReading { joint: JointId(0), q: 1.0, q_dot: 0.0, sampled_at: Time::from_nanos(0) });
    let r = noisy.take().unwrap();
    // Either q or q_dot must have been perturbed (both come from the same RNG).
    assert!((r.q - 1.0).abs() > 1e-9 || r.q_dot.abs() > 1e-9,
            "expected at least one field perturbed; got q={}, q_dot={}", r.q, r.q_dot);
    // Order of magnitude check: perturbation should be within a few stddev.
    assert!((r.q - 1.0).abs() < 5.0 * 0.01, "q drift larger than 5σ");
}
```

Implement `GaussianNoise<R, T>` where `T: Clone + Noise`. Internal `rand_pcg::Pcg64` seeded explicitly. `take()` fetches from inner, calls `apply_noise(&mut rng, stddev)`, returns.

Commit `[sim,arm] Step 9.5: GaussianNoise + Noise trait + impl for JointEncoderReading`.

### Step 9.6: Robustness test — PD reaches pose under encoder delay + noise

**File**: `arm/tests/reach_pose_with_faults.rs` (new).

```rust
#[test]
fn pd_still_reaches_target_with_2ms_encoder_delay_and_noise() {
    use std::rc::Rc;
    use rtf_core::{port::*, time::Duration, clock::Clock};
    use rtf_arm::{world::*, ports::*, examples_::PdJointController, fk, goals::reach_pose::ReachPose, test_helpers::*};
    use rtf_harness::{run, types::{RunConfig, Termination}};
    use rtf_sim::faults::{Delay, GaussianNoise};

    let mut world = build_simple_arm_world(3);
    let raw_clock = world.sim_clock_handle();
    let clock_dyn: Rc<dyn Clock> = raw_clock.clone();

    let raw_rxs = (0..3).map(|i| world.attach_joint_encoder_sensor(JointId(i as u32), RateHz::new(1000))).collect::<Vec<_>>();
    let velocity_txs = (0..3).map(|i| world.attach_joint_velocity_actuator(JointId(i as u32))).collect::<Vec<_>>();

    // Wrap each encoder rx with: GaussianNoise → Delay (composed inside-out).
    let wrapped_rxs: Vec<Box<dyn PortReader<JointEncoderReading>>> = raw_rxs.into_iter().map(|rx| {
        let noisy = GaussianNoise::new(rx, 0.005_f32, 7);
        let delayed = Delay::new(noisy, Duration::from_millis(2), Rc::clone(&clock_dyn));
        Box::new(delayed) as Box<dyn PortReader<JointEncoderReading>>
    }).collect();

    let target_q = vec![0.5_f32; 3];
    let target_ee = fk::forward_kinematics(&world.arm.spec, &target_q);
    // PdJointController is generic over R: PortReader<...>; here R = Box<dyn ...>.
    let controller = PdJointController::<Box<dyn PortReader<JointEncoderReading>>>::new(
        target_q, wrapped_rxs, velocity_txs,
    );
    let goal = ReachPose::new(target_ee, 0.02);
    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(3))
        .with_seed(42);
    let res = run(world, controller, goal, cfg);
    assert!(matches!(res.terminated_by, Termination::GoalComplete),
        "did not converge under faults; score={}", res.score.value);
}
```

Note: `world` was built with `gravity = false` by `build_simple_arm_world`; this test stays with no gravity (it's the PD reach-pose scenario, not pick-place).

Commit `[arm] Step 9.6: Robustness test (encoder delay + noise)`.

End of Phase 9 verification: `cargo test --workspace` is green; this test passes.

---

## Final End-to-End Verification

Run after all 9 phases land. This is the gate for declaring v1 complete.

### V.1 Full test suite passes (deterministic)

```bash
cargo test --workspace --release
```

Run twice; assert identical pass count and identical headline test scores. (Determinism guarantee — design §10; weakened to "fixed toolchain+target".)

### V.2 Clippy clean across the workspace

```bash
cargo clippy --workspace --all-targets -- -D warnings
```

Zero warnings. The Step 0.4 `disallowed-types` / `disallowed-methods` lints catch any accidental `HashMap`/`Instant`/`thread::spawn` use.

### V.3 Format check

```bash
cargo fmt --all -- --check
```

Should exit 0. If it doesn't, run `cargo fmt --all` and commit a separate `[workspace] cargo fmt` commit before proceeding.

### V.4 Rustdoc compiles

```bash
cargo doc --workspace --no-deps
```

Confirms all `///` comments are well-formed and inter-crate doc-links resolve.

### V.5 Headline acceptance test

```bash
cargo test -p rtf_arm --features examples --test pick_place_e2e -- --nocapture
```

Expect: `state_machine_picks_block_and_drops_in_bin` passes with `score > 0.9` and `Termination::GoalComplete` within 15 s of sim time.

### V.6 Robustness test

```bash
cargo test -p rtf_arm --features examples --test reach_pose_with_faults -- --nocapture
```

PD reaches pose under 2 ms encoder delay + 0.005 rad noise.

### V.7 Visualizer round-trip (manual, opt-in)

Install rerun-cli once:

```bash
cargo install rerun-cli --version 0.21
```

Then in one terminal:

```bash
rerun
# (opens viewer; listens on default rerun port)
```

In another terminal:

```bash
cargo test -p rtf_arm --features "examples viz-rerun" --test pick_place_e2e -- --nocapture
```

Confirm in the rerun viewer: arm capsules visible, block AABB visible, table + bin visible, timeline scrubbable, block visibly drops into bin near the end.

### V.8 Documentation snapshot

```bash
git log --oneline docs/plans/
```

Expect entries for: v1 design, design-review-1, v2 design (+ amendments), implementation plan v1, implementation-plan-review-1, implementation plan v2, plus per-step commits per the plan.

---

## Notes on parallelism within phases

- **Phase 7** (PickObject, PlaceInBin, Stack goals): the three goals are entirely independent — separate files, separate tests. Can be implemented in parallel by separate sessions/agents and merged.
- **Phase 8** (viz) and **Phase 9** (faults): both depend only on Phase 5. Can run alongside Phases 6 and 7.
- **Within other phases**: steps build on each other; execute serially within a phase.

## Where to deviate

Deviations from this plan are allowed but require, per design v2 §11 and §10:

1. The deviation must not silently expand v1 scope (re-read design §11 first; if you'd be building something §11 lists as out of scope, stop and update the design).
2. The deviation must preserve all design §10 determinism invariants.
3. The deviation must be noted in the affected step's commit message (`[scope] Step N.M: <desc>; deviates from plan: <one-line reason>`).
4. If the deviation affects more than one step (e.g., a refactor that touches Phase 4–7), update the dependency table in this plan in the same commit.

This is a personal-use project, not a team contract — but the discipline above is what made the plan executable in the first place. Skipping it tends to compound into rework.

## Out of scope for v1 (do not silently expand)

Per design §11: physics dynamics beyond vertical-fall + support snap, lateral motion of free objects, joint torques, mesh geometry, multi-robot scenes, non-arm modalities, hardware adapters, data-defined scenarios, multi-component score aggregation policy, realistic-by-default sim, composite goal types (`Sequence`/`Choice`), cross-platform bit-identical determinism. If a step in this plan is tempted to add any of these, stop and re-validate against §11 first.


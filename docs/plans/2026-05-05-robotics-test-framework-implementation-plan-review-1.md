VERDICT: NEEDS_REVISION

## Summary Assessment

The plan is well-structured and the TDD discipline + per-phase decomposition is largely sound, but several load-bearing problems will block execution: the workspace member named `core` shadows Rust's built-in stdlib `core` crate (every other crate's `use core::...` will resolve to stdlib), the `RunnableWorld` trait surface has a circular default-method problem between Step 2.10 and Step 4.3, several "failing tests" will not actually compile against the implementation the plan proposes (Step 9.2's `&dyn Clock` vs. recommended `Rc<SimClock>`, Step 4.5's recorder counter test is admittedly broken and the fix is never written), and several steps cross the 2–5 minute budget by a wide margin (Step 3.11, 6.3, 8.3, 7.4 sub-tasks).

## Critical Issues (must fix)

1. **`core` is a reserved crate name in Rust.** Step 0.1 does `cargo new --lib core` and Step 0.2 has every other crate `use core::...`. But `core` is the Rust standard library's no-std core crate and is implicitly in the extern prelude in edition 2021. Inside `sim/`, `arm/`, `harness/`, `viz/`, `use core::time::Time;` will resolve to *stdlib* `core::time::Time` (which doesn't exist as a path that way — it's `core::time::Duration`), producing baffling errors. Cargo will also emit `warning: package 'core' conflicts with a built-in crate name` and any rustdoc/clippy step gets noisy. **Fix**: rename the workspace package (e.g., `rtf-core`, `rtf_core`, `bot-core`) and adjust every `use core::` import to `use rtf_core::`. This affects Steps 0.1, 0.2, 1.1–1.10, and every later step that imports from `core`. (The plan's `core/Cargo.toml` would set `name = "rtf_core"` and the path dep stays `path = "../core"`, so the directory name can stay `core` if you want — but the package name in `Cargo.toml` cannot.) Symptom you'll hit on day 1: `error[E0432]: unresolved import 'core::time::Time'`.

2. **`RunnableWorld` has a circular default-method requirement between Step 2.10 and Step 4.3.** Step 2.10 defines the trait with `tick`, `snapshot`, `time` as required. The `EmptyWorld` test impls all three. Step 4.3 then says "add `publish_sensors` and `consume_actuators_and_integrate` to `RunnableWorld`, with `tick` as a default that calls both with no controller in between (kept for the no-op world test)." If `tick` is the default, then `publish_sensors` and `consume_actuators_and_integrate` must be required (otherwise the default body has nothing to call) — which breaks the existing `EmptyWorld` impl that only provides `tick`. If `publish_sensors`/`consume_...` are defaults (no-ops), then `ArmWorld` must override them, but `tick` cannot have a meaningful default since publish/consume might be no-ops in some impls. **Fix**: pick one of the following and write it down explicitly:
   - (a) Restructure Step 2.10 to define all three methods up front (`tick` as the only required method? or all three required?), and ship `EmptyWorld` with no-op publish/consume. Then 4.3 just changes the call protocol in `harness::run`.
   - (b) Tell 4.3 explicitly: "this step also updates the `EmptyWorld` test from Step 2.10 to impl the two new methods" and add it to that step's `git add` list.
   The plan currently asserts the cake-and-eat-it option that doesn't exist.

3. **`harness::run` cannot call `world.set_gravity(cfg.gravity)` generically.** Step 6.6 says: "The `harness::run` function reads `cfg.gravity` once before the loop and calls `world.set_gravity(cfg.gravity)`." But `harness::run<W: RunnableWorld, ...>` is generic over `W`, and `set_gravity` is an `ArmWorld`-specific inherent method. There is no `set_gravity` on `RunnableWorld`. Three real options: (i) add `fn set_gravity(&mut self, on: bool) {}` to `RunnableWorld` as a default no-op (cheap, but pollutes the trait with arm-specific concept), (ii) plumb gravity through `RunConfig` into the world *at construction time* via a builder (cleaner), or (iii) introduce a `Configurable` sub-trait and bound `harness::run` on it. **Fix**: pick one. Without it, Step 6.6 will not compile.

4. **Step 9.2's failing test uses `&clock as &dyn Clock` but the implementation note recommends `Rc<SimClock>`.** The test reads:

   ```rust
   let clock = SimClock::new();
   let delayed = Delay::new(rx, Duration::from_millis(2), &clock as &dyn Clock);
   ```

   Then the implementation note says: "Pick `Rc<SimClock>` and have `ArmWorld::sim_clock() -> Rc<SimClock>`." If `Delay` stores `Rc<dyn Clock>`, the test code above will not compile (cannot pass `&dyn Clock` where `Rc<dyn Clock>` is expected). If `Delay` stores `&'a dyn Clock`, the lifetime parameter pollutes every fault wrapper and breaks the `Vec<Box<dyn PortReader<...>>>` pattern in Step 9.6 because the box has no lifetime parameter. **Fix**: rewrite the failing test in 9.2 to use `Rc::new(SimClock::new())` consistently with the implementation. Same for Step 9.4's `StaleData` test.

5. **Step 4.5's recorder-invocation test is admittedly broken and the plan never shows the fix.** The plan's own text says: "Counter is consumed; we'd need to check externally. Easier: use `Rc<Cell<u32>>`." Then the test code shown does `let _ = res;` and asserts nothing. The plan promises a refactor in parentheses but never delivers it. As written, the test passes vacuously and does not exercise the implementation. **Fix**: replace the test body with the actual `Rc<Cell<u32>>` version, e.g.:

   ```rust
   let counter = Rc::new(Cell::new(0u32));
   struct Counter(Rc<Cell<u32>>);
   impl Recorder for Counter {
       fn record(&mut self, _: &SceneSnapshot) { self.0.set(self.0.get() + 1); }
   }
   let cfg = RunConfig::default()
       .with_deadline(Duration::from_millis(5))
       .with_tick_rate(1000)
       .with_recorder(Counter(Rc::clone(&counter)));
   run(W { t: Time::from_nanos(0) }, Noop, AlwaysHalf, cfg);
   assert_eq!(counter.get(), 5); // one record per ms over 5 ms
   ```

6. **Step 5.4 changes the controller's location after Step 5.2 already committed it elsewhere.** Step 5.2 commits `arm/examples/reach_pose_pd.rs`. Step 5.4 says "to use it from a `#[test]`, either move it to `arm/src/examples_.rs` ... Pick the cleaner path: move to `arm/src/examples_.rs` as a public module gated by a `with_examples` Cargo feature." This means Step 5.2 commits a path that Step 5.4 immediately moves. Two churn-y commits where one would do. **Fix**: put it in `arm/src/examples_.rs` from the start of Step 5.2 (gated by `#[cfg(any(test, feature = "examples"))]` to avoid bloating the public API). Then Step 5.4 just imports it.

7. **Step 1.4's failing test signature for `take()` is inconsistent with its mutability claim.** The test reads:
   ```rust
   let (tx, mut rx) = port::<i32>();
   tx.send(7);
   assert_eq!(rx.take(), Some(7));
   ```
   This requires `take(&mut self)`. But the very next test does `assert_eq!(rx.latest(), None);` after `take()` — suggesting `latest(&self)`. That's fine. However, Step 9.1's `PortReader<T>` trait sketch declares `fn take(&mut self) -> Option<T>`. To impl `PortReader for PortRx<T>`, the impl block needs `&mut self` access to internal `RefCell<Option<T>>`. With `Rc<RefCell<Option<T>>>` (as Step 1.4 specifies), `take()` could actually be `&self` (you'd `borrow_mut()` on the RefCell). So the planned signature `fn take(&mut self)` is needlessly restrictive given the impl. **Fix**: decide whether the API exposes `&self` everywhere (consistent with `Rc<RefCell<...>>` interior mutability) or `&mut self` for `take` (requires `PortRx<T>` to not be `Clone`-able — and the design says ports are owned, so this is OK either way). Either is fine, but `PortReader<T>::take(&mut self)` and `PortRx<T>::take(&mut self)` have to match.

8. **Step 7.4 references `arm::test_helpers::block_id(&world)` and `bin_id(&world)` and `build_pick_and_place_world()` that no prior step creates.** Step 5.3 introduces `build_simple_arm_world` and `attach_standard_arm_ports` but not `build_pick_and_place_world`, `block_id`, or `bin_id`. Step 6.7 mentions `build_pick_and_place_world` *inline* in a comment but never as a defined helper. **Fix**: either add an explicit Step 7.0 (or extend 6.7) that creates `arm/src/test_helpers.rs` with `build_pick_and_place_world() -> ArmWorld` returning the (table + bin + block) world plus accessors `block_id(&world) -> ObjectId` and `bin_id(&world) -> u32`, or inline them in Step 7.4 with a short scenario builder. Without it, the headline acceptance test won't compile.

9. **Several steps are well over the 2–5 minute budget.** Spot-checks:

   - **Step 3.11 (`ArmWorld::tick`)**: in one step you must (a) iterate the encoder map and publish, (b) iterate the EE-pose map and publish, (c) iterate the velocity actuator map and apply, (d) iterate the gripper actuator map and apply, (e) compute FK, (f) update grasp state (close/proximity/open transitions), (g) integrate `q += q_dot * dt`, (h) advance `sim_time`, AND (i) refactor everything into `publish_sensors` + `consume_actuators_and_integrate`. That is ~30 minutes of focused work, with at least three independent debuggable concerns (publish, consume, integrate). **Split into**: 3.11a publish_sensors (encoders only); 3.11b publish EE pose; 3.11c consume_actuators_and_integrate (velocity command); 3.11d gripper actuator + grasp transitions; 3.11e time advance + tests for each.

   - **Step 6.3 (`gravity_step`)**: implementing the per-Free-object loop with sort-by-q.z, gravity integration, support search, snap-to-top, and Settled transition is ~15 minutes minimum. **Split into**: 6.3a gravity-only integration on a single Free object (no support snap); 6.3b add support snap when bottom_z ≤ top_z + ε; 6.3c stable sort by (q.z, id) for determinism (the design §5.5 requirement that the plan's pseudocode currently glosses over).

   - **Step 8.3 (`RerunRecorder`)**: mapping five `Primitive` variants to rerun archetypes, each with archetype-specific batching/coloring quirks, is easily 20–30 minutes — plus the rerun version-skew between 0.21's Capsules3D vs the plan's "if available in your rerun version or fall back to Mesh3D" hedging. **Split into**: 8.3a stream/init + Sphere only; 8.3b Box; 8.3c Capsule (or Mesh3D fallback if Capsules3D not present); 8.3d Line; 8.3e Label/text — each verifiable independently with the in-memory recorder constructor.

   - **Step 7.4 (PickPlace state machine controller + e2e test)**: the controller is itself a non-trivial state machine (`Approach → CloseGripper → MoveToBin → OpenGripper → Done`) with geometric transition predicates, and the test wires the entire pipeline. ~30+ minutes. **Split into**: 7.4a write the state-machine controller in `examples_::PickPlace` with unit tests covering each transition; 7.4b wire the e2e test using it.

10. **Step 1.4's `port<T>` `Send + Clone + 'static` constraints from design §4 are dropped without explanation.** Design §4 says `pub fn port<T: Send + Clone + 'static>() -> (PortTx<T>, PortRx<T>);`. The plan's Step 1.4 says "single-threaded single-slot using `Rc<RefCell<Option<T>>>`" and notes "no `Send`/`Sync` cross-thread sharing — design §10 single-threaded guarantee makes this fine". This is a real deviation from design §4. The trait bound `T: Send` is gratuitously restrictive given the single-threaded story (good; drop it), but the design v2 §4 still lists it. The plan should call this out explicitly: "deviation from design §4 — `T: Clone + 'static` only, since ports are single-threaded by design §10." Otherwise the implementer will copy §4's signature and be confused why nothing works with non-Send types later.

11. **`controller_events` cargo feature is not plumbed.** Design §7 says: "Recorded only when the `controller_events` cargo feature is enabled — zero overhead otherwise." Step 2.9 defines the `ControllerEvent` enum but adds no cargo feature gate to `sim/Cargo.toml`. Step 4.5 doesn't gate the recorder on this feature either. **Fix**: in Step 2.9, add `[features] controller_events = []` to `sim/Cargo.toml` and gate the `record_event` default body / the enum itself on `#[cfg(feature = "controller_events")]`. Without this, the design's "zero overhead otherwise" guarantee is silently violated.

12. **Step 1.4's `take()` test asserts `.latest() == None` after `take()`, but the implementation note says "Rc<RefCell<Option<T>>>".** With `Rc`, two references to the same port could both call `take()` — but the API gives you one tx and one rx per `port()`. That's fine semantically, but: if `PortRx<T>` is `Clone` (as `Rc` would naturally make it), then `let mut rx2 = rx.clone(); rx.take(); rx2.latest()` could observe the cleared slot. The plan should explicitly opt out of `Clone for PortRx<T>` (single-consumer guarantee) or document that cloning is allowed and `take` is racy across clones. Minor but a real footgun for users who wrap ports in helper structs.

13. **Step 5.2 imports `std::collections::HashMap` in the example.** The line `use std::collections::HashMap;` appears but `HashMap` is never used in the shown code body. This is harmless dead code — but more importantly, design §10's determinism audit forbids `HashMap` in the tick path, and the plan's Conventions section restates this. Putting `HashMap` in a controller example, even unused, primes the implementer to reach for it later. **Fix**: drop the `use HashMap;` line; if you need a map keyed by joint, use `Vec<...>` indexed by joint index (which is what the example actually does).

## Suggestions (nice to have)

1. **Add a workspace-level `[lints]` table to enforce determinism invariants.** Currently the plan relies on `cargo clippy --workspace -- -D warnings` plus reviewer eyeballs for the §10.2 forbidden list. A `clippy::disallowed_types = [{ path = "std::collections::HashMap", reason = "use BTreeMap; HashMap iteration is nondeterministic" }, { path = "std::time::Instant", reason = "inject Clock instead" }]` and similar for `std::thread::spawn` would mechanize the audit. Add to Step 0.1 or a new Step 0.4.

2. **Step 3.4's FK test happens to pass with q=[0,0,0] regardless of multiplication order.** The test only verifies `forward_kinematics(spec, [0,0,0])` against a sum-of-translations baseline; this is true whether you compose `joint * offset * joint * offset * ...` or `offset * joint * offset * joint * ...`. Add a non-zero-q test (e.g., `q = [PI/2, 0, 0]`, expect end-effector to swing to +y) that actually distinguishes the correct composition from an incorrect one. Otherwise a subtle FK bug ships and breaks Step 5.4.

3. **`PortReader<T>` trait is a real architectural change from design v2 — call it out explicitly.** Design v2 §4 specifies `PortRx<T>` directly with concrete `latest`/`take` methods; nowhere does it mention an object-safe trait. Step 9.1 introduces `PortReader<T>` as a back-fill to Phase 1.4 and updates Phase 5/7 controllers to be generic over `R: PortReader<T>`. This is necessary for fault-wrapper transparency, but it's a meaningful surface-area addition. Either: (a) update design v2 to specify `PortReader<T>` (cleanest), or (b) add a "Deviations from design v2" section to the plan that lists this explicitly so a future maintainer doesn't think it's accidental.

4. **Add a `cargo doc --workspace --no-deps` step to the Final Verification.** Catches broken doc-link comments and ensures rustdoc still compiles. Cheap insurance.

5. **Add a `cargo fmt --check` step.** Standard hygiene.

6. **Step 2.5's "object-safe by construction" claim about `Visualizable` should be tested.** Add a one-line test: `fn _is_object_safe() { let _: &dyn Visualizable; }`. If anyone later adds a generic method or `Self: Sized`, the test breaks immediately.

7. **Step 6.7's release-test description says "block lands on the table, settled" but doesn't specify the controller.** It says "tiny scripted state machine" but leaves the implementation to the implementer. For a 2–5 minute step that's a stretch. Either provide the controller code in the plan or split: 6.7a write the scripted controller; 6.7b write the e2e test.

8. **The "Where to deviate" section is a bit too permissive.** "If the SNN integration story needs concrete validation earlier than v1, add an SNN controller step after Phase 5" gives broad license without acceptance criteria. Consider tightening to: "Deviations require updating the dependency table in this plan and noting the deviation in the affected step's commit message." Otherwise the section becomes an excuse to skip the discipline that makes the plan executable.

9. **No instruction for actually launching the rerun viewer.** Step 8.4 says "Open the rerun viewer manually" and V.5 says "Open the rerun viewer, confirm: ...". Add a one-line note: `RerunRecorder::spawn_viewer("name")` (uses `rerun::RecordingStream::serve` + `rerun viewer`) or `cargo install rerun-cli && rerun foo.rrd`. Otherwise a fresh implementer will have to figure out the rerun docs themselves at the worst possible time.

10. **No SNN integration test in v1 scope, despite design §8.2 specifying the canonical drift-free template.** This is correct per design v2's "out of scope for v1" implicit reading, but worth confirming. The design review-1 caught a real bug in §8.2's old version; the plan should at least have a smoke test that verifies the design's drift-free template compiles, even without a functional SNN. A 5-minute Step 5.5 (or Phase 9.7) "compile-test the SNN accumulator pattern from design §8.2" would protect against regression. Currently zero plan tasks exercise §8.2.

11. **Step 9.5's `GaussianNoise` test relies on RNG output.** `assert_ne!(r.q, 1.0)` could (vanishingly rarely) fail if randn returns exactly 0.0 with seed 0. More robust: assert `r.q != 1.0 || did_perturb_some_field` by also checking q_dot. Or just seed a known-non-zero draw. Pedantic but the plan elsewhere is religious about determinism.

## Verified Claims (things I confirmed are correct or well-reasoned)

1. **`Time(i64)` signed-arithmetic story is consistent throughout.** Step 1.1's failing test for `Time` subtraction explicitly checks `(t1 - t0).as_nanos() == -500`, forcing signed `Duration`. Step 6.3's `gravity_step(&mut scene, 1_000_000)` (1ms in ns) matches the `dt_ns: i64` signature. Step 6.6's `dt.as_nanos()` returns `i64` per Step 1.1's signature. The arithmetic chain compiles end-to-end.

2. **`Score { value: f64, breakdown: SmallVec<...> }` is used correctly.** Step 1.7 defines it per design §4. Step 5.1, 7.1, 7.2, 7.3 all return `Score::new(...)` from `evaluate`. Headline test in 7.4 uses `res.score.value > 0.9` — matches the struct field. ✓

3. **`Controller::step(&mut self, t: Time)` with no observation argument is honored everywhere.** Every controller in the plan (Counter test in 1.5, PdJointController in 5.2, Noop in 4.2, PickPlace in 7.4) owns its port endpoints as fields and reads them inside `step`. ✓

4. **`Goal<W: WorldView>` directly over the world type is used consistently.** No `View` indirection introduced anywhere. Plan's Step 1.8 has `Goal<W>` directly; all subsequent goals impl `Goal<ArmWorld>`. ✓ matches design v2's "Goal simplified" change.

5. **`SensorReading::sampled_at` contract is enforced where it matters.** Step 1.9 defines the trait, Step 3.5 impls it for `JointEncoderReading` and `EePoseReading`, and Step 3.5's failing test exercises `r.sampled_at()`. Step 9.4's `StaleData` test uses `Time` of the reading vs. clock-now — consistent with the contract. ✓

6. **`BTreeMap` is consistently used in the tick path (per design §10).** Step 2.3's Scene uses `BTreeMap<ObjectId, Object>`; Step 3.7's ArmWorld uses `BTreeMap<PortId, _>` for all four port registries; Step 3.11 specifies "iterate BTreeMaps in PortId order". ✓ no `HashMap` appears in any tick-path code shown.

7. **`RunConfig<R: Recorder = NullRecorder>` generic-over-Recorder is correct.** Step 4.1 spells it out per design §6; default is `NullRecorder`. The harness function in 4.2/4.3 takes `cfg: RunConfig<R>` — monomorphizes per design's "monomorphize, no boxing" requirement. ✓

8. **The dependency arrows in Step 0.2 match design §3 exactly.** `core` no internal deps; `sim → core`; `arm → core, sim`; `harness → core, sim`; `viz → sim` (NEVER → arm). Plan honors the HAL boundary. ✓

9. **The implementation order respects the design's "Suggested Implementation Order" (§12) almost exactly.** Phases 1–5 mirror design steps 1–5; Phase 6 mirrors design step 6 (gravity); Phase 7 mirrors step 7 (goals); Phase 8 mirrors step 8 (viz); Phase 9 mirrors step 9 (faults). The design said 1–5 are the minimum coherent slice without gravity, 6 makes Stack honest, 8 and 9 are independent — the plan's parallelism table reflects all three. ✓

10. **The `RateScheduler` design and Step 2.7 test are sound.** The "long-run rate matches requested" test (`333 Hz over 1000 ticks → 333±1 fires`) actually exercises the phase-accumulator semantics from design §5.6, including the documented ±1-tick jitter. The "divisible rate fires exactly on period" test catches the simpler case. ✓

11. **The TDD cycle for Phase 1 steps 1.1–1.9 is genuinely test-first.** Each step writes a failing test that targets a specific type/method, runs to confirm compile failure, then implements to pass. The failing tests would actually fail for the right reason (missing type) before implementation. ✓

12. **Determinism guards in the loop body of Step 4.3 are correctly ordered.** Per-tick: publish_sensors → goal.tick → controller.step → consume_actuators → integrate → time advance. This matches design §5.7 step ordering. ✓

13. **The headline acceptance test in 7.4 maps to the design's promise** (state-machine controller + PlaceInBin goal + 15s deadline + score > 0.9). ✓ aligns with design §6's end-to-end example.

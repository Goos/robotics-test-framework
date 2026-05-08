# Sweep-Based Controllers — Scan-Pose + Grasp-Pose Redesign

Date: 2026-05-08
Status: Draft (post-Rapier v1.x follow-up; addresses the 11 ignored sweep-based seed tests from Rapier Phase 3.5 scope-cut)

## Context

Rapier Phase 3 (commits `33ff732..9168c42`) replaced the kinematic-weld grasp with joint-attached grasp + slip-impulse detection, and added a wrist joint (J3) so the EE can orient straight down for grasping. After that landing, three sweep-based example controllers regressed:

- `find_grasp_place_seed_42` and `_seed_1337`: 1 of 3 seeds passed
- `continuous_spawn_seed_*`: all 3 seeds + 1 debug-overlay variant failed
- `find_by_touch_seed_*`: all 3 seeds + 1 debug-overlay variant failed

Root cause (per Step 3.5 commit body): under wrist-down grasp-ready geometry, the fingers extend ~8 cm below the EE. Sweeping at any z-altitude that keeps the pressure / torque sensor (3 cm radius) within reach of the block top causes the fingers to bulldoze the block before the localization algorithms can converge. peak_xy lands several cm off the actual block; descent misses; grasp never forms.

Phase 3 scope-cut these tests via `#[ignore]` markers with explanatory comments. This doc addresses the redesign that restores them.

## Approach: separate scan-pose from grasp-pose

Each affected controller adopts the same pattern: a distinct **scan pose** for sweeping (wrist level, fingers extend horizontally forward — no vertical interference with blocks) and a distinct **grasp pose** for descending (wrist down — fingers can envelop blocks).

New states inserted between the existing `Sweeping` and `DescendToContact`:

```rust
enum NewState {
    Sweeping,              // EXISTING — uses scan pose (wrist level, sweep_z=0.62)
    LiftToScanAltitude,    // NEW (transition entry) — joints from initial to scan pose
    RotateWristForGrasp,   // NEW (post-detection) — drives J3 from 0 to π/2 over ~500ms while EE stays at scan_z
    DescendToContact,      // EXISTING — wrist down, descends to block
    // ... rest unchanged
}
```

Logic:
- **Constructor**: pre-compute IK targets for the scan pose (wrist level, sweep altitude) AND grasp pose (wrist down). Same `ik_3r` helper, different `target_pitch`.
- **Initial state**: `LiftToScanAltitude` (or replace it directly into the constructor's start state if the initial pose is already roughly scan-friendly).
- **Sweeping**: drive joints toward each scan-pose waypoint at sweep_z. Pressure/torque sensor detection triggers transition to `RotateWristForGrasp`.
- **RotateWristForGrasp**: J0/J1/J2 unchanged (EE stays at peak_xy + sweep_z); J3 drives from 0 to π/2. Transition to `DescendToContact` when J3 within tolerance.
- **DescendToContact** onward: unchanged.

The 3R IK already handles the geometry — same `ik_3r(radial, z, target_pitch, l1, l2, l3)` signature, just called with `target_pitch = 0.0` for scan poses and `π/2` for grasp poses.

## Per-controller specifics

### Step 4.1 — `find_grasp_place`

- Sensor: pressure (EE-mounted, eps=0.03 from Phase 1.x).
- Search region: bounded Aabb on the table top (existing).
- Single object per run.
- **Commit shape**: 1–2 commits (state machine + tuning if needed).
- **Acceptance**: `find_grasp_place_seed_1`, `_seed_42`, `_seed_1337` all reach `Termination::GoalComplete + score > 0.9` within 30s deadline. Un-ignore the 2 currently-ignored seed tests + 1 debug-overlay variant.

### Step 4.2 — `continuous_spawn`

- Sensor: pressure (same as find_grasp_place).
- Multiple objects spawned over time (3 boxes, 15s interval).
- Controller loops the SearchAndPlace state machine; needs the scan-pose pattern applied to each iteration.
- **Commit shape**: 1–2 commits, builds on Step 4.1's pattern.
- **Acceptance**: 3 seeds + debug-overlay variant all pass within 90s deadline.
- **Acceptance partially met**: 3 of 4 seeds pass (42, 1337, 42_with_debug_overlay); seed 1 remains ignored due to a spawn-during-sweep timing collision — see commit body for detail.

### Step 4.3 — `find_by_touch`

- Sensor: joint torque (not pressure) — torque doesn't suffer the sensor-radius problem, but the existing controller still tries to localize via peak xy at sweep altitude, which suffers the same finger-block interference.
- **Different consideration**: under scan-pose (wrist level, fingers forward), arm-vs-block contact during sweep is unavoidable — that's the *whole point* of touch-based search. The torque sensor needs to fire on intentional contact rather than be designed around. The redesign for this controller is more about timing (slow sweep, immediate stop on torque spike, avoid bulldozing past the block).
- Possible refinement: ScanPose with fingers OPEN (separation 0.04) so they straddle the block on contact rather than push it.
- **Commit shape**: 1–2 commits.
- **Acceptance**: 3 seeds + debug-overlay variant all pass within 30s deadline.
- **Acceptance NOT met**: scan-pose pattern alone can't resolve touch-vs-displacement tension. Re-ignored with detailed TODO. See commit body.

## Common implementation notes

- **Scan altitude**: try sweep_z = 0.62 m initially (1 cm above block top + finger half-extent + clearance under wrist-level fingers). Tune per controller.
- **Pressure eps**: keep at 0.03 unless tuning forces otherwise.
- **Wrist rotation duration**: ~500ms at 0.5 rad/s (matches close_hold_ticks pattern).
- **IK pre-computation**: each controller pre-computes 2 IK targets per waypoint (scan pose + grasp pose). Sweep waypoints already exist; just add a parallel `grasp_waypoints` list with same xy + grasp_z + target_pitch=π/2.
- **Determinism**: scan-pose changes don't affect determinism. Same seeds produce same outcomes.
- **No framework changes**: this is purely controller-side work. ArmWorld, PhysicsWorld, ports — all unchanged.

## Process

- Steps 4.1 → 4.2 → 4.3 in order; each unblocks one controller.
- TDD per step. Inline self-review.
- Each commit must keep `cargo test --release --workspace --all-targets --features viz-rerun` green (currently green with ignored tests; should stay green AND have fewer ignored tests).
- Use the speed-up tips from Phase 3 (release mode, single-seed during tuning, single-test-at-a-time during iteration).
- After Step 4.3 lands, brief V-gate sweep (V.1 once, V.2/V.3/V.4 single-shot — full V.1 twice not needed since this is purely controller-level changes).

## Out of scope

- Adapting the redesign to other arm configurations (single-arm only).
- New scenarios beyond the three existing controllers.
- Sensor model changes (pressure stays as-is; torque stays as-is).
- Plan-level adjustments to grasp_robustness or pick_place (these don't have sweep phases and are unaffected).

//! Step 7.4b: Headline acceptance test for v1 — the PickPlace state machine
//! picks the block off the table, swings to the bin, and drops it. Exercises
//! the full pipeline (gravity, settled-grasp, port wiring, harness loop, goal
//! evaluation) end-to-end.
//!
//! Step 8.4: To inspect a run visually with the rerun viewer:
//!   1. Install the viewer once: `cargo install rerun-cli --version 0.21`
//!   2. Run the rrd-saving variant:
//!      `cargo test -p rtf_arm --features viz-rerun --test pick_place_e2e \
//!         state_machine_picks_block_and_drops_in_bin_save_rrd -- --nocapture`
//!   3. Open the printed `.rrd` file (path is platform-dependent — Linux
//!      `/tmp/pick_place.rrd`, macOS `$TMPDIR/pick_place.rrd`):
//!      `rerun <printed-path>`

#![cfg(feature = "e2e")]

use rtf_arm::{
    examples_::PickPlace,
    goals::PlaceInBin,
    test_helpers::{block_id, bin_id, build_pick_and_place_world},
    RateHz,
};
use rtf_core::time::Duration;
use rtf_harness::{run, RunConfig, Termination};

#[test]
fn state_machine_picks_block_and_drops_in_bin() {
    let mut world = build_pick_and_place_world();
    let ports = world.attach_standard_arm_ports();
    let ee_pose_rx = world.attach_ee_pose_sensor(RateHz::new(100));
    let block = block_id(&world);
    let bin = bin_id(&world);

    let controller = PickPlace::new(
        ports.encoder_rxs,
        ee_pose_rx,
        ports.velocity_txs,
        ports.gripper_tx,
        /* target_block_xy */ (0.6, 0.0),
        /* target_bin_xy */ (0.0, 0.6),
        /* arm_shoulder_z */ 0.8,
        /* l1 */ 0.4,
        /* l2 */ 0.4,
    );
    let goal = PlaceInBin::new(block, bin);

    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(15))
        .with_seed(42);

    let res = run(world, controller, goal, cfg);
    eprintln!(
        "e2e PickPlace: terminated_by={:?}, final_time={:?}, score={}",
        res.terminated_by, res.final_time, res.score.value,
    );
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "did not converge in 15s; final score={}, terminated_by={:?}",
        res.score.value, res.terminated_by,
    );
    assert!(res.score.value > 0.9);
}

#[cfg(feature = "viz-rerun")]
#[test]
fn state_machine_picks_block_and_drops_in_bin_with_rerun() {
    use rtf_viz::RerunRecorder;

    let rec = RerunRecorder::in_memory("pick_place").unwrap();

    let mut world = build_pick_and_place_world();
    let ports = world.attach_standard_arm_ports();
    let ee_pose_rx = world.attach_ee_pose_sensor(RateHz::new(100));
    let block = block_id(&world);
    let bin = bin_id(&world);

    let controller = PickPlace::new(
        ports.encoder_rxs,
        ee_pose_rx,
        ports.velocity_txs,
        ports.gripper_tx,
        (0.6, 0.0),
        (0.0, 0.6),
        0.8, 0.4, 0.4,
    );
    let goal = PlaceInBin::new(block, bin);

    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(15))
        .with_seed(42)
        .with_recorder(rec);

    let res = run(world, controller, goal, cfg);
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "did not converge with rerun recorder; final score = {}",
        res.score.value,
    );
}

#[cfg(feature = "viz-rerun")]
#[test]
fn state_machine_picks_block_and_drops_in_bin_save_rrd() {
    use rtf_viz::RerunRecorder;

    let path = std::env::temp_dir().join("pick_place.rrd");
    let _ = std::fs::remove_file(&path);
    let rec = RerunRecorder::save_to_file(&path, "pick_place").unwrap();

    let mut world = build_pick_and_place_world();
    let ports = world.attach_standard_arm_ports();
    let ee_pose_rx = world.attach_ee_pose_sensor(RateHz::new(100));
    let block = block_id(&world);
    let bin = bin_id(&world);

    let controller = PickPlace::new(
        ports.encoder_rxs,
        ee_pose_rx,
        ports.velocity_txs,
        ports.gripper_tx,
        (0.6, 0.0),
        (0.0, 0.6),
        0.8, 0.4, 0.4,
    );
    let goal = PlaceInBin::new(block, bin);

    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(15))
        .with_seed(42)
        .with_recorder(rec);

    let res = run(world, controller, goal, cfg);
    eprintln!("Saved rrd file to: {:?}", path);
    eprintln!("View with: rerun {:?}", path);
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "did not converge while saving rrd; final score = {}",
        res.score.value,
    );
}

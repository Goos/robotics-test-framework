//! Step 7.4b: Headline acceptance test for v1 — the PickPlace state machine
//! picks the block off the table, swings to the bin, and drops it. Exercises
//! the full pipeline (gravity, settled-grasp, port wiring, harness loop, goal
//! evaluation) end-to-end.

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

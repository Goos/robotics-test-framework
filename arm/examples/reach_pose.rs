//! Phase 5 milestone scenario: PD controller reaches a 3-joint target pose
//! within 2 seconds. First scenario that exercises the full pipeline:
//! ArmWorld → harness tick loop → PdJointController → ReachPose goal →
//! GoalComplete termination.
//!
//! Run a single demo (records to $TMPDIR/reach_pose.rrd if --features viz-rerun):
//!   `cargo run --example reach_pose --features examples`
//!   `cargo run --example reach_pose --features viz-rerun`
//!
//! Run as a test (records to $TMPDIR/<test_name>.rrd if --features viz-rerun):
//!   `cargo test --example reach_pose --features examples`
//!   `cargo test --example reach_pose --features viz-rerun`

use rtf_arm::{
    fk::forward_kinematics, goals::reach_pose::ReachPose, test_helpers::build_simple_arm_world,
    PdJointController,
};
use rtf_core::time::Duration;
#[cfg(test)]
use rtf_harness::Termination;
use rtf_harness::{run, RunConfig};

fn run_reach_pose(rrd_name: &str) -> rtf_harness::RunResult {
    let _ = rrd_name; // used only when viz-rerun is on
    let mut world = build_simple_arm_world(3);
    let ports = world.attach_standard_arm_ports();

    let target_q = vec![0.5_f32; 3];
    let controller =
        PdJointController::new(target_q.clone(), ports.encoder_rxs, ports.velocity_txs);
    let target_ee = forward_kinematics(&world.arm.spec, &target_q);
    let goal = ReachPose::new(target_ee, 0.01);

    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(2))
        .with_tick_rate(1000)
        .with_seed(42);

    #[cfg(feature = "viz-rerun")]
    {
        match rtf_viz::maybe_recorder_for(rrd_name) {
            Some(rec) => run(world, controller, goal, cfg.with_recorder(rec)),
            None => run(world, controller, goal, cfg),
        }
    }
    #[cfg(not(feature = "viz-rerun"))]
    {
        run(world, controller, goal, cfg)
    }
}

fn main() {
    let res = run_reach_pose("reach_pose");
    println!(
        "ReachPose: terminated_by={:?}, final_time_ns={}, score={}",
        res.terminated_by,
        res.final_time.as_nanos(),
        res.score.value,
    );
}

#[test]
fn pd_reaches_target_pose_within_2_seconds() {
    let res = run_reach_pose("pd_reaches_target_pose_within_2_seconds");
    eprintln!(
        "e2e: terminated_by={:?}, final_time_ns={}, score={}",
        res.terminated_by,
        res.final_time.as_nanos(),
        res.score.value,
    );
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "did not converge in time; final score = {}",
        res.score.value
    );
    // Sanity check: PD must actually have run for measurable sim time.
    // (Catches degenerate-geometry traps where t=0 already satisfies the goal.)
    assert!(
        res.final_time > rtf_core::time::Time::from_millis(1),
        "suspicious: GoalComplete fired at t={:?} — likely degenerate test geometry",
        res.final_time
    );
}

//! End-to-end smoke test (Phase 5 milestone): PD controller reaches a 3-joint
//! target pose within 2 seconds. First test that exercises the full pipeline:
//! ArmWorld → harness tick loop → PdJointController → ReachPose goal →
//! GoalComplete termination.

#![cfg(feature = "e2e")]

use rtf_arm::{
    examples_::PdJointController,
    fk::forward_kinematics,
    goals::reach_pose::ReachPose,
    test_helpers::build_simple_arm_world,
};
use rtf_core::time::Duration;
use rtf_harness::{run, RunConfig, Termination};

#[test]
fn pd_reaches_target_pose_within_2_seconds() {
    let mut world = build_simple_arm_world(3);
    let ports = world.attach_standard_arm_ports();

    let target_q = vec![0.5_f32; 3];
    let controller = PdJointController::new(target_q.clone(), ports.encoder_rxs, ports.velocity_txs);
    let target_ee = forward_kinematics(&world.arm.spec, &target_q);
    let goal = ReachPose::new(target_ee, 0.01);

    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(2))
        .with_tick_rate(1000)
        .with_seed(42);

    let res = run(world, controller, goal, cfg);
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "did not converge in time; final score = {}", res.score.value
    );
}

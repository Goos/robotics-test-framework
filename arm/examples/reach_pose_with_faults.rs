//! Step 9.6 robustness scenario: PD controller still reaches the target pose
//! when each encoder is wrapped with GaussianNoise (sigma=5e-3 rad) followed
//! by Delay (2 ms latency). Validates that the fault wrappers satisfy
//! `Box<dyn PortReader<JointEncoderReading>>` and that the PD gains are
//! robust enough to absorb light sensor faults.
//!
//! Run a single demo (records to $TMPDIR/reach_pose_with_faults.rrd if --features viz-rerun):
//!   `cargo run --example reach_pose_with_faults --features examples`
//!   `cargo run --example reach_pose_with_faults --features viz-rerun`
//!
//! Run as a test:
//!   `cargo test --example reach_pose_with_faults --features examples`

use std::rc::Rc;

use rtf_arm::{
    fk::forward_kinematics,
    goals::reach_pose::ReachPose,
    ports::{JointEncoderReading, JointId},
    test_helpers::build_simple_arm_world,
    world::RateHz,
    PdJointController,
};
use rtf_core::{clock::Clock, port::PortReader, time::Duration};
#[cfg(test)]
use rtf_harness::Termination;
use rtf_harness::{run, RunConfig};
use rtf_sim::faults::{Delay, GaussianNoise};

fn run_reach_pose_with_faults(rrd_name: &str) -> rtf_harness::RunResult {
    run_reach_pose_with_faults_with(rrd_name, /* debug_overlay */ false)
}

fn run_reach_pose_with_faults_with(rrd_name: &str, debug_overlay: bool) -> rtf_harness::RunResult {
    let _ = rrd_name; // used only when viz-rerun is on
    let mut world = build_simple_arm_world(3);
    if debug_overlay {
        world.enable_debug_overlay(true);
    }
    let raw_clock = world.sim_clock_handle();
    let clock_dyn: Rc<dyn Clock> = raw_clock.clone();

    let raw_rxs: Vec<_> = (0..3)
        .map(|i| world.attach_joint_encoder_sensor(JointId(i as u32), RateHz::new(1000)))
        .collect();
    let velocity_txs: Vec<_> = (0..3)
        .map(|i| world.attach_joint_velocity_actuator(JointId(i as u32)))
        .collect();

    // Compose inside-out: GaussianNoise wraps the raw rx, then Delay wraps
    // the noisy reader. Boxed as `Box<dyn PortReader<...>>` to give the
    // controller a single uniform R type per joint.
    let wrapped_rxs: Vec<Box<dyn PortReader<JointEncoderReading>>> = raw_rxs
        .into_iter()
        .map(|rx| {
            let noisy = GaussianNoise::new(rx, 0.005_f32, 7);
            let delayed = Delay::new(noisy, Duration::from_millis(2), Rc::clone(&clock_dyn));
            Box::new(delayed) as Box<dyn PortReader<JointEncoderReading>>
        })
        .collect();

    let target_q = vec![0.5_f32; 3];
    let target_ee = forward_kinematics(&world.arm.spec, &target_q);
    let controller = PdJointController::new(target_q, wrapped_rxs, velocity_txs);
    let goal = ReachPose::new(target_ee, 0.02);

    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(3))
        .with_tick_rate(1000)
        .with_seed(42);

    #[cfg(feature = "viz-rerun")]
    {
        if std::env::var("RTF_DEBUG_OVERLAY").as_deref() == Ok("1") {
            world.enable_debug_overlay(true);
        }
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
    let res = run_reach_pose_with_faults("reach_pose_with_faults");
    println!(
        "ReachPose+faults: terminated_by={:?}, final_time_ns={}, score={}",
        res.terminated_by,
        res.final_time.as_nanos(),
        res.score.value,
    );
}

#[test]
fn pd_still_reaches_target_with_2ms_encoder_delay_and_noise() {
    let res =
        run_reach_pose_with_faults("pd_still_reaches_target_with_2ms_encoder_delay_and_noise");
    eprintln!(
        "e2e (faults): terminated_by={:?}, final_time_ns={}, score={}",
        res.terminated_by,
        res.final_time.as_nanos(),
        res.score.value,
    );
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "did not converge under faults; final score={}, terminated_by={:?}",
        res.score.value,
        res.terminated_by,
    );
}

/// Sanity-check: the Rapier debug overlay doesn't break ReachPose+faults.
#[test]
fn pd_reaches_target_with_faults_and_debug_overlay() {
    let res = run_reach_pose_with_faults_with("pd_reaches_target_with_faults_overlay", true);
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "ReachPose+faults (overlay on) did not converge; terminated_by={:?}, score={}",
        res.terminated_by,
        res.score.value,
    );
}

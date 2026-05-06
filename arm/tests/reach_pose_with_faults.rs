//! Step 9.6: Robustness end-to-end — PD controller still reaches the target
//! pose when each encoder is wrapped with GaussianNoise (sigma=5e-3 rad)
//! followed by Delay (2 ms latency). Validates that the fault wrappers
//! satisfy `Box<dyn PortReader<JointEncoderReading>>` (Step 9.1) and that
//! the PD gains are robust enough to absorb light sensor faults.

#![cfg(feature = "e2e")]

use std::rc::Rc;

use rtf_arm::{
    examples_::PdJointController,
    fk::forward_kinematics,
    goals::reach_pose::ReachPose,
    ports::{JointEncoderReading, JointId},
    test_helpers::build_simple_arm_world,
    world::RateHz,
};
use rtf_core::{
    clock::Clock,
    port::PortReader,
    time::Duration,
};
use rtf_harness::{run, RunConfig, Termination};
use rtf_sim::faults::{Delay, GaussianNoise};

#[test]
fn pd_still_reaches_target_with_2ms_encoder_delay_and_noise() {
    let mut world = build_simple_arm_world(3);
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

    let res = run(world, controller, goal, cfg);
    eprintln!(
        "e2e (faults): terminated_by={:?}, final_time_ns={}, score={}",
        res.terminated_by, res.final_time.as_nanos(), res.score.value,
    );
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "did not converge under faults; final score={}, terminated_by={:?}",
        res.score.value, res.terminated_by,
    );
}

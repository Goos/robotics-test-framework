use std::collections::BTreeMap;
use std::rc::Rc;

use rtf_core::port::{PortRx, PortTx};
use rtf_core::port_id::PortId;
use rtf_core::time::Time;
use rtf_sim::rate_scheduler::RateScheduler;
use rtf_sim::scene::Scene;
use rtf_sim::sim_clock::SimClock;

use crate::arm::Arm;
use crate::ports::{
    EePoseReading, GripperCommand, JointEncoderReading, JointId, JointVelocityCommand,
};
use crate::spec::ArmSpec;
use crate::state::ArmState;

/// Sample rate (Hz) for a sensor port. Newtype to keep call-sites
/// self-documenting and to allow attach helpers to compose with `RateScheduler`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RateHz(pub u32);

impl RateHz {
    pub fn new(hz: u32) -> Self { RateHz(hz) }
}

/// Per-joint encoder publisher: the channel sender, the joint it samples,
/// and the rate-scheduler that decides when to push a fresh reading.
/// Consumed in Step 3.11a (`publish_sensors` joint-encoder branch).
#[allow(dead_code)]
pub(crate) struct EncoderPublisher {
    pub joint: JointId,
    pub tx: PortTx<JointEncoderReading>,
    pub scheduler: RateScheduler,
}

/// EE-pose publisher: sender + scheduler. Consumed in Step 3.11b.
#[allow(dead_code)]
pub(crate) struct EePosePublisher {
    pub tx: PortTx<EePoseReading>,
    pub scheduler: RateScheduler,
}

/// Per-joint velocity-command receiver. Consumed in Step 3.11c.
#[allow(dead_code)]
pub(crate) struct JointVelocityConsumer {
    pub joint: JointId,
    pub rx: PortRx<JointVelocityCommand>,
}

/// Gripper command receiver. Consumed in Step 3.11d.
#[allow(dead_code)]
pub(crate) struct GripperConsumer {
    pub rx: PortRx<GripperCommand>,
}

/// Tickable arm world: scene + arm + per-tick port registries (design v2 §5.7).
/// `gravity_enabled` toggles the per-tick gravity step in §5.5.
pub struct ArmWorld {
    pub scene: Scene,
    pub arm: Arm,
    pub gravity_enabled: bool,
    /// Authoritative sim time. Advanced from Step 3.12 (`tick`).
    #[allow(dead_code)]
    sim_time: Time,
    /// Shared injectable clock. Exposed via `sim_clock_handle` (Step 3.7.5);
    /// advanced in Step 3.12. Phase 9 fault wrappers also clone this Rc.
    sim_clock: Rc<SimClock>,
    /// Monotonic port-id counter. Allocated by Steps 3.8–3.10 attach helpers.
    next_port_id: u32,
    /// Filled by `attach_joint_encoder_sensor` (Step 3.8); drained in Step 3.11a.
    pub(crate) sensors_joint_encoder: BTreeMap<PortId, EncoderPublisher>,
    /// Filled by Step 3.10 (`attach_ee_pose_sensor`); drained in Step 3.11b.
    #[allow(dead_code)]
    pub(crate) sensors_ee_pose: BTreeMap<PortId, EePosePublisher>,
    /// Filled by Step 3.9 (`attach_joint_velocity_actuator`); drained in Step 3.11c.
    #[allow(dead_code)]
    pub(crate) actuators_joint_velocity: BTreeMap<PortId, JointVelocityConsumer>,
    /// Filled by Step 3.10 (`attach_gripper_actuator`); drained in Step 3.11d.
    #[allow(dead_code)]
    pub(crate) actuators_gripper: BTreeMap<PortId, GripperConsumer>,
}

impl ArmWorld {
    pub fn new(scene: Scene, spec: ArmSpec, gravity_enabled: bool) -> Self {
        let n = spec.joints.len();
        Self {
            scene,
            arm: Arm { spec, state: ArmState::zeros(n), id: 0 },
            gravity_enabled,
            sim_time: Time::ZERO,
            sim_clock: Rc::new(SimClock::new()),
            next_port_id: 0,
            sensors_joint_encoder: BTreeMap::new(),
            sensors_ee_pose: BTreeMap::new(),
            actuators_joint_velocity: BTreeMap::new(),
            actuators_gripper: BTreeMap::new(),
        }
    }

    pub fn time(&self) -> Time { self.sim_time }

    /// Shareable handle to the world's `SimClock`. Multiple holders observe
    /// the same time advancement; callers must respect the design's
    /// single-threaded contract (no `Send`/`Sync` on `SimClock`).
    pub fn sim_clock_handle(&self) -> Rc<SimClock> {
        Rc::clone(&self.sim_clock)
    }

    /// Register a joint-encoder sensor on `joint` publishing at `rate` Hz.
    /// Returns the receiver end of a fresh single-consumer port; the world
    /// retains the sender + scheduler and pushes readings during `publish_sensors`.
    pub fn attach_joint_encoder_sensor(
        &mut self,
        joint: JointId,
        rate: RateHz,
    ) -> PortRx<JointEncoderReading> {
        let (tx, rx) = rtf_core::port::port::<JointEncoderReading>();
        let port_id = PortId(self.next_port_id);
        self.next_port_id += 1;
        self.sensors_joint_encoder.insert(port_id, EncoderPublisher {
            joint,
            tx,
            scheduler: RateScheduler::new_hz(rate.0),
        });
        rx
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rtf_sim::scene::Scene;

    fn simple_spec() -> crate::spec::ArmSpec {
        use crate::spec::{ArmSpec, GripperSpec, JointSpec};
        use core::f32::consts::PI;
        use nalgebra::{Isometry3, Vector3};
        ArmSpec {
            joints: vec![JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-PI, PI) }; 2],
            link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.1); 2],
            gripper: GripperSpec { proximity_threshold: 0.02, max_grasp_size: 0.05 },
        }
    }

    #[test]
    fn arm_world_is_constructible_with_gravity_default_on() {
        let world = ArmWorld::new(Scene::new(0), simple_spec(), /* gravity */ true);
        assert!(world.gravity_enabled);
        assert_eq!(world.arm.state.q.len(), simple_spec().joints.len());
        assert_eq!(world.time(), rtf_core::time::Time::ZERO);
    }

    #[test]
    fn sim_clock_handle_returns_sharable_rc() {
        use std::rc::Rc;
        let world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        let h1 = world.sim_clock_handle();
        let h2 = world.sim_clock_handle();
        assert!(Rc::ptr_eq(&h1, &h2));
    }

    #[test]
    fn attaching_two_encoders_yields_distinct_port_ids() {
        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        let _rx_a = world.attach_joint_encoder_sensor(JointId(0), RateHz::new(1000));
        let _rx_b = world.attach_joint_encoder_sensor(JointId(1), RateHz::new(1000));
        assert_eq!(world.sensors_joint_encoder.len(), 2);
    }
}

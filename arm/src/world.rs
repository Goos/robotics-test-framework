use std::collections::BTreeMap;
use std::rc::Rc;

use rtf_core::port::{PortRx, PortTx};
use rtf_core::port_id::PortId;
use rtf_core::time::{Duration, Time};
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
    /// Filled by `attach_ee_pose_sensor` (Step 3.10); drained in Step 3.11b.
    pub(crate) sensors_ee_pose: BTreeMap<PortId, EePosePublisher>,
    /// Filled by `attach_joint_velocity_actuator` (Step 3.9); drained in Step 3.11c.
    pub(crate) actuators_joint_velocity: BTreeMap<PortId, JointVelocityConsumer>,
    /// Filled by `attach_gripper_actuator` (Step 3.10); drained in Step 3.11d.
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

    /// Current end-effector world-frame pose, computed via forward kinematics
    /// from the arm's spec and joint state. Used by `publish_sensors` and by
    /// goal/proximity checks (Phase 5+).
    pub fn ee_pose(&self) -> nalgebra::Isometry3<f32> {
        crate::fk::forward_kinematics(&self.arm.spec, &self.arm.state.q)
    }

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

    /// Register a joint-velocity-command actuator on `joint`. Returns the
    /// sender end; the world retains the receiver and drains it during
    /// `consume_actuators` (Step 3.11c) to update the integrated joint state.
    pub fn attach_joint_velocity_actuator(
        &mut self,
        joint: JointId,
    ) -> PortTx<JointVelocityCommand> {
        let (tx, rx) = rtf_core::port::port::<JointVelocityCommand>();
        let port_id = PortId(self.next_port_id);
        self.next_port_id += 1;
        self.actuators_joint_velocity.insert(port_id, JointVelocityConsumer { joint, rx });
        tx
    }

    /// Register a gripper-command actuator. Returns the sender end; the world
    /// retains the receiver and drains it during `consume_actuators` (Step 3.11d).
    pub fn attach_gripper_actuator(&mut self) -> PortTx<GripperCommand> {
        let (tx, rx) = rtf_core::port::port::<GripperCommand>();
        let port_id = PortId(self.next_port_id);
        self.next_port_id += 1;
        self.actuators_gripper.insert(port_id, GripperConsumer { rx });
        tx
    }

    /// Register an end-effector pose sensor publishing at `rate` Hz. Returns
    /// the receiver end; the world retains the sender + scheduler and pushes
    /// EE-pose readings during `publish_sensors` (Step 3.11b).
    pub fn attach_ee_pose_sensor(&mut self, rate: RateHz) -> PortRx<EePoseReading> {
        let (tx, rx) = rtf_core::port::port::<EePoseReading>();
        let port_id = PortId(self.next_port_id);
        self.next_port_id += 1;
        self.sensors_ee_pose.insert(port_id, EePosePublisher {
            tx,
            scheduler: RateScheduler::new_hz(rate.0),
        });
        rx
    }

    /// Drain all actuator ports, integrate joint state forward by `dt`, and
    /// advance the sim clock (design v2 §5.7). Velocity, gripper, and
    /// grasped-object pose tracking as of Step 3.11e; gravity in Phase 6.
    pub fn consume_actuators_and_integrate(&mut self, dt: Duration) {
        let dt_s = dt.as_nanos() as f32 / 1.0e9_f32;

        // Drain joint-velocity commands; latest wins per port. Collect first
        // so we don't hold a borrow on `self.actuators_joint_velocity` while
        // we mutate `self.arm.state.q_dot` in the apply pass. The Vec is
        // bounded by the number of attached velocity actuators (small).
        let updates: Vec<(JointId, f32)> = self
            .actuators_joint_velocity
            .values_mut()
            .filter_map(|cons| cons.rx.latest().map(|cmd| (cmd.joint, cmd.q_dot_target)))
            .collect();
        for (joint, q_dot_target) in updates {
            self.arm.state.q_dot[joint.0 as usize] = q_dot_target;
        }

        // Forward-Euler integrate joint positions: q += q_dot * dt.
        for i in 0..self.arm.state.q.len() {
            self.arm.state.q[i] += self.arm.state.q_dot[i] * dt_s;
        }

        // Drain gripper command; latest across all ports wins. Collected
        // before mutating arm/scene state for the same alias reason.
        let gripper_cmd: Option<GripperCommand> = self
            .actuators_gripper
            .values_mut()
            .filter_map(|cons| cons.rx.latest())
            .last();

        if let Some(cmd) = gripper_cmd {
            self.apply_gripper_command(cmd);
        }

        // Grasped object is welded to the EE — refresh its pose from the
        // post-integration EE so the visualization and downstream queries see
        // it move with the arm (design v2 §5.4).
        if let Some(grasped_id) = self.arm.state.grasped {
            let ee = self.ee_pose();
            if let Some(obj) = self.scene.object_mut(grasped_id) {
                obj.pose = ee;
            }
        }

        // Advance authoritative sim time and the shared clock together.
        self.sim_time = self.sim_time + dt;
        self.sim_clock.advance(dt);
    }

    /// Apply a single gripper command — flip `gripper_closed`, then handle the
    /// open→release and closed→try-grasp transitions per design v2 §5.4.
    fn apply_gripper_command(&mut self, cmd: GripperCommand) {
        let was_closed = self.arm.state.gripper_closed;
        let now_closed = cmd.close;
        self.arm.state.gripper_closed = now_closed;

        // Opening transition: release any held object back to Free.
        if was_closed && !now_closed {
            if let Some(grasped_id) = self.arm.state.grasped.take() {
                if let Some(obj) = self.scene.object_mut(grasped_id) {
                    obj.state = rtf_sim::object::ObjectState::Free;
                }
            }
        }

        // Closing transition (and nothing currently held): try to grasp the
        // first graspable Free object within proximity_threshold of the EE.
        // Iteration is by ObjectId (BTreeMap) so the choice is deterministic.
        if !was_closed && now_closed && self.arm.state.grasped.is_none() {
            let ee = self.ee_pose();
            let threshold = self.arm.spec.gripper.proximity_threshold;
            let arm_id = self.arm.id;
            let to_grasp = self.scene.objects().find_map(|(id, obj)| {
                if !obj.graspable { return None; }
                if !matches!(obj.state, rtf_sim::object::ObjectState::Free) { return None; }
                let dist = (obj.pose.translation.vector - ee.translation.vector).norm();
                (dist <= threshold).then_some(*id)
            });
            if let Some(id) = to_grasp {
                self.arm.state.grasped = Some(id);
                if let Some(obj) = self.scene.object_mut(id) {
                    obj.state = rtf_sim::object::ObjectState::Grasped {
                        by: rtf_sim::object::ArmRef(arm_id),
                    };
                }
            }
        }
    }

    /// Advance every sensor scheduler by `dt` and publish a fresh reading on
    /// each port that fires this step. Iteration order is `PortId` (BTreeMap),
    /// keeping per-tick output deterministic across runs (design v2 §10.2).
    pub fn publish_sensors_for_dt(&mut self, dt: Duration) {
        let dt_ns = dt.as_nanos();
        let sampled_at = self.sim_time;

        for pubr in self.sensors_joint_encoder.values_mut() {
            if pubr.scheduler.tick(dt_ns) {
                let i = pubr.joint.0 as usize;
                pubr.tx.send(JointEncoderReading {
                    joint: pubr.joint,
                    q: self.arm.state.q[i],
                    q_dot: self.arm.state.q_dot[i],
                    sampled_at,
                });
            }
        }

        // Compute EE pose once per tick (design v2 §5.7 caches FK across all
        // EE-pose subscribers). FK is borrowed before grabbing the mut iter
        // because `values_mut` on `sensors_ee_pose` would alias `&self.arm`.
        let ee_pose = if self.sensors_ee_pose.is_empty() {
            None
        } else {
            Some(self.ee_pose())
        };
        if let Some(pose) = ee_pose {
            for pubr in self.sensors_ee_pose.values_mut() {
                if pubr.scheduler.tick(dt_ns) {
                    pubr.tx.send(EePoseReading { pose, sampled_at });
                }
            }
        }
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

    /// Test fixture for grasp-tracking tests: x-axis link offsets so a single
    /// joint-0 rotation about z swings the EE in the xy plane (the default
    /// `simple_spec` has z-axis offsets, which leaves the EE on the rotation
    /// axis and therefore invariant under joint motion).
    fn moving_ee_spec() -> crate::spec::ArmSpec {
        use crate::spec::{ArmSpec, GripperSpec, JointSpec};
        use nalgebra::{Isometry3, Vector3};
        ArmSpec {
            joints: vec![JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-3.2, 3.2) }; 2],
            link_offsets: vec![Isometry3::translation(0.5, 0.0, 0.0); 2],
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

    #[test]
    fn attach_velocity_actuator_returns_tx_and_registers_consumer() {
        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        let _tx = world.attach_joint_velocity_actuator(JointId(0));
        assert_eq!(world.actuators_joint_velocity.len(), 1);
    }

    #[test]
    fn attach_gripper_returns_tx_and_registers_consumer() {
        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        let _tx = world.attach_gripper_actuator();
        assert_eq!(world.actuators_gripper.len(), 1);
    }

    #[test]
    fn attach_ee_pose_returns_rx_and_registers_publisher() {
        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        let _rx = world.attach_ee_pose_sensor(RateHz::new(100));
        assert_eq!(world.sensors_ee_pose.len(), 1);
    }

    #[test]
    fn publish_sensors_emits_encoder_at_rate() {
        use rtf_core::time::Duration;
        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        let rx = world.attach_joint_encoder_sensor(JointId(0), RateHz::new(1000));
        world.publish_sensors_for_dt(Duration::from_millis(1));
        let r = rx.latest().expect("encoder published");
        assert_eq!(r.joint, JointId(0));
        assert_eq!(r.q, 0.0);
        assert_eq!(r.q_dot, 0.0);
    }

    #[test]
    fn publish_sensors_emits_ee_pose() {
        use rtf_core::time::Duration;
        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        let rx = world.attach_ee_pose_sensor(RateHz::new(100));
        world.publish_sensors_for_dt(Duration::from_millis(10));
        let r = rx.latest().expect("ee pose published");
        assert_eq!(r.sampled_at, world.time());
    }

    #[test]
    fn velocity_command_advances_joint_position() {
        use rtf_core::time::Duration;
        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        let tx = world.attach_joint_velocity_actuator(JointId(0));
        tx.send(JointVelocityCommand { joint: JointId(0), q_dot_target: 1.0 });
        world.consume_actuators_and_integrate(Duration::from_millis(10));
        assert!(world.arm.state.q[0] > 0.0);
    }

    #[test]
    fn closing_gripper_near_graspable_object_attaches_it() {
        use rtf_core::time::Duration;
        use rtf_sim::object::{Object, ObjectId, ObjectState};
        use rtf_sim::shape::Shape;

        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        let ee = world.ee_pose();
        let block_id = ObjectId(42);
        world.scene.insert_object(Object::new(
            block_id, ee, Shape::Sphere { radius: 0.01 }, 0.1, /* graspable */ true,
        ));

        let g_tx = world.attach_gripper_actuator();
        g_tx.send(GripperCommand { close: true });
        world.consume_actuators_and_integrate(Duration::from_millis(1));

        assert_eq!(world.arm.state.grasped, Some(block_id));
        assert!(matches!(
            world.scene.object(block_id).unwrap().state,
            ObjectState::Grasped { .. }
        ));
    }

    #[test]
    fn grasped_object_pose_tracks_ee() {
        use rtf_core::time::Duration;
        use rtf_sim::object::{Object, ObjectId};
        use rtf_sim::shape::Shape;

        let mut world = ArmWorld::new(Scene::new(0), moving_ee_spec(), true);
        let block_id = ObjectId(42);
        world.scene.insert_object(Object::new(
            block_id, world.ee_pose(), Shape::Sphere { radius: 0.01 }, 0.1, true,
        ));

        let v_tx = world.attach_joint_velocity_actuator(JointId(0));
        let g_tx = world.attach_gripper_actuator();
        g_tx.send(GripperCommand { close: true });
        world.consume_actuators_and_integrate(Duration::from_millis(1));
        let pose_before = world.scene.object(block_id).unwrap().pose;

        v_tx.send(JointVelocityCommand { joint: JointId(0), q_dot_target: 1.0 });
        world.consume_actuators_and_integrate(Duration::from_millis(50));
        let pose_after = world.scene.object(block_id).unwrap().pose;

        assert_ne!(pose_before.translation.vector, pose_after.translation.vector);
        let ee = world.ee_pose();
        assert!((pose_after.translation.vector - ee.translation.vector).norm() < 1e-4);
    }
}

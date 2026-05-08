use std::collections::{BTreeMap, VecDeque};
use std::rc::Rc;

use rtf_core::port::{PortRx, PortTx};
use rtf_core::port_id::PortId;
use rtf_core::time::{Duration, Time};
use rtf_sim::object::Object;
use rtf_sim::rate_scheduler::RateScheduler;
use rtf_sim::scene::Scene;
use rtf_sim::sim_clock::SimClock;

use crate::arm::Arm;
use crate::ports::{
    EePoseReading, GripperCommand, JointEncoderReading, JointId, JointVelocityCommand,
    PressureReading,
};
use crate::spec::ArmSpec;
use crate::state::ArmState;

/// Deferred Object insertion: pop from the schedule the first time the
/// world's sim time is at or past `at`, allocate a fresh `ObjectId` from
/// the scene's monotonic counter, and insert. The `template.id` field is
/// ignored — every spawn gets a freshly allocated id so callers can't
/// accidentally collide with other scheduled spawns.
struct PendingSpawn {
    at: Time,
    template: Object,
}

/// Sample rate (Hz) for a sensor port. Newtype to keep call-sites
/// self-documenting and to allow attach helpers to compose with `RateScheduler`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RateHz(pub u32);

impl RateHz {
    pub fn new(hz: u32) -> Self {
        RateHz(hz)
    }
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

/// EE-mounted pressure publisher: sender + scheduler + proximity falloff
/// radius (`eps`). Consumed in `publish_sensors_for_dt` (find-grasp-place
/// design §2.2).
#[allow(dead_code)]
pub(crate) struct PressurePublisher {
    pub tx: PortTx<PressureReading>,
    pub scheduler: RateScheduler,
    pub eps: f32,
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
    /// Authoritative sim time. Advanced by `consume_actuators_and_integrate`
    /// and read by `publish_sensors` (and by the trait `time()` accessor).
    sim_time: Time,
    /// Last `sim_time` we published sensors at. The trait `publish_sensors`
    /// computes `dt = sim_time - last_publish_time`, calls
    /// `publish_sensors_for_dt(dt)`, then updates this watermark.
    last_publish_time: Time,
    /// Shared injectable clock. Exposed via `sim_clock_handle` (Step 3.7.5);
    /// advanced in lockstep with `sim_time`. Phase 9 fault wrappers also
    /// clone this Rc.
    sim_clock: Rc<SimClock>,
    /// Monotonic port-id counter. Allocated by Steps 3.8–3.10 attach helpers.
    next_port_id: u32,
    /// Filled by `attach_joint_encoder_sensor` (Step 3.8); drained in Step 3.11a.
    pub(crate) sensors_joint_encoder: BTreeMap<PortId, EncoderPublisher>,
    /// Filled by `attach_ee_pose_sensor` (Step 3.10); drained in Step 3.11b.
    pub(crate) sensors_ee_pose: BTreeMap<PortId, EePosePublisher>,
    /// Filled by `attach_pressure_sensor` (find-grasp-place design §2.3);
    /// drained in `publish_sensors_for_dt`.
    pub(crate) sensors_pressure: BTreeMap<PortId, PressurePublisher>,
    /// Filled by `attach_joint_velocity_actuator` (Step 3.9); drained in Step 3.11c.
    pub(crate) actuators_joint_velocity: BTreeMap<PortId, JointVelocityConsumer>,
    /// Filled by `attach_gripper_actuator` (Step 3.10); drained in Step 3.11d.
    pub(crate) actuators_gripper: BTreeMap<PortId, GripperConsumer>,
    /// Sorted by `at` (earliest first) so the per-tick drain only needs to
    /// peek the front. Spawns are processed at the end of
    /// `consume_actuators_and_integrate_inner`, after sim_time advances.
    pending_spawns: VecDeque<PendingSpawn>,
    /// Rapier-backed physics world (rapier-integration plan, Phase 1).
    /// Populated at construction with bodies for every fixture, object,
    /// and arm link in the scene. Stepped each tick once Step 1.6 wires
    /// up the tick loop; for now (Step 1.5) the field exists but isn't
    /// driven, so behavior is unchanged.
    #[cfg(feature = "physics-rapier")]
    physics: rtf_sim::physics::PhysicsWorld,
}

impl ArmWorld {
    pub fn new(scene: Scene, spec: ArmSpec, gravity_enabled: bool) -> Self {
        let n = spec.joints.len();
        let arm = Arm {
            spec,
            state: ArmState::zeros(n),
            id: 0,
        };

        // Construct + populate the Rapier physics world. With the
        // `physics-rapier` feature off, this entire block is gated out
        // and behavior is unchanged.
        #[cfg(feature = "physics-rapier")]
        let physics = {
            use rtf_sim::physics::world::ArmLinkShape;
            let mut pw = rtf_sim::physics::PhysicsWorld::new(gravity_enabled);
            for (_, fix) in scene.fixtures() {
                pw.insert_fixture(fix);
            }
            for (_, obj) in scene.objects() {
                pw.insert_object(obj);
            }
            for link in arm.link_poses() {
                pw.insert_arm_link(
                    arm.id,
                    link.slot,
                    link.pose,
                    ArmLinkShape {
                        radius: crate::arm::LINK_RADIUS,
                        half_height: link.half_height,
                    },
                );
            }
            pw
        };

        Self {
            scene,
            arm,
            gravity_enabled,
            sim_time: Time::ZERO,
            last_publish_time: Time::ZERO,
            sim_clock: Rc::new(SimClock::new()),
            next_port_id: 0,
            sensors_joint_encoder: BTreeMap::new(),
            sensors_ee_pose: BTreeMap::new(),
            sensors_pressure: BTreeMap::new(),
            actuators_joint_velocity: BTreeMap::new(),
            actuators_gripper: BTreeMap::new(),
            pending_spawns: VecDeque::new(),
            #[cfg(feature = "physics-rapier")]
            physics,
        }
    }

    /// Read-only accessor for the Rapier physics world. Used by tests
    /// that want to verify body insertion / count; controllers don't
    /// need this and shouldn't reach for Rapier directly.
    #[cfg(feature = "physics-rapier")]
    pub fn physics(&self) -> &rtf_sim::physics::PhysicsWorld {
        &self.physics
    }

    pub fn time(&self) -> Time {
        self.sim_time
    }

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

    /// Schedule an Object to be inserted into the scene at sim time `at`.
    /// The `template.id` field is ignored — the world allocates a fresh
    /// ObjectId from the scene's monotonic counter at spawn time so
    /// schedule entries can never collide.
    ///
    /// Spawns are kept sorted by `at` (earliest first) so the per-tick
    /// drain in `consume_actuators_and_integrate_inner` only needs to peek
    /// the front. Order among entries with equal `at` follows insertion
    /// order (deterministic; design v2 §10.2).
    pub fn schedule_spawn(&mut self, at: Time, template: Object) {
        let entry = PendingSpawn { at, template };
        // Find the first existing spawn with `at` strictly later than
        // the new one and insert just before it; preserves insertion
        // order among equal-time entries.
        let pos = self
            .pending_spawns
            .iter()
            .position(|s| s.at > entry.at)
            .unwrap_or(self.pending_spawns.len());
        self.pending_spawns.insert(pos, entry);
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
        self.sensors_joint_encoder.insert(
            port_id,
            EncoderPublisher {
                joint,
                tx,
                scheduler: RateScheduler::new_hz(rate.0),
            },
        );
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
        self.actuators_joint_velocity
            .insert(port_id, JointVelocityConsumer { joint, rx });
        tx
    }

    /// Register a gripper-command actuator. Returns the sender end; the world
    /// retains the receiver and drains it during `consume_actuators` (Step 3.11d).
    pub fn attach_gripper_actuator(&mut self) -> PortTx<GripperCommand> {
        let (tx, rx) = rtf_core::port::port::<GripperCommand>();
        let port_id = PortId(self.next_port_id);
        self.next_port_id += 1;
        self.actuators_gripper
            .insert(port_id, GripperConsumer { rx });
        tx
    }

    /// Register an EE-mounted pressure sensor publishing at `rate` Hz with
    /// proximity-falloff radius `eps` (find-grasp-place design §2). Returns
    /// the receiver end; the world retains the sender + scheduler and pushes
    /// readings during `publish_sensors`.
    pub fn attach_pressure_sensor(&mut self, rate: RateHz, eps: f32) -> PortRx<PressureReading> {
        let (tx, rx) = rtf_core::port::port::<PressureReading>();
        let port_id = PortId(self.next_port_id);
        self.next_port_id += 1;
        self.sensors_pressure.insert(
            port_id,
            PressurePublisher {
                tx,
                scheduler: RateScheduler::new_hz(rate.0),
                eps,
            },
        );
        rx
    }

    /// Register an end-effector pose sensor publishing at `rate` Hz. Returns
    /// the receiver end; the world retains the sender + scheduler and pushes
    /// EE-pose readings during `publish_sensors` (Step 3.11b).
    pub fn attach_ee_pose_sensor(&mut self, rate: RateHz) -> PortRx<EePoseReading> {
        let (tx, rx) = rtf_core::port::port::<EePoseReading>();
        let port_id = PortId(self.next_port_id);
        self.next_port_id += 1;
        self.sensors_ee_pose.insert(
            port_id,
            EePosePublisher {
                tx,
                scheduler: RateScheduler::new_hz(rate.0),
            },
        );
        rx
    }

    /// Drain all actuator ports, integrate joint state forward by `dt`, and
    /// advance the sim clock (design v2 §5.7). Velocity, gripper, and
    /// grasped-object pose tracking as of Step 3.11e; gravity in Phase 6.
    ///
    /// Named `_inner` so the `RunnableWorld::consume_actuators_and_integrate`
    /// trait method can delegate here without recursing into itself.
    pub fn consume_actuators_and_integrate_inner(&mut self, dt: Duration) {
        // Kinematic-only gravity path (design v2 §5.5) — preserved behind
        // the `not(physics-rapier)` cfg so the default build keeps
        // working through Phase 1 of the Rapier integration. Step 1.13
        // removes this branch entirely once Rapier is the only path.
        #[cfg(not(feature = "physics-rapier"))]
        if self.gravity_enabled {
            rtf_sim::gravity::reevaluate_settled(&mut self.scene);
            rtf_sim::gravity::gravity_step(&mut self.scene, dt.as_nanos());
        }

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

        // Rapier physics step (rapier-integration plan, Step 1.6).
        // Per design §4: update arm-link kinematic body poses from FK,
        // step the pipeline, then sync the resolved Object poses back
        // into the domain Scene. Step 1.7: re-derive Settled/Free state
        // from the post-sync velocities — Rapier doesn't track support
        // chains, so the `on:` field carries SupportId::Unknown.
        #[cfg(feature = "physics-rapier")]
        {
            for link in self.arm.link_poses() {
                self.physics
                    .set_arm_link_pose(self.arm.id, link.slot, link.pose);
            }
            self.physics.step(dt_s);
            self.physics.sync_to_scene(&mut self.scene);

            // Threshold loosened from design §6's 1e-3 to 1e-2 to absorb
            // Rapier's residual lin-vel jitter for objects in stable
            // contact (sub-mm jitter at default solver settings keeps the
            // norm slightly above 1e-3 indefinitely without sleep).
            const SETTLED_VELOCITY_THRESHOLD: f32 = 1e-2;
            for (_, obj) in self.scene.objects_mut() {
                if matches!(obj.state, rtf_sim::object::ObjectState::Grasped { .. }) {
                    continue;
                }
                let resting = obj.lin_vel.norm() < SETTLED_VELOCITY_THRESHOLD;
                obj.state = if resting {
                    rtf_sim::object::ObjectState::Settled {
                        on: rtf_sim::object::SupportId::Unknown,
                    }
                } else {
                    rtf_sim::object::ObjectState::Free
                };
            }
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

        // Process any pending spawns whose `at` time has now elapsed.
        // Scheduled spawns that are due now are inserted in (at, insertion
        // order) — pending_spawns is kept sorted by `at` so we just drain
        // from the front.
        while let Some(spawn) = self.pending_spawns.front() {
            if spawn.at > self.sim_time {
                break;
            }
            let mut spawn = self.pending_spawns.pop_front().unwrap();
            spawn.template.id = self.scene.allocate_object_id();
            self.scene.insert_object(spawn.template);
        }
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
        // first graspable, not-already-Grasped object within
        // proximity_threshold of the EE. Per design v2 §5.4, only `Grasped`
        // is excluded — `Free` and `Settled` are both eligible (Phase 7's
        // PickObject starts with the block Settled on a table fixture).
        // Iteration is by ObjectId (BTreeMap) so the choice is deterministic.
        if !was_closed && now_closed && self.arm.state.grasped.is_none() {
            let ee = self.ee_pose();
            let threshold = self.arm.spec.gripper.proximity_threshold;
            let arm_id = self.arm.id;
            let to_grasp = self.scene.objects().find_map(|(id, obj)| {
                if !obj.graspable {
                    return None;
                }
                if matches!(obj.state, rtf_sim::object::ObjectState::Grasped { .. }) {
                    return None;
                }
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
        // Pressure publishers also need the EE position, so share the same
        // cached pose.
        let ee_pose = if self.sensors_ee_pose.is_empty() && self.sensors_pressure.is_empty() {
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

            // Pressure: scan only graspable Objects for the closest surface
            // (find-grasp-place design §2.2 amended) — Fixtures are support
            // surfaces, not contact targets, and including them caused
            // spurious sweep-altitude triggers when joint-space interpolation
            // dipped EE z toward the table top.
            if !self.sensors_pressure.is_empty() {
                let ee_point = nalgebra::Point3::from(pose.translation.vector);
                for pubr in self.sensors_pressure.values_mut() {
                    if !pubr.scheduler.tick(dt_ns) {
                        continue;
                    }
                    let eps = pubr.eps;
                    let mut max_pressure = 0.0_f32;
                    for (_, obj) in self.scene.objects() {
                        let d = obj.shape.distance_to_surface(&obj.pose, &ee_point);
                        if d <= eps {
                            max_pressure = max_pressure.max((eps - d) / eps);
                        }
                    }
                    pubr.tx.send(PressureReading {
                        pressure: max_pressure,
                        sampled_at,
                    });
                }
            }
        }
    }

    /// Look up a scene object by id. Convenience pass-through used by goal
    /// and test code so callers don't need to reach through `world.scene`.
    pub fn object(&self, id: rtf_sim::object::ObjectId) -> Option<&rtf_sim::object::Object> {
        self.scene.object(id)
    }
}

impl rtf_core::world_view::WorldView for ArmWorld {}

impl rtf_sim::runnable_world::RunnableWorld for ArmWorld {
    /// Compute the elapsed `dt` since the last publish, advance every sensor
    /// scheduler, and refresh the watermark. Bridges the trait's no-arg shape
    /// to the inherent `publish_sensors_for_dt(dt)` (design v2 §5.7).
    fn publish_sensors(&mut self) {
        let dt = self.sim_time - self.last_publish_time;
        self.publish_sensors_for_dt(dt);
        self.last_publish_time = self.sim_time;
    }

    fn consume_actuators_and_integrate(&mut self, dt: Duration) {
        self.consume_actuators_and_integrate_inner(dt);
    }

    fn snapshot(&self) -> rtf_sim::primitive::SceneSnapshot {
        use rtf_sim::visualizable::Visualizable;
        let mut items = Vec::new();
        for (_, fix) in self.scene.fixtures() {
            fix.append_primitives(&mut items);
        }
        for (_, obj) in self.scene.objects() {
            obj.append_primitives(&mut items);
        }
        self.arm.append_primitives(&mut items);
        rtf_sim::primitive::SceneSnapshot {
            t: self.sim_time,
            items,
        }
    }

    fn time(&self) -> Time {
        self.sim_time
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
            joints: vec![
                JointSpec::Revolute {
                    axis: Vector3::z_axis(),
                    limits: (-PI, PI)
                };
                2
            ],
            link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.1); 2],
            gripper: GripperSpec {
                proximity_threshold: 0.02,
                max_grasp_size: 0.05,
            },
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
            joints: vec![
                JointSpec::Revolute {
                    axis: Vector3::z_axis(),
                    limits: (-3.2, 3.2)
                };
                2
            ],
            link_offsets: vec![Isometry3::translation(0.5, 0.0, 0.0); 2],
            gripper: GripperSpec {
                proximity_threshold: 0.02,
                max_grasp_size: 0.05,
            },
        }
    }

    #[test]
    fn arm_world_is_constructible_with_gravity_default_on() {
        let world = ArmWorld::new(Scene::new(0), simple_spec(), /* gravity */ true);
        assert!(world.gravity_enabled);
        assert_eq!(world.arm.state.q.len(), simple_spec().joints.len());
        assert_eq!(world.time(), rtf_core::time::Time::ZERO);
    }

    #[cfg(feature = "physics-rapier")]
    #[test]
    fn armworld_with_physics_feature_owns_physics_world_with_all_bodies() {
        // Empty scene + 2-joint simple_spec = 2 arm-link bodies, 0 fixtures,
        // 0 objects → physics body_count == 2.
        let world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        assert_eq!(
            world.physics().body_count(),
            simple_spec().joints.len(),
            "expected one physics body per arm link"
        );
    }

    #[cfg(feature = "physics-rapier")]
    #[test]
    fn object_transitions_to_settled_when_velocity_below_threshold() {
        // Build a world with a sphere already at rest on a flat fixture
        // (initial pose puts it touching the fixture top). After a few
        // steps it stays Settled.
        use nalgebra::{Isometry3, Vector3};
        use rtf_sim::fixture::Fixture;
        use rtf_sim::object::{Object, ObjectId, ObjectState, SupportId};
        use rtf_sim::shape::Shape;

        let mut scene = Scene::new(0);
        scene.add_fixture(Fixture {
            id: 0,
            pose: Isometry3::translation(0.0, 0.0, 0.0),
            shape: Shape::Aabb {
                half_extents: Vector3::new(1.0, 1.0, 0.05),
            },
            is_support: true,
        });
        scene.insert_object(Object::new(
            ObjectId(1),
            // Sphere centre at z = fixture_top + radius = 0.05 + 0.05 = 0.10.
            Isometry3::translation(0.0, 0.0, 0.10),
            Shape::Sphere { radius: 0.05 },
            0.1,
            true,
        ));
        let mut world = ArmWorld::new(scene, simple_spec(), true);

        // 200 ms of stepping; sphere starts at rest and stays at rest.
        for _ in 0..200 {
            world.consume_actuators_and_integrate_inner(Duration::from_millis(1));
        }
        let obj = world.scene.object(ObjectId(1)).unwrap();
        assert!(
            matches!(
                obj.state,
                ObjectState::Settled {
                    on: SupportId::Unknown
                }
            ),
            "expected Settled with SupportId::Unknown, got {:?}",
            obj.state
        );
    }

    #[cfg(feature = "physics-rapier")]
    #[test]
    fn object_remains_free_when_moving() {
        // Sphere in mid-air (no fixture below to settle on) — it falls
        // and stays Free for the duration of a short step run.
        use nalgebra::Isometry3;
        use rtf_sim::object::{Object, ObjectId, ObjectState};
        use rtf_sim::shape::Shape;

        let mut scene = Scene::new(0);
        scene.insert_object(Object::new(
            ObjectId(1),
            Isometry3::translation(0.0, 0.0, 1.0),
            Shape::Sphere { radius: 0.05 },
            0.1,
            true,
        ));
        let mut world = ArmWorld::new(scene, simple_spec(), true);

        // 50 ms of free-fall — well above settled-velocity threshold.
        for _ in 0..50 {
            world.consume_actuators_and_integrate_inner(Duration::from_millis(1));
        }
        let obj = world.scene.object(ObjectId(1)).unwrap();
        assert!(
            matches!(obj.state, ObjectState::Free),
            "expected Free during fall, got {:?}",
            obj.state
        );
    }

    #[cfg(feature = "physics-rapier")]
    #[test]
    fn grasped_object_state_unchanged_by_settled_logic() {
        use nalgebra::Isometry3;
        use rtf_sim::object::{ArmRef, Object, ObjectId, ObjectState};
        use rtf_sim::shape::Shape;

        let mut scene = Scene::new(0);
        scene.insert_object(Object::new(
            ObjectId(1),
            Isometry3::translation(0.0, 0.0, 0.5),
            Shape::Sphere { radius: 0.05 },
            0.1,
            true,
        ));
        let mut world = ArmWorld::new(scene, simple_spec(), true);
        // Force-set Grasped (bypass apply_gripper_command for test
        // simplicity) — Step 1.8 will wire the proper kinematic-flip.
        let obj = world.scene.object_mut(ObjectId(1)).unwrap();
        obj.state = ObjectState::Grasped { by: ArmRef(0) };

        for _ in 0..100 {
            world.consume_actuators_and_integrate_inner(Duration::from_millis(1));
        }
        let obj = world.scene.object(ObjectId(1)).unwrap();
        assert!(
            matches!(obj.state, ObjectState::Grasped { .. }),
            "Grasped state should survive the Settled-derivation pass; got {:?}",
            obj.state
        );
    }

    #[cfg(feature = "physics-rapier")]
    #[test]
    fn armworld_step_keeps_settled_object_on_table() {
        // build_pick_and_place_world has the block pre-Settled on a
        // table fixture. After 100 ms of stepping, the block should
        // remain near its starting xy/z (small jitter is fine).
        use crate::test_helpers::{build_pick_and_place_world, BLOCK_OBJECT_ID};
        let mut world = build_pick_and_place_world();
        let pose0 = world.scene.object(BLOCK_OBJECT_ID).unwrap().pose;
        for _ in 0..100 {
            world.consume_actuators_and_integrate_inner(Duration::from_millis(1));
        }
        let pose1 = world.scene.object(BLOCK_OBJECT_ID).unwrap().pose;
        let drift = (pose1.translation.vector - pose0.translation.vector).norm();
        assert!(
            drift < 0.02,
            "block drifted {drift:.4} m in 100 ms (expected < 0.02)"
        );
    }

    #[cfg(feature = "physics-rapier")]
    #[test]
    fn armworld_step_pushes_object_when_arm_collides() {
        // Place a small free Sphere directly in the path of an arm
        // link's swept volume. With joint 0 rotating about z-axis, the
        // arm-link capsules sweep an arc in xy. moving_ee_spec has two
        // 0.5 m x-axis link offsets, so link 1's capsule midpoint is
        // at (0.75, 0, 0) at q=0 with radius 0.02 m + capsule end caps.
        // We place the sphere at (0.5, 0.05, 0) so that as joint 0
        // rotates from 0 toward +1 rad, link 0's collider sweeps right
        // through it.
        use nalgebra::{Isometry3, Vector3};
        use rtf_sim::object::{Object, ObjectId};
        use rtf_sim::shape::Shape;

        let mut scene = Scene::new(0);
        let block = ObjectId(1);
        scene.insert_object(Object::new(
            block,
            Isometry3::translation(0.5, 0.05, 0.0),
            Shape::Sphere { radius: 0.05 },
            0.1,
            true,
        ));
        let mut world = ArmWorld::new(scene, moving_ee_spec(), /* gravity */ false);

        let pose0 = world.scene.object(block).unwrap().pose;
        let tx = world.attach_joint_velocity_actuator(JointId(0));
        tx.send(JointVelocityCommand {
            joint: JointId(0),
            q_dot_target: 1.5,
        });
        // 500 ms at 1.5 rad/s → joint sweeps 0.75 rad, taking link 0
        // through the sphere's xy.
        for _ in 0..500 {
            world.consume_actuators_and_integrate_inner(Duration::from_millis(1));
        }
        let pose1 = world.scene.object(block).unwrap().pose;
        // Use squared distance via direct multiplication; project lint
        // disallows `f32::powi` for cross-toolchain determinism.
        let dx = pose1.translation.x - pose0.translation.x;
        let dy = pose1.translation.y - pose0.translation.y;
        let drift_xy = (dx * dx + dy * dy).sqrt();
        assert!(
            drift_xy > 0.01,
            "expected the sweeping arm link to push the sphere; xy drift was {drift_xy:.4} m"
        );
        let _ = Vector3::<f32>::zeros(); // silence unused-import warning
    }

    #[cfg(feature = "physics-rapier")]
    #[test]
    fn armworld_physics_includes_inserted_fixtures_and_objects() {
        use nalgebra::{Isometry3, Vector3};
        use rtf_sim::fixture::Fixture;
        use rtf_sim::object::{Object, ObjectId};
        use rtf_sim::shape::Shape;

        let mut scene = Scene::new(0);
        scene.add_fixture(Fixture {
            id: 0,
            pose: Isometry3::translation(0.0, 0.0, 0.0),
            shape: Shape::Aabb {
                half_extents: Vector3::new(1.0, 1.0, 0.05),
            },
            is_support: true,
        });
        scene.insert_object(Object::new(
            ObjectId(1),
            Isometry3::translation(0.0, 0.0, 0.5),
            Shape::Sphere { radius: 0.05 },
            0.1,
            true,
        ));

        let world = ArmWorld::new(scene, simple_spec(), true);
        // 1 fixture + 1 object + 2 arm-link bodies = 4 total.
        assert_eq!(world.physics().body_count(), 4);
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
        tx.send(JointVelocityCommand {
            joint: JointId(0),
            q_dot_target: 1.0,
        });
        world.consume_actuators_and_integrate_inner(Duration::from_millis(10));
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
            block_id,
            ee,
            Shape::Sphere { radius: 0.01 },
            0.1,
            /* graspable */ true,
        ));

        let g_tx = world.attach_gripper_actuator();
        g_tx.send(GripperCommand { close: true });
        world.consume_actuators_and_integrate_inner(Duration::from_millis(1));

        assert_eq!(world.arm.state.grasped, Some(block_id));
        assert!(matches!(
            world.scene.object(block_id).unwrap().state,
            ObjectState::Grasped { .. }
        ));
    }

    #[test]
    fn arm_world_implements_runnable_world() {
        use rtf_sim::runnable_world::RunnableWorld;
        let world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        let snap = world.snapshot();
        assert_eq!(snap.t, RunnableWorld::time(&world));
        // 2 capsules (one per link) + 1 box (gripper) for a 2-joint arm = 3.
        assert!(snap.items.len() >= 3);
    }

    #[test]
    fn grasped_object_pose_tracks_ee() {
        use rtf_core::time::Duration;
        use rtf_sim::object::{Object, ObjectId};
        use rtf_sim::shape::Shape;

        let mut world = ArmWorld::new(Scene::new(0), moving_ee_spec(), true);
        let block_id = ObjectId(42);
        world.scene.insert_object(Object::new(
            block_id,
            world.ee_pose(),
            Shape::Sphere { radius: 0.01 },
            0.1,
            true,
        ));

        let v_tx = world.attach_joint_velocity_actuator(JointId(0));
        let g_tx = world.attach_gripper_actuator();
        g_tx.send(GripperCommand { close: true });
        world.consume_actuators_and_integrate_inner(Duration::from_millis(1));
        let pose_before = world.scene.object(block_id).unwrap().pose;

        v_tx.send(JointVelocityCommand {
            joint: JointId(0),
            q_dot_target: 1.0,
        });
        world.consume_actuators_and_integrate_inner(Duration::from_millis(50));
        let pose_after = world.scene.object(block_id).unwrap().pose;

        assert_ne!(
            pose_before.translation.vector,
            pose_after.translation.vector
        );
        let ee = world.ee_pose();
        assert!((pose_after.translation.vector - ee.translation.vector).norm() < 1e-4);
    }

    #[test]
    fn pressure_sensor_zero_when_no_object_nearby() {
        // Empty scene (no ground, no fixtures, no objects); pressure sensor
        // sees nothing and reports 0.
        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        let rx = world.attach_pressure_sensor(RateHz::new(1000), 0.03);
        world.publish_sensors_for_dt(Duration::from_millis(1));
        let r = rx.latest().expect("pressure published");
        assert!(
            r.pressure.abs() < 1e-6,
            "expected 0 pressure, got {}",
            r.pressure
        );
    }

    #[test]
    fn pressure_sensor_falls_off_linearly_with_distance() {
        use nalgebra::{Isometry3, Vector3};
        use rtf_sim::object::{Object, ObjectId, ObjectState};
        use rtf_sim::shape::Shape;

        // EE for simple_spec sits at world origin (0, 0, 0.2).
        // Place a thin Aabb directly below the EE so the EE-to-surface
        // distance is exactly d. Top of box sits at (z = ee.z - d).
        let eps = 0.03_f32;
        for d in [0.0_f32, 0.005, 0.01, 0.02, 0.029] {
            let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
            let ee = world.ee_pose().translation.vector;
            let top_z = ee.z - d;
            let half_z = 0.05;
            let center_z = top_z - half_z;
            world.scene.insert_object(Object {
                id: ObjectId(1),
                pose: Isometry3::translation(ee.x, ee.y, center_z),
                shape: Shape::Aabb {
                    half_extents: Vector3::new(0.05, 0.05, half_z),
                },
                mass: 0.1,
                graspable: false,
                state: ObjectState::Free,
                lin_vel: Vector3::zeros(),
            });
            let rx = world.attach_pressure_sensor(RateHz::new(1000), eps);
            world.publish_sensors_for_dt(Duration::from_millis(1));
            let r = rx.latest().expect("pressure published");
            let expected = (eps - d) / eps;
            assert!(
                (r.pressure - expected).abs() < 1e-4,
                "d={d}: expected {expected}, got {}",
                r.pressure
            );
        }
    }

    #[test]
    fn pressure_sensor_saturates_inside_object() {
        use nalgebra::{Isometry3, Vector3};
        use rtf_sim::object::{Object, ObjectId, ObjectState};
        use rtf_sim::shape::Shape;

        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        let ee = world.ee_pose().translation.vector;
        // Box centered at the EE — the EE is well inside it.
        world.scene.insert_object(Object {
            id: ObjectId(1),
            pose: Isometry3::translation(ee.x, ee.y, ee.z),
            shape: Shape::Aabb {
                half_extents: Vector3::new(0.1, 0.1, 0.1),
            },
            mass: 0.1,
            graspable: false,
            state: ObjectState::Free,
            lin_vel: Vector3::zeros(),
        });
        let rx = world.attach_pressure_sensor(RateHz::new(1000), 0.03);
        world.publish_sensors_for_dt(Duration::from_millis(1));
        let r = rx.latest().expect("pressure published");
        // d = 0 inside the box → (eps - 0)/eps = 1.0.
        assert!(
            (r.pressure - 1.0).abs() < 1e-6,
            "expected 1.0, got {}",
            r.pressure
        );
    }

    #[test]
    fn pressure_sensor_publishes_at_configured_rate() {
        // Attach at 100 Hz, pump 50 ms of sim time → expect 5 ticks.
        // We don't have an easy way to count sends through a single-slot
        // port, so step in 1ms increments and count distinct sample
        // timestamps observed via `latest()`.
        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), true);
        let rx = world.attach_pressure_sensor(RateHz::new(100), 0.03);
        let mut seen = std::collections::BTreeSet::new();
        for _ in 0..50 {
            world.consume_actuators_and_integrate_inner(Duration::from_millis(1));
            world.publish_sensors_for_dt(Duration::from_millis(1));
            if let Some(r) = rx.latest() {
                seen.insert(r.sampled_at.as_nanos());
            }
        }
        assert_eq!(
            seen.len(),
            5,
            "expected 5 distinct publish times in 50ms at 100Hz, got {}: {:?}",
            seen.len(),
            seen
        );
    }

    #[test]
    fn pressure_sensor_does_not_fire_at_sweep_altitude() {
        // Find-grasp-place sanity check: a table at z in [0.45, 0.50] (top at
        // 0.50) plus an EE at z = 0.57 (sweep_z) should be 7 cm above the
        // table top — far outside eps=0.03, so the table fixture must not
        // trigger any pressure.
        use nalgebra::{Isometry3, Vector3};
        use rtf_sim::fixture::Fixture;
        use rtf_sim::shape::Shape;

        // Build a simple 1-joint arm with a single 0.57m z-link so the EE
        // sits at z=0.57 (matches the find-grasp-place sweep altitude).
        let spec = {
            use crate::spec::{ArmSpec, GripperSpec, JointSpec};
            use core::f32::consts::PI;
            ArmSpec {
                joints: vec![JointSpec::Revolute {
                    axis: Vector3::z_axis(),
                    limits: (-PI, PI),
                }],
                link_offsets: vec![Isometry3::translation(0.0, 0.0, 0.57)],
                gripper: GripperSpec {
                    proximity_threshold: 0.05,
                    max_grasp_size: 0.1,
                },
            }
        };
        let mut scene = Scene::new(0);
        scene.add_fixture(Fixture {
            id: 0,
            pose: Isometry3::translation(0.0, 0.0, 0.475),
            shape: Shape::Aabb {
                half_extents: Vector3::new(0.4, 0.4, 0.025),
            },
            is_support: true,
        });
        let mut world = ArmWorld::new(scene, spec, /* gravity */ false);
        let rx = world.attach_pressure_sensor(RateHz::new(1000), 0.03);
        world.publish_sensors_for_dt(Duration::from_millis(1));
        let r = rx.latest().expect("pressure published");
        assert!(
            r.pressure.abs() < 1e-6,
            "table top should be 7cm below sweep_z (well outside eps=0.03), got pressure={}",
            r.pressure
        );
    }

    /// Kinematic-gravity-fall settled detection: only meaningful in the
    /// `not(physics-rapier)` path. With Rapier on, Settled-derivation
    /// from velocity arrives in Step 1.7 — until then the test would
    /// incorrectly observe `Free` (initial state) post-step. Step 1.13
    /// retires this test entirely once the kinematic path is removed.
    #[cfg(not(feature = "physics-rapier"))]
    #[test]
    fn arm_world_with_gravity_lets_object_fall_onto_table() {
        use nalgebra::{Isometry3, Vector3};
        use rtf_core::time::Duration;
        use rtf_sim::fixture::Fixture;
        use rtf_sim::object::{Object, ObjectId, ObjectState};
        use rtf_sim::shape::Shape;

        let mut scene = rtf_sim::scene::Scene::with_ground(0);
        scene.add_fixture(Fixture {
            id: 0,
            pose: Isometry3::translation(0.0, 0.0, 0.5),
            shape: Shape::Aabb {
                half_extents: Vector3::new(0.5, 0.5, 0.01),
            },
            is_support: true,
        });
        let block = ObjectId(1);
        scene.insert_object(Object::new(
            block,
            Isometry3::translation(0.0, 0.0, 1.0),
            Shape::Sphere { radius: 0.05 },
            0.1,
            true,
        ));
        let mut world = ArmWorld::new(scene, simple_spec(), /* gravity */ true);
        for _ in 0..2000 {
            world.consume_actuators_and_integrate_inner(Duration::from_millis(1));
        }
        assert!(matches!(
            world.scene.object(block).unwrap().state,
            ObjectState::Settled { .. }
        ));
    }

    /// Build a minimal Object template for spawn tests; id is overwritten
    /// at spawn time by the world.
    fn block_template() -> Object {
        use nalgebra::{Isometry3, Vector3};
        use rtf_sim::object::{Object, ObjectId, ObjectState};
        use rtf_sim::shape::Shape;
        Object {
            id: ObjectId(0),
            pose: Isometry3::translation(0.5, 0.0, 1.0),
            shape: Shape::Aabb {
                half_extents: Vector3::new(0.025, 0.025, 0.025),
            },
            mass: 0.1,
            graspable: true,
            state: ObjectState::Free,
            lin_vel: Vector3::zeros(),
        }
    }

    #[test]
    fn schedule_spawn_inserts_at_due_time() {
        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), false);
        world.schedule_spawn(Time::from_millis(10), block_template());
        // Step 5 ms — not yet due, no insert.
        world.consume_actuators_and_integrate_inner(Duration::from_millis(5));
        assert_eq!(world.scene.objects().count(), 0);
        // Step 5 more ms — at exactly 10 ms now, due, insert fires.
        world.consume_actuators_and_integrate_inner(Duration::from_millis(5));
        assert_eq!(world.scene.objects().count(), 1);
    }

    #[test]
    fn schedule_spawn_assigns_unique_ids() {
        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), false);
        for _ in 0..3 {
            world.schedule_spawn(Time::from_millis(1), block_template());
        }
        world.consume_actuators_and_integrate_inner(Duration::from_millis(2));
        let ids: std::collections::BTreeSet<_> = world.scene.objects().map(|(id, _)| *id).collect();
        assert_eq!(ids.len(), 3, "expected 3 distinct ids, got {ids:?}");
    }

    #[test]
    fn schedule_spawn_processes_in_time_order() {
        // Schedule three spawns out of order; verify they fire in time order
        // and that each batch up-to-now is fully consumed before the next
        // step. Use distinct pose.x values per template so we can tell which
        // spawn produced each inserted object after the fact.
        use nalgebra::Translation3;
        let mut world = ArmWorld::new(Scene::new(0), simple_spec(), false);
        let mut t30 = block_template();
        t30.pose.translation = Translation3::new(0.30, 0.0, 1.0);
        let mut t10 = block_template();
        t10.pose.translation = Translation3::new(0.10, 0.0, 1.0);
        let mut t20 = block_template();
        t20.pose.translation = Translation3::new(0.20, 0.0, 1.0);
        world.schedule_spawn(Time::from_millis(30), t30);
        world.schedule_spawn(Time::from_millis(10), t10);
        world.schedule_spawn(Time::from_millis(20), t20);
        // Step to t=10ms → only the t=10 spawn fires.
        world.consume_actuators_and_integrate_inner(Duration::from_millis(10));
        let xs: Vec<f32> = world
            .scene
            .objects()
            .map(|(_, o)| o.pose.translation.x)
            .collect();
        assert_eq!(xs, vec![0.10]);
        // Step to t=20ms → t=20 fires.
        world.consume_actuators_and_integrate_inner(Duration::from_millis(10));
        let mut xs: Vec<f32> = world
            .scene
            .objects()
            .map(|(_, o)| o.pose.translation.x)
            .collect();
        xs.sort_by(f32::total_cmp);
        assert_eq!(xs, vec![0.10, 0.20]);
        // Step to t=30ms → t=30 fires.
        world.consume_actuators_and_integrate_inner(Duration::from_millis(10));
        let mut xs: Vec<f32> = world
            .scene
            .objects()
            .map(|(_, o)| o.pose.translation.x)
            .collect();
        xs.sort_by(f32::total_cmp);
        assert_eq!(xs, vec![0.10, 0.20, 0.30]);
    }
}

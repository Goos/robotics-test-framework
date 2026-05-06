//! Test helpers — gated like `examples_` (cfg(test) or feature = "examples").
//! Builders for the canonical Phase 5 happy-path arm setup so end-to-end
//! tests don't have to repeat boilerplate.

use nalgebra::{Isometry3, Vector3};
use rtf_core::port::{PortReader, PortRx, PortTx};
use rtf_sim::{
    fixture::Fixture,
    object::{Object, ObjectId, ObjectState, SupportId},
    scene::Scene,
    shape::Shape,
};

use crate::{
    ports::{GripperCommand, JointEncoderReading, JointId, JointVelocityCommand},
    spec::{ArmSpec, GripperSpec, JointSpec},
    world::{ArmWorld, RateHz},
};

/// Conventional ids for the pick-and-place scenario built by
/// [`build_pick_and_place_world`]. Fixture id 0 is reserved for the table;
/// the ground plane uses `u32::MAX` (per `Scene::with_ground`).
pub const BLOCK_OBJECT_ID: ObjectId = ObjectId(1);
pub const BIN_FIXTURE_ID: u32 = 2;

/// Build an `ArmWorld` with `n_joints` z-axis revolute joints, 0.2 m **x-axis**
/// link offsets, default gripper (proximity 0.02, max_grasp 0.05), gravity
/// OFF (Phase 5 doesn't model gravity). Seeded `Scene::new(0)` for
/// determinism.
///
/// Link offsets are along the x-axis on purpose: with z-axis Rz joints and
/// z-axis link offsets, the EE pose is invariant under any joint motion
/// (Rz fixes z-axis points), which makes ReachPose tests degenerate — the
/// goal sees zero distance at t=0 and short-circuits before the controller
/// even runs. Same trap as Step 3.11e's `moving_ee_spec`. Don't "fix" back.
pub fn build_simple_arm_world(n_joints: usize) -> ArmWorld {
    use core::f32::consts::PI;
    let spec = ArmSpec {
        joints: vec![JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-PI, PI) }; n_joints],
        link_offsets: vec![Isometry3::translation(0.2, 0.0, 0.0); n_joints],
        gripper: GripperSpec { proximity_threshold: 0.02, max_grasp_size: 0.05 },
    };
    ArmWorld::new(Scene::new(0), spec, /* gravity */ false)
}

/// Build the canonical pick-and-place world used by Phase 7 sample tasks:
/// ground + a table fixture (id 0) + a bin fixture ([`BIN_FIXTURE_ID`]) + a
/// graspable block ([`BLOCK_OBJECT_ID`]) pre-Settled on the table top, plus
/// a 3-joint planar arm reaching from the origin. Gravity is ON.
pub fn build_pick_and_place_world() -> ArmWorld {
    use core::f32::consts::PI;
    let mut scene = Scene::with_ground(0);

    scene.add_fixture(Fixture {
        id: 0,
        pose: Isometry3::translation(0.5, 0.0, 0.475),
        shape: Shape::Aabb { half_extents: Vector3::new(0.4, 0.4, 0.025) },
        is_support: true,
    });

    scene.add_fixture(Fixture {
        id: BIN_FIXTURE_ID,
        pose: Isometry3::translation(0.0, 0.5, 0.55),
        shape: Shape::Aabb { half_extents: Vector3::new(0.1, 0.1, 0.05) },
        is_support: true,
    });

    scene.insert_object(Object {
        id: BLOCK_OBJECT_ID,
        pose: Isometry3::translation(0.5, 0.0, 0.525),
        shape: Shape::Aabb { half_extents: Vector3::new(0.025, 0.025, 0.025) },
        mass: 0.1,
        graspable: true,
        state: ObjectState::Settled { on: SupportId::Fixture(0) },
        lin_vel: Vector3::zeros(),
    });

    let spec = ArmSpec {
        joints: vec![JointSpec::Revolute { axis: Vector3::z_axis(), limits: (-PI, PI) }; 3],
        link_offsets: vec![Isometry3::translation(0.2, 0.0, 0.0); 3],
        gripper: GripperSpec { proximity_threshold: 0.1, max_grasp_size: 0.1 },
    };

    ArmWorld::new(scene, spec, /* gravity */ true)
}

pub fn block_id(_world: &ArmWorld) -> ObjectId { BLOCK_OBJECT_ID }
pub fn bin_id(_world: &ArmWorld) -> u32 { BIN_FIXTURE_ID }

/// Bundle of every port the canonical PD + gripper controller needs:
/// per-joint encoder receivers + velocity senders, plus a single gripper
/// command sender. Generic over `R` so future fault-wrapped tests can swap
/// `PortRx<JointEncoderReading>` for a wrapped reader.
pub struct StandardArmPorts<R = PortRx<JointEncoderReading>>
where
    R: PortReader<JointEncoderReading>,
{
    pub encoder_rxs: Vec<R>,
    pub velocity_txs: Vec<PortTx<JointVelocityCommand>>,
    pub gripper_tx: PortTx<GripperCommand>,
}

impl ArmWorld {
    /// Attach one encoder (1 kHz) + one velocity actuator per joint, plus a
    /// single gripper actuator. Returns the matching receivers/senders so the
    /// test harness can hand them to a controller.
    pub fn attach_standard_arm_ports(&mut self) -> StandardArmPorts {
        let n = self.arm.spec.joints.len();
        let encoder_rxs = (0..n)
            .map(|i| self.attach_joint_encoder_sensor(JointId(i as u32), RateHz::new(1000)))
            .collect();
        let velocity_txs = (0..n)
            .map(|i| self.attach_joint_velocity_actuator(JointId(i as u32)))
            .collect();
        let gripper_tx = self.attach_gripper_actuator();
        StandardArmPorts { encoder_rxs, velocity_txs, gripper_tx }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn build_simple_arm_world_constructs_with_n_joints() {
        let world = build_simple_arm_world(3);
        assert_eq!(world.arm.spec.joints.len(), 3);
        assert!(!world.gravity_enabled);
    }

    #[test]
    fn attach_standard_arm_ports_returns_n_encoder_and_velocity_ports() {
        let mut world = build_simple_arm_world(2);
        let ports = world.attach_standard_arm_ports();
        assert_eq!(ports.encoder_rxs.len(), 2);
        assert_eq!(ports.velocity_txs.len(), 2);
    }

    #[test]
    fn pick_and_place_world_has_table_bin_and_block() {
        let world = build_pick_and_place_world();
        assert_eq!(world.scene.fixtures().count(), 3);
        assert!(world.scene.object(BLOCK_OBJECT_ID).is_some());
        assert_eq!(block_id(&world), BLOCK_OBJECT_ID);
        assert_eq!(bin_id(&world), BIN_FIXTURE_ID);
        assert!(world.gravity_enabled);
    }
}

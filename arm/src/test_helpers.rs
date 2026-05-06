//! Test helpers — gated like `examples_` (cfg(test) or feature = "examples").
//! Builders for the canonical Phase 5 happy-path arm setup so end-to-end
//! tests don't have to repeat boilerplate.

use nalgebra::{Isometry3, Vector3};
use rtf_core::port::{PortReader, PortRx, PortTx};
use rtf_sim::scene::Scene;

use crate::{
    ports::{GripperCommand, JointEncoderReading, JointId, JointVelocityCommand},
    spec::{ArmSpec, GripperSpec, JointSpec},
    world::{ArmWorld, RateHz},
};

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
}

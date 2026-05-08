//! rtf_arm — robotic arm primitives + reference controllers.
//!
//! Provides a Z-Y-Y-Y kinematic-arm + dynamic-objects simulation with
//! joint encoders, EE-mounted pressure + joint-torque sensors,
//! per-joint velocity actuators, and a continuous gripper actuator.
//! Backed by Rapier 3D physics (rtf_sim::physics) — see
//! `docs/plans/2026-05-07-rapier-integration-design.md` for the full
//! integration design.
//!
//! ## Grasp model (Phase 3+)
//!
//! Grasp is **joint-attached with friction-flavored slip detection**
//! (design §11.3): on the gripper-close edge, when both fingers are in
//! contact with a graspable object, a Rapier `FixedJoint` is created
//! between the EE arm-link kinematic body and the object's dynamic
//! body. The object stays Dynamic — gravity, contact, and fixture
//! interactions all continue to apply. Per tick, the joint's
//! accumulated impulse is checked against `SLIP_IMPULSE_THRESHOLD`
//! (5.0); if exceeded, the joint releases and the held object
//! transitions Grasped → Free, modeling a "this much load would have
//! slipped" event. On gripper-open, the joint is removed explicitly.
//!
//! This is the same pattern Isaac Sim and NVIDIA PhysX use (a
//! soft-weld dressed in physics-shaped API). It pivoted from a
//! pure-friction grasp attempt (Step 3.4) that ran into Rapier's
//! kinematic↔dynamic friction-coupling limitations under fast yaw —
//! see design §11.3 for the full rationale.
//!
//! See `arm/examples/pick_place.rs` for a full happy-path scenario
//! and `arm/examples/grasp_robustness.rs` for the slow-vs-fast slip
//! demonstration.

pub mod arm;
pub mod fk;
pub mod goals;
pub mod ik;
pub mod pd_controller;
pub mod ports;
pub mod spec;
pub mod state;
#[cfg(any(test, feature = "examples"))]
pub mod test_helpers;
pub mod world;

pub use arm::Arm;
pub use fk::{forward_kinematics, joint_transform};
pub use pd_controller::PdJointController;
pub use ports::{
    ArmContactReading, EePoseReading, GripperCommand, JointEncoderReading, JointId,
    JointTorqueReading, JointVelocityCommand, PressureReading,
};
pub use spec::{ArmSpec, GripperSpec, JointSpec};
pub use state::ArmState;
pub use world::{ArmWorld, RateHz};

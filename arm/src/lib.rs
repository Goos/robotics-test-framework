//! rtf_arm — populated starting in Phase 1.

pub mod fk;
pub mod ports;
pub mod spec;
pub mod state;

pub use fk::{forward_kinematics, joint_transform};
pub use ports::{
    EePoseReading, GripperCommand, JointEncoderReading, JointId, JointVelocityCommand,
};
pub use spec::{ArmSpec, GripperSpec, JointSpec};
pub use state::ArmState;

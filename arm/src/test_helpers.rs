//! Test helpers — gated by `cfg(test)` or `feature = "examples"`. Builders
//! for the canonical Phase 5 happy-path arm setup so example scenarios and
//! integration tests don't have to repeat boilerplate.

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

/// Search region (find-grasp-place design §3.3) — Aabb on the table top
/// where the seeded block is placed by [`build_search_world`]. Selected so
/// every point is reachable by the standard Z-Y-Y arm and disjoint from the
/// bin xy.
pub const SEARCH_REGION_X: (f32, f32) = (0.40, 0.70);
pub const SEARCH_REGION_Y: (f32, f32) = (-0.25, 0.25);
/// Block top z (table top + block half-height). Convenience for callers that
/// pre-compute IK targets at grasp altitude.
pub const SEARCH_REGION_Z: f32 = 0.55;
pub const BLOCK_HALF_HEIGHT: f32 = 0.025;

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
        joints: vec![
            JointSpec::Revolute {
                axis: Vector3::z_axis(),
                limits: (-PI, PI)
            };
            n_joints
        ],
        link_offsets: vec![Isometry3::translation(0.2, 0.0, 0.0); n_joints],
        gripper: GripperSpec {
            proximity_threshold: 0.02,
            max_grasp_size: 0.05,
        },
    };
    ArmWorld::new(Scene::new(0), spec, /* gravity */ false)
}

/// Build the canonical pick-and-place world used by Phase 7 sample tasks:
/// ground + a table fixture (id 0) + a bin fixture ([`BIN_FIXTURE_ID`]) + a
/// graspable block ([`BLOCK_OBJECT_ID`]) pre-Settled on the table top, plus
/// a Z-Y-Y arm with a 0.8 m pedestal and two 0.4 m links (total reach 0.8 m
/// from the shoulder). Gravity is ON. Block xy=(0.6, 0) and bin xy=(0, 0.6)
/// are both within IK-reach of the shoulder at (0, 0, 0.8).
pub fn build_pick_and_place_world() -> ArmWorld {
    use core::f32::consts::PI;
    let mut scene = Scene::with_ground(0);

    scene.add_fixture(Fixture {
        id: 0,
        pose: Isometry3::translation(0.5, 0.0, 0.475),
        shape: Shape::Aabb {
            half_extents: Vector3::new(0.4, 0.4, 0.025),
        },
        is_support: true,
        color: rtf_sim::palette::TABLE_GRAY,
    });

    let bin_pose = Isometry3::translation(0.0, 0.6, 0.60); // rim TOP per design §7.3
    let bin_parts = rtf_sim::fixture::bin_decomposition(
        bin_pose,
        nalgebra::Vector2::new(0.10, 0.10),
        /* inner_depth */ 0.08,
        /* wall_thickness */ 0.01,
    );
    for (i, (pose, he)) in bin_parts.iter().enumerate() {
        scene.add_fixture(Fixture {
            id: BIN_FIXTURE_ID + i as u32,
            pose: *pose,
            shape: Shape::Aabb { half_extents: *he },
            is_support: i == 0, // only the floor is a settled-on support
            color: rtf_sim::palette::BIN_GRAY,
        });
    }

    scene.insert_object(Object {
        id: BLOCK_OBJECT_ID,
        pose: Isometry3::translation(0.6, 0.0, 0.525),
        shape: Shape::Aabb {
            half_extents: Vector3::new(0.025, 0.025, 0.025),
        },
        mass: 0.1,
        graspable: true,
        state: ObjectState::Settled {
            on: SupportId::Fixture(0),
        },
        friction: 2.0,
        lin_vel: Vector3::zeros(),
    });

    // Z-Y-Y-Y arm: J0 yaws the chain in xy; J1+J2 are pitch joints that
    // descend/ascend the EE in the radial-z plane; J3 (Phase 3.4.5b) is a
    // wrist pitch joint that lets the controller align EE +x with world
    // -z (fingers pointing straight down) regardless of the J1+J2 pose.
    // Pedestal lifts the shoulder to z=0.8; two equal 0.4 m links give a
    // 0.8 m reach circle around the shoulder; the wrist link adds 0.05 m
    // of forward EE offset. Block (~0.66 m from shoulder) and bin
    // (~0.65 m from shoulder) are both well within reach.
    let spec = ArmSpec {
        joints: vec![
            JointSpec::Revolute {
                axis: Vector3::z_axis(),
                limits: (-PI, PI),
            }, // J0 yaw
            JointSpec::Revolute {
                axis: Vector3::y_axis(),
                limits: (-PI, PI),
            }, // J1 shoulder pitch
            JointSpec::Revolute {
                axis: Vector3::y_axis(),
                limits: (-PI, PI),
            }, // J2 elbow pitch
            JointSpec::Revolute {
                axis: Vector3::y_axis(),
                limits: (-PI, PI),
            }, // J3 wrist pitch (Phase 3.4.5b)
        ],
        link_offsets: vec![
            Isometry3::translation(0.0, 0.0, 0.8), // pedestal — invariant under J0 yaw
            Isometry3::translation(0.4, 0.0, 0.0), // upper arm
            Isometry3::translation(0.4, 0.0, 0.0), // forearm
            Isometry3::translation(0.05, 0.0, 0.0), // wrist (Phase 3.4.5b)
        ],
        gripper: GripperSpec {
            proximity_threshold: 0.05,
            max_grasp_size: 0.1,
        },
    };

    ArmWorld::new(scene, spec, /* gravity */ true)
}

pub fn block_id(_world: &ArmWorld) -> ObjectId {
    BLOCK_OBJECT_ID
}
/// Returns the bin's **floor** fixture id. Post-Commit 4 the bin is 5
/// fixtures (floor + 4 walls) at ids `BIN_FIXTURE_ID..BIN_FIXTURE_ID+5`;
/// only the floor is `is_support: true`, so this is the id used as the
/// place target by pick-and-place controllers.
pub fn bin_id(_world: &ArmWorld) -> u32 {
    BIN_FIXTURE_ID
}

/// Build the find-grasp-place world: same arm + table + bin as
/// [`build_pick_and_place_world`], but the block xy is sampled uniformly
/// from the search region (see [`SEARCH_REGION_X`] / [`SEARCH_REGION_Y`])
/// using a `PcgNoiseSource` seeded with `seed`. Same seed → byte-identical
/// world; different seeds → different block xy.
///
/// The controller is given the region bounds at construction; it does NOT
/// know the actual block xy.
pub fn build_search_world(seed: u64) -> ArmWorld {
    use core::f32::consts::PI;
    use rtf_core::noise_source::NoiseSource;
    use rtf_sim::faults::PcgNoiseSource;

    let mut src = PcgNoiseSource::from_seed(seed);
    let x = SEARCH_REGION_X.0 + src.uniform_unit() * (SEARCH_REGION_X.1 - SEARCH_REGION_X.0);
    let y = SEARCH_REGION_Y.0 + src.uniform_unit() * (SEARCH_REGION_Y.1 - SEARCH_REGION_Y.0);
    // Table top sits at z = 0.475 + 0.025 = 0.5; block sits half its
    // height above that.
    let block_z = 0.5 + BLOCK_HALF_HEIGHT;

    let mut scene = Scene::with_ground(0);

    scene.add_fixture(Fixture {
        id: 0,
        pose: Isometry3::translation(0.5, 0.0, 0.475),
        shape: Shape::Aabb {
            half_extents: Vector3::new(0.4, 0.4, 0.025),
        },
        is_support: true,
        color: rtf_sim::palette::TABLE_GRAY,
    });

    let bin_pose = Isometry3::translation(0.0, 0.6, 0.60); // rim TOP per design §7.3
    let bin_parts = rtf_sim::fixture::bin_decomposition(
        bin_pose,
        nalgebra::Vector2::new(0.10, 0.10),
        /* inner_depth */ 0.08,
        /* wall_thickness */ 0.01,
    );
    for (i, (pose, he)) in bin_parts.iter().enumerate() {
        scene.add_fixture(Fixture {
            id: BIN_FIXTURE_ID + i as u32,
            pose: *pose,
            shape: Shape::Aabb { half_extents: *he },
            is_support: i == 0,
            color: rtf_sim::palette::BIN_GRAY,
        });
    }

    scene.insert_object(Object {
        id: BLOCK_OBJECT_ID,
        pose: Isometry3::translation(x, y, block_z),
        shape: Shape::Aabb {
            half_extents: Vector3::new(0.025, 0.025, BLOCK_HALF_HEIGHT),
        },
        mass: 0.1,
        graspable: true,
        state: ObjectState::Settled {
            on: SupportId::Fixture(0),
        },
        friction: 2.0,
        lin_vel: Vector3::zeros(),
    });

    // Same Z-Y-Y-Y geometry as build_pick_and_place_world (Phase 3.4.5b
    // adds the wrist joint).
    let spec = ArmSpec {
        joints: vec![
            JointSpec::Revolute {
                axis: Vector3::z_axis(),
                limits: (-PI, PI),
            },
            JointSpec::Revolute {
                axis: Vector3::y_axis(),
                limits: (-PI, PI),
            },
            JointSpec::Revolute {
                axis: Vector3::y_axis(),
                limits: (-PI, PI),
            },
            JointSpec::Revolute {
                axis: Vector3::y_axis(),
                limits: (-PI, PI),
            }, // J3 wrist pitch (Phase 3.4.5b)
        ],
        link_offsets: vec![
            Isometry3::translation(0.0, 0.0, 0.8),
            Isometry3::translation(0.4, 0.0, 0.0),
            Isometry3::translation(0.4, 0.0, 0.0),
            Isometry3::translation(0.05, 0.0, 0.0), // wrist (Phase 3.4.5b)
        ],
        gripper: GripperSpec {
            // Wider than the pick-and-place threshold (0.05) so an arm-link-
            // pushed block remains within grasp range when the EE descends
            // at the touch-detected xy. Find-by-touch (joint torque) needs
            // the slack to recover from several-cm post-contact drift; find-
            // grasp-place (pressure) hits the block much closer to centre
            // and the wider threshold doesn't change its behavior.
            proximity_threshold: 0.10,
            max_grasp_size: 0.1,
        },
    };

    ArmWorld::new(scene, spec, /* gravity */ true)
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
            .map(|i| self.attach_joint_velocity_actuator(JointId(i as u32), None))
            .collect();
        let gripper_tx = self.attach_gripper_actuator(None);
        StandardArmPorts {
            encoder_rxs,
            velocity_txs,
            gripper_tx,
        }
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
        // ground + table + 5 bin pieces (floor + 4 walls) = 7
        assert_eq!(world.scene.fixtures().count(), 7);
        assert!(world.scene.object(BLOCK_OBJECT_ID).is_some());
        assert_eq!(block_id(&world), BLOCK_OBJECT_ID);
        assert_eq!(bin_id(&world), BIN_FIXTURE_ID);
        assert!(world.gravity_enabled);
    }

    /// Phase 3.4.5b: the four scenario world arms now carry a wrist joint
    /// (J3) plus a 5 cm wrist link. At q=0 the chain is straight along
    /// world +x, so the EE position lands at the old (0.8, 0, 0.8) plus
    /// the 5 cm wrist offset = (0.85, 0, 0.8). Locks in the spec change
    /// so a future regression to a 3-joint scenario arm is caught here.
    #[test]
    fn pick_and_place_world_arm_has_wrist_joint_and_link() {
        let world = build_pick_and_place_world();
        assert_eq!(
            world.arm.spec.joints.len(),
            4,
            "scenario arm should have 4 joints (Z-Y-Y-Y) post Phase 3.4.5b"
        );
        let ee = world.ee_pose();
        let pos = ee.translation;
        assert!(
            (pos.x - 0.85).abs() < 1e-5 && pos.y.abs() < 1e-5 && (pos.z - 0.8).abs() < 1e-5,
            "expected EE at (0.85, 0, 0.8) at q=0; got ({}, {}, {})",
            pos.x,
            pos.y,
            pos.z
        );
    }

    #[test]
    fn search_world_arm_has_wrist_joint() {
        let world = build_search_world(0);
        assert_eq!(world.arm.spec.joints.len(), 4);
    }

    #[test]
    fn build_search_world_places_block_in_region() {
        let world = build_search_world(42);
        let block = world.scene.object(BLOCK_OBJECT_ID).expect("block placed");
        let xy = (block.pose.translation.x, block.pose.translation.y);
        assert!(
            (SEARCH_REGION_X.0..=SEARCH_REGION_X.1).contains(&xy.0),
            "block x={} outside region",
            xy.0
        );
        assert!(
            (SEARCH_REGION_Y.0..=SEARCH_REGION_Y.1).contains(&xy.1),
            "block y={} outside region",
            xy.1
        );
    }

    #[test]
    fn build_search_world_is_deterministic_per_seed() {
        let a = build_search_world(42);
        let b = build_search_world(42);
        let pa = a.scene.object(BLOCK_OBJECT_ID).unwrap().pose.translation;
        let pb = b.scene.object(BLOCK_OBJECT_ID).unwrap().pose.translation;
        assert_eq!(pa.x, pb.x);
        assert_eq!(pa.y, pb.y);
    }

    #[test]
    fn build_search_world_varies_with_seed() {
        let a = build_search_world(1);
        let b = build_search_world(2);
        let pa = a.scene.object(BLOCK_OBJECT_ID).unwrap().pose.translation;
        let pb = b.scene.object(BLOCK_OBJECT_ID).unwrap().pose.translation;
        assert!(
            (pa.x - pb.x).abs() > 1e-6 || (pa.y - pb.y).abs() > 1e-6,
            "seeds 1 and 2 produced identical block xy ({}, {})",
            pa.x,
            pa.y
        );
    }
}

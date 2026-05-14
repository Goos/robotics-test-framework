//! Step 6.6: Released-object integration test. Drives an `ArmWorld` manually
//! (no `harness::run` because it consumes the world) through the full grasp →
//! release → gravity-fall → settle cycle.

#![cfg(feature = "e2e")]

use nalgebra::{Isometry3, Vector3};

use rtf_arm::{
    ports::{GripperCommand, JointId},
    spec::{ArmSpec, GripperSpec, JointSpec},
    world::ArmWorld,
};
use rtf_core::time::Duration;
use rtf_sim::{
    fixture::Fixture,
    object::{Object, ObjectId, ObjectState},
    primitive::Color,
    scene::Scene,
    shape::Shape,
};

#[test]
fn object_grasped_then_released_falls_to_table() {
    // Build scene: ground + a table fixture (table is informational here; the
    // block lands on the ground in this v1 test since we don't move the EE
    // over the table — we just verify the grasp/release/settle cycle).
    let mut scene = Scene::with_ground(0);
    scene.add_fixture(Fixture {
        id: 0,
        pose: Isometry3::translation(0.5, 0.0, 0.475),
        shape: Shape::Aabb {
            half_extents: Vector3::new(0.4, 0.4, 0.025),
        },
        is_support: true,
        color: Color::WHITE,
    });

    // Spec-intended geometry: three x-axis link offsets, EE at (0.6, 0, 0).
    // The block starts at the EE so the first gripper-close grasps it. After
    // Step 6.6.5 the gripper accepts Settled objects too, so the tick-1 snap
    // to ground (block bottom at z=-0.025 overlaps ground top z=0) doesn't
    // block the grasp. On release, gravity drops the block back onto the
    // ground at z=0.025 (sphere radius).
    use core::f32::consts::PI;
    let spec = ArmSpec {
        joints: vec![
            JointSpec::Revolute {
                axis: Vector3::z_axis(),
                limits: (-PI, PI)
            };
            3
        ],
        link_offsets: vec![Isometry3::translation(0.2, 0.0, 0.0); 3],
        gripper: GripperSpec {
            proximity_threshold: 0.5,
            max_grasp_size: 0.1,
        },
    };

    let block_id = ObjectId(99);
    scene.insert_object(Object::new(
        block_id,
        Isometry3::translation(0.6, 0.0, 0.0),
        Shape::Sphere { radius: 0.025 },
        0.1,
        true,
    ));
    let mut world = ArmWorld::new(scene, spec, /* gravity */ true);
    let _v_txs: Vec<_> = (0..3)
        .map(|i| world.attach_joint_velocity_actuator(JointId(i as u32), None))
        .collect();
    let g_tx = world.attach_gripper_actuator(None);

    // Phase 1: close gripper → grasp. Phase 3.2 slews separation at
    // 0.5 m/s, so going from open (0.04) to below the closed threshold
    // (0.02) takes ~40 ms; drive 60 ms to be safely past.
    g_tx.send(GripperCommand {
        target_separation: 0.012,
    });
    for _ in 0..60 {
        world.consume_actuators_and_integrate_inner(Duration::from_millis(1));
    }
    assert_eq!(
        world.arm.state.grasped,
        Some(block_id),
        "block should be grasped after the gripper slews closed"
    );

    // Phase 2: hold position (no joint velocity sent).
    for _ in 0..195 {
        world.consume_actuators_and_integrate_inner(Duration::from_millis(1));
    }
    assert_eq!(
        world.arm.state.grasped,
        Some(block_id),
        "should still be grasped"
    );

    // Phase 3: open gripper → release. Same slew rate; drive 60 ms.
    g_tx.send(GripperCommand {
        target_separation: 0.04,
    });
    for _ in 0..60 {
        world.consume_actuators_and_integrate_inner(Duration::from_millis(1));
    }
    assert_eq!(world.arm.state.grasped, None, "block should be released");

    // Phase 4: gravity pulls the released block down. Plenty of headroom.
    for _ in 0..2000 {
        world.consume_actuators_and_integrate_inner(Duration::from_millis(1));
    }
    let block = world.scene.object(block_id).expect("block exists");
    assert!(
        matches!(block.state, ObjectState::Settled { .. }),
        "block should have settled (ground or table); got state {:?}",
        block.state,
    );
}

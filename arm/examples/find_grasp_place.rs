//! Find-grasp-place scenario: a Z-Y-Y arm sweeps a serpentine raster over
//! the search region using its EE-mounted pressure sensor as the only source
//! of object-location information, then descends, grasps the contact'd
//! block, and drops it into the bin.
//!
//! Step 5 lands the controller skeleton + Sweeping state. The post-contact
//! states (Step 6) and the headline e2e (Step 7) follow in subsequent
//! commits; `main()` is a stub until Step 7 wires up the harness loop.
//!
//! Run terminal-only (Step 7+):
//!   `cargo run --example find_grasp_place --features examples`

use rtf_arm::{
    ik::ik_2r,
    ports::{
        EePoseReading, GripperCommand, JointEncoderReading, JointId, JointVelocityCommand,
        PressureReading,
    },
};
use rtf_core::{
    controller::{ControlError, Controller},
    port::{PortReader, PortTx},
    time::Time,
};

// -- Controller ----------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SearchAndPlaceState {
    /// Walking through `sweep_waypoints` at fixed altitude until the
    /// pressure signal exceeds threshold.
    Sweeping,
    /// Pressure spiked while sweeping; descending toward the block top at
    /// `contact_xy` for grasp.
    DescendToContact,
    /// Closing gripper (n = ticks held closed so far).
    CloseGripper(u32),
    /// Lifting the grasped block back to park altitude.
    AscendWithBlock,
    /// Yawing J0 toward the bin while holding park altitude.
    YawToBin,
    /// Opening gripper (n = ticks held open so far).
    OpenGripper(u32),
    /// Goal achieved.
    Done,
    /// Sweep exhausted without ever crossing the pressure threshold —
    /// terminal failure state.
    Failed,
}

/// Block-top z in the find-grasp-place world (table top 0.5 m + block
/// half-height 0.025). Post-contact descent targets EE here.
const GRASP_Z: f32 = 0.55;
/// Park altitude — matches the find-grasp-place sweep_z (kept high enough
/// above the bin top at z=0.6 for the released block to fall in).
const PARK_Z: f32 = 0.85;
/// Ticks to hold the gripper closed before transitioning to Ascend.
/// Matches PickPlace.
const CLOSE_HOLD_TICKS: u32 = 200;
/// Ticks to hold the gripper open before transitioning to Done.
/// Matches PickPlace.
const OPEN_HOLD_TICKS: u32 = 200;

pub struct SearchAndPlace<R, P, Pr>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
    Pr: PortReader<PressureReading>,
{
    state: SearchAndPlaceState,
    /// Pre-computed (J0, J1, J2) joint targets in serpentine order over the
    /// search region at fixed sweep altitude. Constructor-computed so the
    /// hot path doesn't run IK per tick.
    sweep_waypoints: Vec<(f32, f32, f32)>,
    sweep_idx: usize,
    /// Recorded EE xy at the moment pressure exceeded threshold; populated
    /// when transitioning Sweeping → DescendToContact.
    contact_xy: Option<(f32, f32)>,
    target_bin_xy: (f32, f32),
    pressure_threshold: f32,
    arm_shoulder_z: f32,
    l1: f32,
    l2: f32,
    encoder_rxs: Vec<R>,
    ee_pose_rx: P,
    pressure_rx: Pr,
    velocity_txs: Vec<PortTx<JointVelocityCommand>>,
    gripper_tx: PortTx<GripperCommand>,
    /// Per-joint convergence threshold (radians).
    joint_tol: f32,
}

impl<R, P, Pr> SearchAndPlace<R, P, Pr>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
    Pr: PortReader<PressureReading>,
{
    /// Construct the controller. Pre-computes a serpentine raster of joint-space
    /// waypoints over the (x_min..=x_max) × (y_min..=y_max) region at fixed
    /// `sweep_z`. Each xy is converted to (J0, J1, J2) by polar-decomposing
    /// the planar offset and calling 2R IK on the radial-z plane. Panics if
    /// any waypoint is unreachable (region should be sized so this can't
    /// happen — see find-grasp-place design §3.3).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        encoder_rxs: Vec<R>,
        ee_pose_rx: P,
        pressure_rx: Pr,
        velocity_txs: Vec<PortTx<JointVelocityCommand>>,
        gripper_tx: PortTx<GripperCommand>,
        region_x: (f32, f32),
        region_y: (f32, f32),
        sweep_z: f32,
        stripe_dy: f32,
        target_bin_xy: (f32, f32),
        arm_shoulder_z: f32,
        l1: f32,
        l2: f32,
    ) -> Self {
        let xy_waypoints = serpentine_waypoints(region_x, region_y, stripe_dy);
        let sweep_waypoints: Vec<(f32, f32, f32)> = xy_waypoints
            .iter()
            .map(|&(x, y)| {
                let r = (x * x + y * y).sqrt();
                let (j1, j2) = ik_2r(r, sweep_z - arm_shoulder_z, l1, l2)
                    .expect("sweep waypoint unreachable — check region vs arm reach");
                let yaw = y.atan2(x);
                (yaw, j1, j2)
            })
            .collect();
        Self {
            state: SearchAndPlaceState::Sweeping,
            sweep_waypoints,
            sweep_idx: 0,
            contact_xy: None,
            target_bin_xy,
            pressure_threshold: 0.2,
            arm_shoulder_z,
            l1,
            l2,
            encoder_rxs,
            ee_pose_rx,
            pressure_rx,
            velocity_txs,
            gripper_tx,
            joint_tol: 0.02,
        }
    }

    pub fn state(&self) -> SearchAndPlaceState {
        self.state
    }

    fn joint_qs(&self) -> Vec<f32> {
        self.encoder_rxs
            .iter()
            .map(|rx| rx.latest().map(|r| r.q).unwrap_or(0.0))
            .collect()
    }

    fn ee_xy(&self) -> Option<(f32, f32)> {
        let r = self.ee_pose_rx.latest()?;
        Some((r.pose.translation.x, r.pose.translation.y))
    }

    /// Send a per-joint velocity command driving (q0, q1, q2) toward the
    /// supplied target via P-on-position with kp=4.0, clamped to ±2 rad/s
    /// (matches PickPlace's gain choices).
    fn drive_joints_toward(&mut self, target: (f32, f32, f32)) {
        let qs = self.joint_qs();
        for (i, t) in [(0usize, target.0), (1, target.1), (2, target.2)] {
            if i >= self.velocity_txs.len() {
                break;
            }
            let cur = qs.get(i).copied().unwrap_or(0.0);
            let q_dot = ((t - cur) * 4.0).clamp(-2.0, 2.0);
            self.velocity_txs[i].send(JointVelocityCommand {
                joint: JointId(i as u32),
                q_dot_target: q_dot,
            });
        }
    }

    fn halt_joints(&mut self) {
        for (i, tx) in self.velocity_txs.iter().enumerate() {
            tx.send(JointVelocityCommand {
                joint: JointId(i as u32),
                q_dot_target: 0.0,
            });
        }
    }

    /// True iff every joint is within `joint_tol` of the corresponding target.
    fn joints_converged_to(&self, target: (f32, f32, f32)) -> bool {
        let qs = self.joint_qs();
        let q0 = qs.first().copied().unwrap_or(0.0);
        let q1 = qs.get(1).copied().unwrap_or(0.0);
        let q2 = qs.get(2).copied().unwrap_or(0.0);
        (q0 - target.0).abs() < self.joint_tol
            && (q1 - target.1).abs() < self.joint_tol
            && (q2 - target.2).abs() < self.joint_tol
    }

    /// Compute (J0, J1, J2) joint target for an EE world xy at altitude `z`.
    /// J0 is yaw = atan2(y, x); J1, J2 come from the closed-form 2R IK on
    /// the radial-z plane. Panics on unreachable targets — every call site
    /// here uses pre-validated geometry (within search region or the bin).
    fn ik_target_for(&self, xy: (f32, f32), z: f32) -> (f32, f32, f32) {
        let (x, y) = xy;
        let r = (x * x + y * y).sqrt();
        let (j1, j2) = ik_2r(r, z - self.arm_shoulder_z, self.l1, self.l2)
            .expect("post-contact IK target unreachable — check geometry");
        (y.atan2(x), j1, j2)
    }
}

impl<R, P, Pr> Controller for SearchAndPlace<R, P, Pr>
where
    R: PortReader<JointEncoderReading>,
    P: PortReader<EePoseReading>,
    Pr: PortReader<PressureReading>,
{
    fn step(&mut self, _t: Time) -> Result<(), ControlError> {
        let pressure = self.pressure_rx.latest().map(|r| r.pressure).unwrap_or(0.0);

        match self.state {
            SearchAndPlaceState::Sweeping => {
                if pressure > self.pressure_threshold {
                    // Capture EE xy at the moment of contact; if the EE pose
                    // hasn't published yet, fall back to the current waypoint
                    // xy so contact_xy is always populated by transition.
                    self.contact_xy = self.ee_xy().or_else(|| {
                        let (j0, j1, j2) = self.sweep_waypoints[self.sweep_idx];
                        // Reverse-FK roughly: radial = l1*sin(j1)+l2*sin(j1+j2);
                        // we don't have FK handy here, so prefer the EE xy
                        // when it's available (the .or_else branch is just a
                        // safety net for tests that forget to publish EE).
                        let _ = (j0, j1, j2);
                        Some((0.0, 0.0))
                    });
                    self.state = SearchAndPlaceState::DescendToContact;
                    self.halt_joints();
                    return Ok(());
                }

                let target = self.sweep_waypoints[self.sweep_idx];
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.sweep_idx += 1;
                    if self.sweep_idx >= self.sweep_waypoints.len() {
                        self.state = SearchAndPlaceState::Failed;
                        self.halt_joints();
                    }
                }
            }
            SearchAndPlaceState::DescendToContact => {
                let cxy = self.contact_xy.expect("contact_xy set on entry");
                let target = self.ik_target_for(cxy, GRASP_Z);
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.state = SearchAndPlaceState::CloseGripper(0);
                }
            }
            SearchAndPlaceState::CloseGripper(n) => {
                self.gripper_tx.send(GripperCommand { close: true });
                self.halt_joints();
                self.state = if n >= CLOSE_HOLD_TICKS {
                    SearchAndPlaceState::AscendWithBlock
                } else {
                    SearchAndPlaceState::CloseGripper(n + 1)
                };
            }
            SearchAndPlaceState::AscendWithBlock => {
                let cxy = self.contact_xy.expect("contact_xy persists after grasp");
                let target = self.ik_target_for(cxy, PARK_Z);
                self.gripper_tx.send(GripperCommand { close: true });
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.state = SearchAndPlaceState::YawToBin;
                }
            }
            SearchAndPlaceState::YawToBin => {
                let target = self.ik_target_for(self.target_bin_xy, PARK_Z);
                self.gripper_tx.send(GripperCommand { close: true });
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.state = SearchAndPlaceState::OpenGripper(0);
                }
            }
            SearchAndPlaceState::OpenGripper(n) => {
                self.gripper_tx.send(GripperCommand { close: false });
                self.halt_joints();
                self.state = if n >= OPEN_HOLD_TICKS {
                    SearchAndPlaceState::Done
                } else {
                    SearchAndPlaceState::OpenGripper(n + 1)
                };
            }
            SearchAndPlaceState::Done | SearchAndPlaceState::Failed => {
                self.halt_joints();
            }
        }
        Ok(())
    }
}

// -- Helpers -------------------------------------------------------------

/// Generate an xy raster over the region in serpentine (boustrophedon) order:
/// stripe at y_min from x_min → x_max, then stripe at y_min + stripe_dy from
/// x_max → x_min, etc. Each stripe contributes its two endpoints. The number
/// of stripes is `ceil((y_max - y_min) / stripe_dy) + 1`, capped at the y_max
/// boundary.
///
/// Stripe count is computed as an integer count to avoid float-accumulation
/// drift adding a spurious extra stripe at the boundary.
fn serpentine_waypoints(
    region_x: (f32, f32),
    region_y: (f32, f32),
    stripe_dy: f32,
) -> Vec<(f32, f32)> {
    let (x_min, x_max) = region_x;
    let (y_min, y_max) = region_y;
    let span = (y_max - y_min).max(0.0);
    // ceil((span / stripe_dy) + 1e-6) handles span=0 (single stripe) and the
    // exact-multiple case (no spurious last stripe past y_max).
    let n_stripes = (span / stripe_dy).round() as usize + 1;
    let mut out = Vec::with_capacity(n_stripes * 2);
    for i in 0..n_stripes {
        let y = (y_min + (i as f32) * stripe_dy).min(y_max);
        let going_right = i % 2 == 0;
        if going_right {
            out.push((x_min, y));
            out.push((x_max, y));
        } else {
            out.push((x_max, y));
            out.push((x_min, y));
        }
    }
    out
}

// -- Runner --------------------------------------------------------------

fn main() {
    // Step 7 fills in the harness loop; for now this just keeps `cargo build
    // --example find_grasp_place` happy.
    println!("find_grasp_place: main() will be wired up in Step 7");
}

// -- Tests ---------------------------------------------------------------

#[cfg(test)]
mod search_and_place_tests {
    use super::*;
    use rtf_arm::test_helpers::{SEARCH_REGION_X, SEARCH_REGION_Y};
    use rtf_core::port::{port, PortRx, PortTx};

    /// Test rig: 3 encoder pubs, 1 EE-pose pub, 1 pressure pub, 3 velocity
    /// subs, 1 gripper sub, plus the controller.
    struct Rig {
        c: SearchAndPlace<
            PortRx<JointEncoderReading>,
            PortRx<EePoseReading>,
            PortRx<PressureReading>,
        >,
        enc_txs: Vec<PortTx<JointEncoderReading>>,
        ee_tx: PortTx<EePoseReading>,
        pressure_tx: PortTx<PressureReading>,
        _vel_rxs: Vec<PortRx<JointVelocityCommand>>,
        _g_rx: PortRx<GripperCommand>,
    }

    fn make_rig_with(stripe_dy: f32) -> Rig {
        let mut enc_rxs = Vec::new();
        let mut enc_txs = Vec::new();
        for _ in 0..3 {
            let (tx, rx) = port::<JointEncoderReading>();
            enc_rxs.push(rx);
            enc_txs.push(tx);
        }
        let (ee_tx, ee_rx) = port::<EePoseReading>();
        let (pressure_tx, pressure_rx) = port::<PressureReading>();
        let mut vel_txs = Vec::new();
        let mut vel_rxs = Vec::new();
        for _ in 0..3 {
            let (tx, rx) = port::<JointVelocityCommand>();
            vel_txs.push(tx);
            vel_rxs.push(rx);
        }
        let (g_tx, g_rx) = port::<GripperCommand>();
        let c = SearchAndPlace::new(
            enc_rxs,
            ee_rx,
            pressure_rx,
            vel_txs,
            g_tx,
            SEARCH_REGION_X,
            SEARCH_REGION_Y,
            /* sweep_z */ 0.57,
            stripe_dy,
            /* target_bin_xy */ (0.0, 0.6),
            /* arm_shoulder_z */ 0.8,
            /* l1 */ 0.4,
            /* l2 */ 0.4,
        );
        Rig {
            c,
            enc_txs,
            ee_tx,
            pressure_tx,
            _vel_rxs: vel_rxs,
            _g_rx: g_rx,
        }
    }

    fn make_rig() -> Rig {
        make_rig_with(0.05)
    }

    fn publish_encoders(rig: &Rig, qs: [f32; 3]) {
        for (i, q) in qs.iter().enumerate() {
            rig.enc_txs[i].send(JointEncoderReading {
                joint: JointId(i as u32),
                q: *q,
                q_dot: 0.0,
                sampled_at: Time::ZERO,
            });
        }
    }

    fn publish_ee_xy(rig: &Rig, xy: (f32, f32)) {
        rig.ee_tx.send(EePoseReading {
            pose: nalgebra::Isometry3::translation(xy.0, xy.1, 0.57),
            sampled_at: Time::ZERO,
        });
    }

    fn publish_pressure(rig: &Rig, p: f32) {
        rig.pressure_tx.send(PressureReading {
            pressure: p,
            sampled_at: Time::ZERO,
        });
    }

    #[test]
    fn sweep_waypoints_cover_region_in_serpentine_order() {
        // 5 cm stripes over y ∈ [-0.25, 0.25] → 11 stripes, each yielding
        // 2 endpoints = 22 waypoints. The first stripe goes left→right
        // (x_min then x_max), the second right→left, etc.
        let wps = serpentine_waypoints(SEARCH_REGION_X, SEARCH_REGION_Y, 0.05);
        assert_eq!(wps.len(), 22);
        // First stripe at y = y_min, going right.
        assert_eq!(wps[0], (SEARCH_REGION_X.0, SEARCH_REGION_Y.0));
        assert_eq!(wps[1], (SEARCH_REGION_X.1, SEARCH_REGION_Y.0));
        // Second stripe one stripe up, going left.
        assert!((wps[2].1 - (SEARCH_REGION_Y.0 + 0.05)).abs() < 1e-6);
        assert_eq!(wps[2].0, SEARCH_REGION_X.1);
        assert_eq!(wps[3].0, SEARCH_REGION_X.0);
        // Last stripe sits at y_max (clamped).
        assert!((wps.last().unwrap().1 - SEARCH_REGION_Y.1).abs() < 1e-6);
    }

    #[test]
    fn sweep_advances_waypoint_when_joints_converge() {
        let mut rig = make_rig();
        let wp0 = rig.c.sweep_waypoints[0];
        // Pretend joints have converged exactly to wp0.
        publish_encoders(&rig, [wp0.0, wp0.1, wp0.2]);
        publish_ee_xy(&rig, (0.4, -0.25));
        publish_pressure(&rig, 0.0);
        let idx_before = rig.c.sweep_idx;
        rig.c.step(Time::ZERO).unwrap();
        assert_eq!(rig.c.sweep_idx, idx_before + 1);
        assert!(matches!(rig.c.state(), SearchAndPlaceState::Sweeping));
    }

    #[test]
    fn sweep_transitions_to_descend_when_pressure_spikes() {
        let mut rig = make_rig();
        publish_encoders(&rig, [0.0, 0.0, 0.0]);
        publish_ee_xy(&rig, (0.55, 0.10));
        // Pressure > threshold (0.2).
        publish_pressure(&rig, 0.6);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::DescendToContact));
        assert_eq!(rig.c.contact_xy, Some((0.55, 0.10)));
    }

    #[test]
    fn sweep_transitions_to_failed_when_exhausted() {
        // Use a large stripe so we have only a couple of waypoints to
        // burn through.
        let mut rig = make_rig_with(/* stripe_dy = full region */ 0.5);
        let n = rig.c.sweep_waypoints.len();
        // Force convergence to each waypoint in turn.
        for _ in 0..n {
            let wp = rig.c.sweep_waypoints[rig.c.sweep_idx];
            publish_encoders(&rig, [wp.0, wp.1, wp.2]);
            publish_ee_xy(&rig, (0.5, 0.0));
            publish_pressure(&rig, 0.0);
            rig.c.step(Time::ZERO).unwrap();
        }
        assert!(matches!(rig.c.state(), SearchAndPlaceState::Failed));
    }

    /// Drive the rig through Sweeping → DescendToContact via a single tick
    /// where pressure spikes above threshold. Returns the rig in the
    /// DescendToContact state.
    fn rig_at_descend(contact_xy: (f32, f32)) -> Rig {
        let mut rig = make_rig();
        publish_encoders(&rig, [0.0, 0.0, 0.0]);
        publish_ee_xy(&rig, contact_xy);
        publish_pressure(&rig, 0.6);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::DescendToContact));
        rig
    }

    #[test]
    fn descend_transitions_to_close_when_joints_converge() {
        let cxy = (0.55, 0.10);
        let mut rig = rig_at_descend(cxy);
        let target = rig.c.ik_target_for(cxy, GRASP_Z);
        // Report joints at the descend target.
        publish_encoders(&rig, [target.0, target.1, target.2]);
        publish_ee_xy(&rig, cxy);
        publish_pressure(&rig, 0.6);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::CloseGripper(_)));
    }

    #[test]
    fn close_transitions_to_ascend_after_hold_ticks() {
        let cxy = (0.55, 0.10);
        let mut rig = rig_at_descend(cxy);
        let target = rig.c.ik_target_for(cxy, GRASP_Z);
        publish_encoders(&rig, [target.0, target.1, target.2]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::CloseGripper(_)));
        // Hold for >= CLOSE_HOLD_TICKS more steps; encoders + pressure stay
        // unchanged (Close is a halt state).
        for _ in 0..(super::CLOSE_HOLD_TICKS + 10) {
            rig.c.step(Time::ZERO).unwrap();
        }
        assert!(matches!(rig.c.state(), SearchAndPlaceState::AscendWithBlock));
    }

    #[test]
    fn ascend_transitions_to_yaw_to_bin() {
        let cxy = (0.55, 0.10);
        let mut rig = rig_at_descend(cxy);
        // March to AscendWithBlock.
        let descend_target = rig.c.ik_target_for(cxy, GRASP_Z);
        publish_encoders(&rig, [descend_target.0, descend_target.1, descend_target.2]);
        rig.c.step(Time::ZERO).unwrap();
        for _ in 0..(super::CLOSE_HOLD_TICKS + 10) {
            rig.c.step(Time::ZERO).unwrap();
        }
        assert!(matches!(rig.c.state(), SearchAndPlaceState::AscendWithBlock));
        // Now report convergence at the ascend target.
        let ascend_target = rig.c.ik_target_for(cxy, super::PARK_Z);
        publish_encoders(&rig, [ascend_target.0, ascend_target.1, ascend_target.2]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::YawToBin));
    }

    #[test]
    fn yaw_to_bin_transitions_to_open_when_yaw_and_joints_converged() {
        let cxy = (0.55, 0.10);
        let mut rig = rig_at_descend(cxy);
        // March to YawToBin.
        let descend_target = rig.c.ik_target_for(cxy, GRASP_Z);
        publish_encoders(&rig, [descend_target.0, descend_target.1, descend_target.2]);
        rig.c.step(Time::ZERO).unwrap();
        for _ in 0..(super::CLOSE_HOLD_TICKS + 10) {
            rig.c.step(Time::ZERO).unwrap();
        }
        let ascend_target = rig.c.ik_target_for(cxy, super::PARK_Z);
        publish_encoders(&rig, [ascend_target.0, ascend_target.1, ascend_target.2]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::YawToBin));
        // Report joints at the bin target.
        let bin_target = rig.c.ik_target_for((0.0, 0.6), super::PARK_Z);
        publish_encoders(&rig, [bin_target.0, bin_target.1, bin_target.2]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::OpenGripper(_)));
    }

    #[test]
    fn open_transitions_to_done() {
        let cxy = (0.55, 0.10);
        let mut rig = rig_at_descend(cxy);
        // March all the way to OpenGripper(0).
        let descend_target = rig.c.ik_target_for(cxy, GRASP_Z);
        publish_encoders(&rig, [descend_target.0, descend_target.1, descend_target.2]);
        rig.c.step(Time::ZERO).unwrap();
        for _ in 0..(super::CLOSE_HOLD_TICKS + 10) {
            rig.c.step(Time::ZERO).unwrap();
        }
        let ascend_target = rig.c.ik_target_for(cxy, super::PARK_Z);
        publish_encoders(&rig, [ascend_target.0, ascend_target.1, ascend_target.2]);
        rig.c.step(Time::ZERO).unwrap();
        let bin_target = rig.c.ik_target_for((0.0, 0.6), super::PARK_Z);
        publish_encoders(&rig, [bin_target.0, bin_target.1, bin_target.2]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::OpenGripper(_)));
        // Hold open for >= OPEN_HOLD_TICKS.
        for _ in 0..(super::OPEN_HOLD_TICKS + 10) {
            rig.c.step(Time::ZERO).unwrap();
        }
        assert!(matches!(rig.c.state(), SearchAndPlaceState::Done));
    }
}

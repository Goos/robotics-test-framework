//! Find-grasp-place scenario: a Z-Y-Y arm sweeps a serpentine raster over
//! the search region using its EE-mounted pressure sensor as the only source
//! of object-location information, then descends, grasps the contacted
//! block, and drops it into the bin.
//!
//! Run a single seed (interactive demo, records to $TMPDIR/find_grasp_place.rrd
//! when --features viz-rerun):
//!   `cargo run --example find_grasp_place --features examples`
//!   `cargo run --example find_grasp_place --features viz-rerun`
//!
//! Run the 3-seed e2e regression as tests (each test records to
//! $TMPDIR/<test_name>.rrd when --features viz-rerun):
//!   `cargo test --example find_grasp_place --features examples`
//!   `cargo test --example find_grasp_place --features viz-rerun`
//!
//! View any saved rrd with `rerun <printed-path>` (install once via
//! `cargo install rerun-cli --version 0.21`).

use rtf_arm::{
    goals::PlaceInBin,
    ik::ik_3r,
    ports::{
        EePoseReading, GripperCommand, JointEncoderReading, JointId, JointVelocityCommand,
        PressureReading,
    },
    test_helpers::{bin_id, block_id, build_search_world, SEARCH_REGION_X, SEARCH_REGION_Y},
    RateHz,
};
use rtf_core::{
    controller::{ControlError, Controller},
    port::{PortReader, PortTx},
    time::{Duration, Time},
};
#[cfg(test)]
use rtf_harness::Termination;
use rtf_harness::{run, RunConfig};

// -- Controller ----------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SearchAndPlaceState {
    /// Initial transition from the world's q=0 pose to the first scan
    /// waypoint (wrist level, scan altitude). One-shot; drops into
    /// `Sweeping` once joints converge. Ensures we don't start sweeping
    /// from an arbitrary partially-IK'd pose.
    LiftToScanAltitude,
    /// Walking through `scan_waypoints` (wrist level, target_pitch=0,
    /// fingers extending horizontally forward) at `sweep_z` until the
    /// pressure signal exceeds threshold. Wrist-level keeps the fingers
    /// from bulldozing the block during the localization sweep.
    Sweeping,
    /// Step 4.1.1: second-pass fine-grained localization. After the
    /// coarse sweep records a peak that's only as accurate as the
    /// coarse stripe spacing (5 cm in y), drive a small fine raster
    /// (2 cm dy stripes, ±6 cm in y, ±6 cm in x) centred on
    /// `coarse_peak_xy` while accumulating a fresh weighted centroid.
    /// The denser raster ensures the actual block is within 1 cm of
    /// some stripe, so the centroid converges close to the block's
    /// true xy. Commits to `LiftAndRetract` once the fine raster is
    /// exhausted.
    LocalizeFineSweep,
    /// Post-localization retract: pull the EE radially inward (toward
    /// the shoulder) AND a few cm up so neither the wrist link arc
    /// nor the finger pillars touch the block while J3 swings through
    /// 90°. Target is `(contact_xy * RETRACT_FACTOR, scan_z + APPROACH_DZ)`
    /// at scan-pose pitch (wrist still level, J3 unchanged).
    LiftAndRetract,
    /// Pressure peak settled; J0/J1/J2 hold the retracted scan-pose
    /// target, J3 drives from 0 to π/2 over ~500 ms so the wrist
    /// transitions from "level" to "down". With the EE retracted, the
    /// wrist link's 5 cm arc sweeps in empty space well clear of the
    /// block.
    RotateWristForGrasp,
    /// J3 settled to grasp pose; drive J0/J1/J2 to bring the EE from the
    /// retracted-and-rotated position back over `contact_xy` at
    /// `scan_z + APPROACH_DZ` (now in grasp pose, fingers vertical).
    /// Approach is purely a horizontal arc in joint space — fingers
    /// pass over the block at altitude before descending.
    ApproachOverContact,
    /// EE positioned over `contact_xy` in grasp pose at approach
    /// altitude; descending to grasp_z to grip the block.
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
/// Cumulative chain pitch (J1+J2+J3) for the wrist-down GRASP pose:
/// EE +x = world -z, fingers protruding straight down. Per `ik_3r`'s
/// "+α-rotates-+x-toward-`-z`" convention, this is +π/2.
const GRASP_WRIST_PITCH: f32 = core::f32::consts::FRAC_PI_2;
/// Cumulative chain pitch for the wrist-level SCAN pose: EE +x = world
/// +x, fingers extending horizontally forward of the EE rather than
/// downward. The Step 4.1 redesign uses this during sweep so the
/// fingers don't bulldoze the block top before pressure-peak
/// localization can converge.
const SCAN_WRIST_PITCH: f32 = 0.0;
/// Vertical clearance (m) added on top of `scan_z` for both the
/// retract-before-rotate position and the approach-over-contact
/// position. 5 cm keeps the wrist link, fingers, and EE body well
/// above the block (top at z=0.55, scan_z=0.60) during rotation and
/// horizontal approach.
const APPROACH_DZ: f32 = 0.05;
/// Radial pullback factor applied to `contact_xy` for the retract
/// state. Multiplying both x and y by 0.70 pulls the EE ~30%
/// shoulder-ward along the line from the shoulder to the block,
/// moving the wrist link's 5 cm rotation arc into empty space. Yaw
/// stays the same (atan2 invariant under uniform xy scaling).
const RETRACT_FACTOR: f32 = 0.70;
/// Ticks for the J3 wrist rotation from scan-pose (0) to grasp-pose
/// (π/2) at 0.5 m/s-equivalent (~3.14 rad/s of joint motion, ~500 ms
/// of slew). 600 ticks @ 1 ms gives the rotation 600 ms to complete +
/// settle within `joint_tol`.
const ROTATE_WRIST_TICKS: u32 = 600;
/// Step 4.1.1: fine-raster stripe spacing (m). Tighter than the
/// coarse `stripe_dy=0.05` so any block is within 1 cm of some
/// stripe, eliminating the off-stripe localization bias that the
/// coarse-only weighted centroid suffers from.
const FINE_STRIPE_DY: f32 = 0.02;
/// Step 4.1.1: fine-raster y half-range (m) around the coarse peak.
/// 6 cm covers ±1 coarse-stripe spacing (5 cm) plus 1 cm slack on
/// each side, ensuring the true block is captured even if the
/// coarse peak xy is off by a coarse-stripe width.
const FINE_HALF_RANGE_Y: f32 = 0.06;
/// Step 4.1.1: fine-raster x half-range (m) per stripe around the
/// coarse peak. The coarse sweep moves at 30 cm/stripe along x, so
/// the coarse peak xy can be off by several cm in x too (the
/// pressure peak fires once the EE leaves the contact zone, so the
/// recorded x is biased into the EE-direction-of-travel). 6 cm
/// matches the y range and covers typical x offsets.
const FINE_HALF_RANGE_X: f32 = 0.06;
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
    /// Pre-computed (J0, J1, J2, J3) joint targets in serpentine order
    /// over the search region at fixed `scan_z`. Constructor-computed so
    /// the hot path doesn't run IK per tick. Step 4.1: J3 keeps the
    /// wrist LEVEL during sweep (target_pitch=0) so the fingers extend
    /// horizontally forward of the EE rather than downward into the
    /// block top.
    scan_waypoints: Vec<(f32, f32, f32, f32)>,
    sweep_idx: usize,
    /// Sweep altitude in world frame (m). Held constant across the
    /// scan_waypoints. Step 4.1: at 0.60 the EE sits 5 cm above the
    /// block top — enough to clear the wrist-level finger pillars
    /// (which extend ±0.04 m vertically around the EE in scan pose).
    /// Pair with an enlarged pressure-sensor eps so the sensor still
    /// reaches the block at this altitude.
    scan_z: f32,
    /// Tick counter for `RotateWristForGrasp`: drives J3 from 0 to
    /// `GRASP_WRIST_PITCH` over `ROTATE_WRIST_TICKS` ticks while
    /// J0/J1/J2 hold the scan-pose target above `contact_xy`.
    rotate_wrist_n: u32,
    /// Pressure-weighted centroid accumulator: sum of `(p * x, p * y)`
    /// over every sample above `pressure_threshold` since the start of
    /// the current contact event, plus the cumulative pressure weight.
    /// Step 4.1: replaces the previous "max-pressure xy" peak with a
    /// centroid so a block falling between two adjacent sweep stripes
    /// localizes to the y-midpoint of those stripes rather than
    /// snapping to whichever stripe happened to register the
    /// fractionally higher peak. Improves grasp accuracy for
    /// off-stripe blocks (seed 1337 corner case). Step 4.1.1: zeroed
    /// on entry to `LocalizeFineSweep` so the fine-raster centroid
    /// is independent of the coarse-sweep contributions.
    pressure_weighted_xy: (f64, f64),
    pressure_weight_sum: f64,
    /// Last known max pressure — kept only for the threshold-fall-off
    /// commit logic (`peak > threshold && current < threshold` → commit).
    peak_pressure: f32,
    /// Step 4.1.1: coarse-sweep peak xy. Set on Sweeping →
    /// LocalizeFineSweep transition and used as the centre of the
    /// fine raster.
    coarse_peak_xy: Option<(f32, f32)>,
    /// Step 4.1.1: fine-raster waypoint xy *offsets* relative to
    /// `coarse_peak_xy`. Pre-computed at construction (just a small
    /// serpentine in (x_offset, y_offset) space). The current target
    /// xy in world frame is `coarse_peak_xy + fine_raster_offsets[fine_idx]`,
    /// re-IK'd each tick (cheap; just a handful of waypoints).
    fine_raster_offsets: Vec<(f32, f32)>,
    fine_idx: usize,
    /// Final commit position for descent (= peak_xy at transition time).
    contact_xy: Option<(f32, f32)>,
    target_bin_xy: (f32, f32),
    pressure_threshold: f32,
    arm_shoulder_z: f32,
    l1: f32,
    l2: f32,
    /// Phase 3.4.5b: wrist link length, fed to ik_3r so the wrist J3 is
    /// driven to keep EE +x = world -z (fingers pointing down).
    l3: f32,
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
    /// Construct the controller. Pre-computes a serpentine raster of
    /// joint-space waypoints over the (x_min..=x_max) × (y_min..=y_max)
    /// region at fixed `scan_z` with the wrist LEVEL (target_pitch=0,
    /// fingers horizontal). Each xy is converted to (J0, J1, J2, J3) by
    /// polar-decomposing the planar offset and calling 3R IK. Panics if
    /// any waypoint is unreachable.
    ///
    /// The Step 4.1 redesign separates the scan pose (used here, fingers
    /// out of the way of the block) from the grasp pose (wrist down,
    /// computed in `ik_target_for` for descent + ascent + place).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        encoder_rxs: Vec<R>,
        ee_pose_rx: P,
        pressure_rx: Pr,
        velocity_txs: Vec<PortTx<JointVelocityCommand>>,
        gripper_tx: PortTx<GripperCommand>,
        region_x: (f32, f32),
        region_y: (f32, f32),
        scan_z: f32,
        stripe_dy: f32,
        target_bin_xy: (f32, f32),
        arm_shoulder_z: f32,
        l1: f32,
        l2: f32,
        l3: f32,
    ) -> Self {
        let xy_waypoints = serpentine_waypoints(region_x, region_y, stripe_dy);
        let scan_waypoints: Vec<(f32, f32, f32, f32)> = xy_waypoints
            .iter()
            .map(|&(x, y)| {
                let r = (x * x + y * y).sqrt();
                let (j1, j2, j3) =
                    ik_3r(r, scan_z - arm_shoulder_z, SCAN_WRIST_PITCH, l1, l2, l3)
                        .expect("scan waypoint unreachable — check region vs arm reach");
                let yaw = y.atan2(x);
                (yaw, j1, j2, j3)
            })
            .collect();
        let fine_raster_offsets = serpentine_waypoints(
            (-FINE_HALF_RANGE_X, FINE_HALF_RANGE_X),
            (-FINE_HALF_RANGE_Y, FINE_HALF_RANGE_Y),
            FINE_STRIPE_DY,
        );
        Self {
            state: SearchAndPlaceState::LiftToScanAltitude,
            scan_waypoints,
            sweep_idx: 0,
            scan_z,
            rotate_wrist_n: 0,
            pressure_weighted_xy: (0.0, 0.0),
            pressure_weight_sum: 0.0,
            peak_pressure: 0.0,
            coarse_peak_xy: None,
            fine_raster_offsets,
            fine_idx: 0,
            contact_xy: None,
            target_bin_xy,
            pressure_threshold: 0.2,
            arm_shoulder_z,
            l1,
            l2,
            l3,
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

    /// Send a per-joint velocity command driving (q0, q1, q2, q3) toward
    /// the supplied target via P-on-position with kp=4.0, clamped to
    /// ±2 rad/s (matches PickPlace's gain choices).
    fn drive_joints_toward(&mut self, target: (f32, f32, f32, f32)) {
        self.drive_joints_toward_with_clamp(target, 2.0);
    }

    /// Same as `drive_joints_toward` but with a configurable speed
    /// clamp. Slow descent (clamp ~0.5) reduces the impulse against
    /// the block on contact, keeping the friction-grasp window viable.
    fn drive_joints_toward_with_clamp(&mut self, target: (f32, f32, f32, f32), clamp: f32) {
        let qs = self.joint_qs();
        for (i, t) in [
            (0usize, target.0),
            (1, target.1),
            (2, target.2),
            (3, target.3),
        ] {
            if i >= self.velocity_txs.len() {
                break;
            }
            let cur = qs.get(i).copied().unwrap_or(0.0);
            let q_dot = ((t - cur) * 4.0).clamp(-clamp, clamp);
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

    /// True iff every joint (J0-J3) is within `joint_tol` of its target.
    fn joints_converged_to(&self, target: (f32, f32, f32, f32)) -> bool {
        let qs = self.joint_qs();
        let q0 = qs.first().copied().unwrap_or(0.0);
        let q1 = qs.get(1).copied().unwrap_or(0.0);
        let q2 = qs.get(2).copied().unwrap_or(0.0);
        let q3 = qs.get(3).copied().unwrap_or(0.0);
        (q0 - target.0).abs() < self.joint_tol
            && (q1 - target.1).abs() < self.joint_tol
            && (q2 - target.2).abs() < self.joint_tol
            && (q3 - target.3).abs() < self.joint_tol
    }

    /// Compute (J0, J1, J2, J3) joint target for an EE world xy at
    /// altitude `z` with the wrist DOWN (fingers pointing straight at
    /// the table). Used for descent, ascent, and bin-place — every
    /// motion that needs to actually grip the block.
    fn ik_target_for(&self, xy: (f32, f32), z: f32) -> (f32, f32, f32, f32) {
        self.ik_target_with_pitch(xy, z, GRASP_WRIST_PITCH)
    }

    /// Compute the joint target for the retract-before-rotate state:
    /// EE pulled radially inward to `contact_xy * RETRACT_FACTOR` and
    /// up by `APPROACH_DZ`, wrist still level (scan pose). Used by
    /// both `LiftAndRetract` (entry) and `RotateWristForGrasp`
    /// (J0/J1/J2 hold while J3 swings).
    fn retracted_scan_target(&self, contact_xy: (f32, f32)) -> (f32, f32, f32, f32) {
        let retracted_xy = (
            contact_xy.0 * RETRACT_FACTOR,
            contact_xy.1 * RETRACT_FACTOR,
        );
        self.ik_target_with_pitch(retracted_xy, self.scan_z + APPROACH_DZ, SCAN_WRIST_PITCH)
    }

    /// Generic 4-DOF IK helper: `(yaw, J1, J2, J3)` for a target EE xy
    /// at altitude `z` with cumulative pitch `target_pitch`. Panics on
    /// unreachable targets — call sites use pre-validated geometry
    /// (within search region or the bin).
    fn ik_target_with_pitch(
        &self,
        xy: (f32, f32),
        z: f32,
        target_pitch: f32,
    ) -> (f32, f32, f32, f32) {
        let (x, y) = xy;
        let r = (x * x + y * y).sqrt();
        let (j1, j2, j3) = ik_3r(
            r,
            z - self.arm_shoulder_z,
            target_pitch,
            self.l1,
            self.l2,
            self.l3,
        )
        .expect("IK target unreachable — check geometry");
        (y.atan2(x), j1, j2, j3)
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
            SearchAndPlaceState::LiftToScanAltitude => {
                // Drive the arm from its q=0 starting pose into the
                // first scan waypoint. Drops to `Sweeping` once joints
                // converge so peak-tracking starts with the EE actually
                // at scan altitude (not partway through the lift).
                let target = self.scan_waypoints[self.sweep_idx];
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.state = SearchAndPlaceState::Sweeping;
                }
            }
            SearchAndPlaceState::Sweeping => {
                // Pressure-weighted centroid: every sample above the
                // threshold contributes (pressure * ee_xy) to the
                // running sum. The committed contact_xy is
                // sum / weight, which interpolates between adjacent
                // sweep stripes rather than snapping to the
                // momentarily-strongest one. peak_pressure is still
                // tracked so the fall-off commit logic still fires.
                if pressure > self.pressure_threshold {
                    if let Some((x, y)) = self.ee_xy() {
                        let p = pressure as f64;
                        self.pressure_weighted_xy.0 += p * x as f64;
                        self.pressure_weighted_xy.1 += p * y as f64;
                        self.pressure_weight_sum += p;
                    }
                    if pressure > self.peak_pressure {
                        self.peak_pressure = pressure;
                    }
                }

                // Step 4.1.1: when the coarse sweep finishes a contact
                // event (peak above threshold, pressure now below),
                // hand off to LocalizeFineSweep with the coarse
                // centroid as the centre of the fine raster. The
                // coarse centroid is precise enough to seed a fine
                // raster ±6 cm around it; the fine raster's denser
                // 2 cm stripes then refine xy to ~1 cm accuracy.
                if self.peak_pressure > self.pressure_threshold
                    && pressure < self.pressure_threshold
                    && self.pressure_weight_sum > 0.0
                {
                    let cx = (self.pressure_weighted_xy.0 / self.pressure_weight_sum) as f32;
                    let cy = (self.pressure_weighted_xy.1 / self.pressure_weight_sum) as f32;
                    self.coarse_peak_xy = Some((cx, cy));
                    // Reset accumulators for a fresh fine-raster
                    // centroid; reset peak so the fine state's
                    // post-fine-sweep diagnostics use fine peak.
                    self.pressure_weighted_xy = (0.0, 0.0);
                    self.pressure_weight_sum = 0.0;
                    self.peak_pressure = 0.0;
                    self.fine_idx = 0;
                    self.state = SearchAndPlaceState::LocalizeFineSweep;
                    self.halt_joints();
                    return Ok(());
                }

                let target = self.scan_waypoints[self.sweep_idx];
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.sweep_idx += 1;
                    if self.sweep_idx >= self.scan_waypoints.len() {
                        self.state = SearchAndPlaceState::Failed;
                        self.halt_joints();
                    }
                }
            }
            SearchAndPlaceState::LocalizeFineSweep => {
                // Walk a small dense raster centred on coarse_peak_xy
                // (set on Sweeping → LocalizeFineSweep transition).
                // Same scan pose (wrist level), same scan_z, just
                // tighter stripe spacing. Accumulate a fresh
                // pressure-weighted centroid; commit it once the
                // entire raster is exhausted.
                let centre = self
                    .coarse_peak_xy
                    .expect("coarse_peak_xy set on Sweeping exit");

                if pressure > self.pressure_threshold {
                    if let Some((x, y)) = self.ee_xy() {
                        let p = pressure as f64;
                        self.pressure_weighted_xy.0 += p * x as f64;
                        self.pressure_weighted_xy.1 += p * y as f64;
                        self.pressure_weight_sum += p;
                    }
                    if pressure > self.peak_pressure {
                        self.peak_pressure = pressure;
                    }
                }

                // Raster exhausted → commit centroid and proceed.
                // Fall back to coarse_peak_xy if the fine raster
                // didn't see anything (block displaced during
                // earlier interaction, or coarse peak was a phantom).
                if self.fine_idx >= self.fine_raster_offsets.len() {
                    let final_xy = if self.pressure_weight_sum > 0.0 {
                        (
                            (self.pressure_weighted_xy.0 / self.pressure_weight_sum) as f32,
                            (self.pressure_weighted_xy.1 / self.pressure_weight_sum) as f32,
                        )
                    } else {
                        centre
                    };
                    self.contact_xy = Some(final_xy);
                    self.rotate_wrist_n = 0;
                    self.state = SearchAndPlaceState::LiftAndRetract;
                    self.halt_joints();
                    return Ok(());
                }

                let (dx, dy) = self.fine_raster_offsets[self.fine_idx];
                let target = self.ik_target_with_pitch(
                    (centre.0 + dx, centre.1 + dy),
                    self.scan_z,
                    SCAN_WRIST_PITCH,
                );
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.fine_idx += 1;
                }
            }
            SearchAndPlaceState::LiftAndRetract => {
                // Drive EE radially inward (cxy * RETRACT_FACTOR) and
                // up by APPROACH_DZ so the wrist-link rotation arc
                // happens in empty space, well clear of the block.
                // J3 unchanged (still scan-pose / wrist level).
                let cxy = self.contact_xy.expect("contact_xy set on Sweeping exit");
                let target = self.retracted_scan_target(cxy);
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.state = SearchAndPlaceState::RotateWristForGrasp;
                }
            }
            SearchAndPlaceState::RotateWristForGrasp => {
                // Hold J0/J1/J2 at the retracted scan target; drive
                // J3 from its scan-pose value (cumulative pitch 0,
                // wrist level) to scan-J3 + π/2 (cumulative pitch
                // π/2, wrist down). Only J3 moves; with the EE
                // retracted ~30% radially, the wrist link's 5 cm arc
                // sweeps in empty space well clear of the block.
                let cxy = self.contact_xy.expect("contact_xy set on Sweeping exit");
                let scan_target = self.retracted_scan_target(cxy);
                let target = (
                    scan_target.0,
                    scan_target.1,
                    scan_target.2,
                    scan_target.3 + GRASP_WRIST_PITCH,
                );
                self.drive_joints_toward(target);
                self.rotate_wrist_n += 1;
                if self.rotate_wrist_n >= ROTATE_WRIST_TICKS
                    && self.joints_converged_to(target)
                {
                    self.state = SearchAndPlaceState::ApproachOverContact;
                }
            }
            SearchAndPlaceState::ApproachOverContact => {
                // Drive EE from the retracted+rotated position back
                // over `contact_xy` at `scan_z + APPROACH_DZ` in
                // grasp pose (wrist down). This is a horizontal arc
                // in joint space; fingers pass over the block at
                // altitude before the descent commits.
                let cxy = self.contact_xy.expect("contact_xy set on Sweeping exit");
                let target = self.ik_target_for(cxy, self.scan_z + APPROACH_DZ);
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.state = SearchAndPlaceState::DescendToContact;
                }
            }
            SearchAndPlaceState::DescendToContact => {
                let cxy = self.contact_xy.expect("contact_xy set on entry");
                let target = self.ik_target_for(cxy, GRASP_Z);
                // Slow descent (clamp 0.5 rad/s) so the EE arrives
                // gently — Step 4.1 redesign: a 2 rad/s descent
                // imparts enough lateral impulse via wrist-link side
                // contact to knock the block 8+ cm before fingers can
                // close around it.
                self.drive_joints_toward_with_clamp(target, 0.5);
                if self.joints_converged_to(target) {
                    self.state = SearchAndPlaceState::CloseGripper(0);
                }
            }
            SearchAndPlaceState::CloseGripper(n) => {
                self.gripper_tx.send(GripperCommand {
                    target_separation: 0.012,
                });
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
                self.gripper_tx.send(GripperCommand {
                    target_separation: 0.012,
                });
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.state = SearchAndPlaceState::YawToBin;
                }
            }
            SearchAndPlaceState::YawToBin => {
                let target = self.ik_target_for(self.target_bin_xy, PARK_Z);
                self.gripper_tx.send(GripperCommand {
                    target_separation: 0.012,
                });
                self.drive_joints_toward(target);
                if self.joints_converged_to(target) {
                    self.state = SearchAndPlaceState::OpenGripper(0);
                }
            }
            SearchAndPlaceState::OpenGripper(n) => {
                self.gripper_tx.send(GripperCommand {
                    target_separation: 0.04,
                });
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

fn run_one_seed(seed: u64, rrd_name: &str) -> rtf_harness::RunResult {
    run_one_seed_with(seed, rrd_name, /* debug_overlay */ false)
}

fn run_one_seed_with(
    seed: u64,
    rrd_name: &str,
    #[cfg_attr(not(feature = "physics-rapier"), allow(unused_variables))] debug_overlay: bool,
) -> rtf_harness::RunResult {
    let _ = rrd_name; // used only when viz-rerun is on
    let mut world = build_search_world(seed);
    #[cfg(feature = "physics-rapier")]
    if debug_overlay {
        world.enable_debug_overlay(true);
    }
    let ports = world.attach_standard_arm_ports();
    let ee_pose_rx = world.attach_ee_pose_sensor(RateHz::new(100));
    // Step 4.1: eps bumped from 0.03 → 0.06 m so the pressure sensor
    // still reaches the block top (z=0.55) from the new
    // finger-clearing scan altitude (0.60 m).
    let pressure_rx = world.attach_pressure_sensor(RateHz::new(1000), 0.06);
    let block = block_id(&world);
    let bin = bin_id(&world);

    let controller = SearchAndPlace::new(
        ports.encoder_rxs,
        ee_pose_rx,
        pressure_rx,
        ports.velocity_txs,
        ports.gripper_tx,
        SEARCH_REGION_X,
        SEARCH_REGION_Y,
        // Step 4.1: scan altitude bumped from 0.57 → 0.60 m so the
        // wrist-level fingers (which protrude ±0.04 m vertically
        // around the EE in scan pose) clear the block top (z=0.55).
        // Pair with an enlarged pressure-sensor eps so the sensor
        // still reaches the block at this altitude.
        /* scan_z */
        0.60,
        /* stripe_dy */ 0.05,
        /* target_bin_xy */ (0.0, 0.6),
        /* arm_shoulder_z */ 0.8,
        /* l1 */ 0.4,
        /* l2 */ 0.4,
        /* l3 (wrist link, Phase 3.4.5b) */ 0.05,
    );
    let goal = PlaceInBin::new(block, bin);
    let cfg = RunConfig::default()
        .with_deadline(Duration::from_secs(30))
        .with_seed(seed);

    #[cfg(feature = "viz-rerun")]
    {
        if std::env::var("RTF_DEBUG_OVERLAY").as_deref() == Ok("1") {
            world.enable_debug_overlay(true);
        }
        match rtf_viz::maybe_recorder_for(rrd_name) {
            Some(rec) => run(world, controller, goal, cfg.with_recorder(rec)),
            None => run(world, controller, goal, cfg),
        }
    }
    #[cfg(not(feature = "viz-rerun"))]
    {
        run(world, controller, goal, cfg)
    }
}

fn main() {
    let seed = 42_u64;
    let res = run_one_seed(seed, "find_grasp_place");
    println!(
        "find_grasp_place(seed={seed}): terminated_by={:?}, final_time={:?}, score={}",
        res.terminated_by, res.final_time, res.score.value,
    );
}

#[test]
fn find_grasp_place_seed_1() {
    let seed = 1_u64;
    let res = run_one_seed(seed, "find_grasp_place_seed_1");
    eprintln!(
        "seed={seed}: terminated_by={:?}, final_time={:?}, score={}",
        res.terminated_by, res.final_time, res.score.value,
    );
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "seed {seed} did not converge in 30s; terminated_by={:?}, score={}",
        res.terminated_by,
        res.score.value,
    );
    assert!(res.score.value > 0.9);
}

// Step 4.1 redesign re-enabled the previously-ignored seeds: scan pose
// (wrist level, target_pitch=0) keeps the fingers horizontal during
// sweep so they no longer ploughs through the block top, and a
// dedicated RotateWristForGrasp state pivots J3 from 0 → π/2 in place
// after pressure-peak detection. See
// `docs/plans/2026-05-08-sweep-controllers-redesign-plan.md`.
#[test]
fn find_grasp_place_seed_42() {
    let seed = 42_u64;
    let res = run_one_seed(seed, "find_grasp_place_seed_42");
    eprintln!(
        "seed={seed}: terminated_by={:?}, final_time={:?}, score={}",
        res.terminated_by, res.final_time, res.score.value,
    );
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "seed {seed} did not converge in 30s; terminated_by={:?}, score={}",
        res.terminated_by,
        res.score.value,
    );
    assert!(res.score.value > 0.9);
}

#[test]
fn find_grasp_place_seed_1337() {
    let seed = 1337_u64;
    let res = run_one_seed(seed, "find_grasp_place_seed_1337");
    eprintln!(
        "seed={seed}: terminated_by={:?}, final_time={:?}, score={}",
        res.terminated_by, res.final_time, res.score.value,
    );
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "seed {seed} did not converge in 30s; terminated_by={:?}, score={}",
        res.terminated_by,
        res.score.value,
    );
    assert!(res.score.value > 0.9);
}

/// Sanity-check that turning the Rapier debug overlay on doesn't break the
/// scenario or panic. The overlay adds Line primitives to every snapshot,
/// which materially changes per-tick work — this test asserts it's
/// nonetheless cheap enough to still converge under the standard deadline.
#[test]
fn find_grasp_place_seed_42_with_debug_overlay() {
    let seed = 42_u64;
    let res = run_one_seed_with(seed, "find_grasp_place_seed_42_overlay", true);
    assert!(
        matches!(res.terminated_by, Termination::GoalComplete),
        "seed {seed} (overlay on) did not converge; terminated_by={:?}, score={}",
        res.terminated_by,
        res.score.value,
    );
    assert!(res.score.value > 0.9);
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
        for _ in 0..N_JOINTS {
            let (tx, rx) = port::<JointEncoderReading>();
            enc_rxs.push(rx);
            enc_txs.push(tx);
        }
        let (ee_tx, ee_rx) = port::<EePoseReading>();
        let (pressure_tx, pressure_rx) = port::<PressureReading>();
        let mut vel_txs = Vec::new();
        let mut vel_rxs = Vec::new();
        for _ in 0..N_JOINTS {
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
            /* scan_z */ 0.60,
            stripe_dy,
            /* target_bin_xy */ (0.0, 0.6),
            /* arm_shoulder_z */ 0.8,
            /* l1 */ 0.4,
            /* l2 */ 0.4,
            /* l3 */ 0.05,
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

    /// Phase 3.4.5b: 4-joint arm (Z-Y-Y-Y) — encoder/velocity ports must
    /// match build_search_world's spec.
    const N_JOINTS: usize = 4;

    fn make_rig() -> Rig {
        make_rig_with(0.05)
    }

    fn publish_encoders(rig: &Rig, qs: [f32; N_JOINTS]) {
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
    fn scan_waypoints_cover_region_in_serpentine_order() {
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

    /// Step 4.1: drive the rig past the new `LiftToScanAltitude` start
    /// state by reporting joints already at the first scan waypoint —
    /// one tick lands us in `Sweeping`. Inner-test helper used by the
    /// state-machine tests below.
    fn enter_sweeping(rig: &mut Rig) {
        let wp0 = rig.c.scan_waypoints[0];
        publish_encoders(rig, [wp0.0, wp0.1, wp0.2, wp0.3]);
        publish_ee_xy(rig, (0.0, 0.0));
        publish_pressure(rig, 0.0);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::Sweeping));
    }

    #[test]
    fn sweep_advances_waypoint_when_joints_converge() {
        let mut rig = make_rig();
        enter_sweeping(&mut rig);
        let wp0 = rig.c.scan_waypoints[0];
        // Pretend joints have converged exactly to wp0.
        publish_encoders(&rig, [wp0.0, wp0.1, wp0.2, wp0.3]);
        publish_ee_xy(&rig, (0.4, -0.25));
        publish_pressure(&rig, 0.0);
        let idx_before = rig.c.sweep_idx;
        rig.c.step(Time::ZERO).unwrap();
        assert_eq!(rig.c.sweep_idx, idx_before + 1);
        assert!(matches!(rig.c.state(), SearchAndPlaceState::Sweeping));
    }

    #[test]
    fn sweep_transitions_to_localize_fine_sweep_after_pressure_peak_falls_off() {
        let mut rig = make_rig();
        enter_sweeping(&mut rig);
        // Tick 1: EE over the block at xy=(0.55, 0.10), pressure peaks
        // at 0.6. Peak gets recorded, but commit waits for pressure to
        // fall back below threshold.
        publish_encoders(&rig, [0.0, 0.0, 0.0, 0.0]);
        publish_ee_xy(&rig, (0.55, 0.10));
        publish_pressure(&rig, 0.6);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::Sweeping));
        // Tick 2: EE has moved past the block, pressure drops to 0.0 →
        // controller commits the COARSE peak and enters
        // LocalizeFineSweep (Step 4.1.1). The coarse peak xy is
        // stored in `coarse_peak_xy`; `contact_xy` won't be filled in
        // until the fine raster commits.
        publish_ee_xy(&rig, (0.60, 0.10));
        publish_pressure(&rig, 0.0);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(
            rig.c.state(),
            SearchAndPlaceState::LocalizeFineSweep
        ));
        assert_eq!(rig.c.coarse_peak_xy, Some((0.55, 0.10)));
        assert_eq!(rig.c.contact_xy, None);
    }

    #[test]
    fn sweep_transitions_to_failed_when_exhausted() {
        // Use a large stripe so we have only a couple of waypoints to
        // burn through.
        let mut rig = make_rig_with(/* stripe_dy = full region */ 0.5);
        enter_sweeping(&mut rig);
        let n = rig.c.scan_waypoints.len();
        // Force convergence to each waypoint in turn.
        for _ in 0..n {
            let wp = rig.c.scan_waypoints[rig.c.sweep_idx];
            publish_encoders(&rig, [wp.0, wp.1, wp.2, wp.3]);
            publish_ee_xy(&rig, (0.5, 0.0));
            publish_pressure(&rig, 0.0);
            rig.c.step(Time::ZERO).unwrap();
        }
        assert!(matches!(rig.c.state(), SearchAndPlaceState::Failed));
    }

    /// Drive the rig through LiftToScanAltitude → Sweeping →
    /// LocalizeFineSweep → LiftAndRetract → RotateWristForGrasp →
    /// ApproachOverContact → DescendToContact. Step 4.1.1 inserted
    /// LocalizeFineSweep between Sweeping and LiftAndRetract; this
    /// helper drives the rig past the fine raster by reporting
    /// joints already at each fine waypoint and keeping pressure at
    /// 0.6 over `contact_xy` so the fine centroid converges to it.
    fn rig_at_descend(contact_xy: (f32, f32)) -> Rig {
        let mut rig = make_rig();
        enter_sweeping(&mut rig);
        // Peak: EE at contact_xy with pressure above threshold.
        publish_ee_xy(&rig, contact_xy);
        publish_pressure(&rig, 0.6);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::Sweeping));
        // Commit: pressure drops below threshold → LocalizeFineSweep.
        publish_ee_xy(&rig, (contact_xy.0 + 0.05, contact_xy.1));
        publish_pressure(&rig, 0.0);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(
            rig.c.state(),
            SearchAndPlaceState::LocalizeFineSweep
        ));
        // Drive the entire fine raster: for each offset, report
        // joints at that waypoint AND pressure 0.6 over contact_xy
        // so the fine centroid accumulates toward it. Two ticks per
        // offset: one to drive (joints already there → fine_idx
        // increments), and the loop continues.
        let n_offsets = rig.c.fine_raster_offsets.len();
        for i in 0..n_offsets {
            let (dx, dy) = rig.c.fine_raster_offsets[i];
            let target = rig.c.ik_target_with_pitch(
                (contact_xy.0 + dx, contact_xy.1 + dy),
                rig.c.scan_z,
                super::SCAN_WRIST_PITCH,
            );
            publish_encoders(&rig, [target.0, target.1, target.2, target.3]);
            publish_ee_xy(&rig, contact_xy);
            publish_pressure(&rig, 0.6);
            rig.c.step(Time::ZERO).unwrap();
        }
        // One more tick to drain the raster-exhausted check →
        // LiftAndRetract.
        publish_pressure(&rig, 0.0);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(
            rig.c.state(),
            SearchAndPlaceState::LiftAndRetract
        ));
        // Retract completes: report joints at the retracted scan-pose
        // target → next tick lands in RotateWristForGrasp. The fine
        // centroid set rig.c.contact_xy to (something close to)
        // contact_xy; retracted_scan_target reads contact_xy via
        // arg, so we pass it explicitly here.
        let retracted = rig.c.retracted_scan_target(contact_xy);
        publish_encoders(&rig, [retracted.0, retracted.1, retracted.2, retracted.3]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(
            rig.c.state(),
            SearchAndPlaceState::RotateWristForGrasp
        ));
        // Drive RotateWristForGrasp to completion: report joints at
        // the rotated target and tick past ROTATE_WRIST_TICKS →
        // ApproachOverContact.
        let rotated = (
            retracted.0,
            retracted.1,
            retracted.2,
            retracted.3 + super::GRASP_WRIST_PITCH,
        );
        publish_encoders(&rig, [rotated.0, rotated.1, rotated.2, rotated.3]);
        for _ in 0..super::ROTATE_WRIST_TICKS + 2 {
            rig.c.step(Time::ZERO).unwrap();
        }
        assert!(matches!(
            rig.c.state(),
            SearchAndPlaceState::ApproachOverContact
        ));
        // Approach completes: report joints at the grasp-pose
        // approach target over contact_xy → DescendToContact.
        let approach = rig
            .c
            .ik_target_for(contact_xy, rig.c.scan_z + super::APPROACH_DZ);
        publish_encoders(&rig, [approach.0, approach.1, approach.2, approach.3]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(
            rig.c.state(),
            SearchAndPlaceState::DescendToContact
        ));
        rig
    }

    #[test]
    fn descend_transitions_to_close_when_joints_converge() {
        let cxy = (0.55, 0.10);
        let mut rig = rig_at_descend(cxy);
        let target = rig.c.ik_target_for(cxy, GRASP_Z);
        // Report joints at the descend target.
        publish_encoders(&rig, [target.0, target.1, target.2, target.3]);
        publish_ee_xy(&rig, cxy);
        publish_pressure(&rig, 0.6);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(
            rig.c.state(),
            SearchAndPlaceState::CloseGripper(_)
        ));
    }

    #[test]
    fn close_transitions_to_ascend_after_hold_ticks() {
        let cxy = (0.55, 0.10);
        let mut rig = rig_at_descend(cxy);
        let target = rig.c.ik_target_for(cxy, GRASP_Z);
        publish_encoders(&rig, [target.0, target.1, target.2, target.3]);
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(
            rig.c.state(),
            SearchAndPlaceState::CloseGripper(_)
        ));
        // Hold for >= CLOSE_HOLD_TICKS more steps; encoders + pressure stay
        // unchanged (Close is a halt state).
        for _ in 0..(super::CLOSE_HOLD_TICKS + 10) {
            rig.c.step(Time::ZERO).unwrap();
        }
        assert!(matches!(
            rig.c.state(),
            SearchAndPlaceState::AscendWithBlock
        ));
    }

    #[test]
    fn ascend_transitions_to_yaw_to_bin() {
        let cxy = (0.55, 0.10);
        let mut rig = rig_at_descend(cxy);
        // March to AscendWithBlock.
        let descend_target = rig.c.ik_target_for(cxy, GRASP_Z);
        publish_encoders(
            &rig,
            [
                descend_target.0,
                descend_target.1,
                descend_target.2,
                descend_target.3,
            ],
        );
        rig.c.step(Time::ZERO).unwrap();
        for _ in 0..(super::CLOSE_HOLD_TICKS + 10) {
            rig.c.step(Time::ZERO).unwrap();
        }
        assert!(matches!(
            rig.c.state(),
            SearchAndPlaceState::AscendWithBlock
        ));
        // Now report convergence at the ascend target.
        let ascend_target = rig.c.ik_target_for(cxy, super::PARK_Z);
        publish_encoders(
            &rig,
            [
                ascend_target.0,
                ascend_target.1,
                ascend_target.2,
                ascend_target.3,
            ],
        );
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::YawToBin));
    }

    #[test]
    fn yaw_to_bin_transitions_to_open_when_yaw_and_joints_converged() {
        let cxy = (0.55, 0.10);
        let mut rig = rig_at_descend(cxy);
        // March to YawToBin.
        let descend_target = rig.c.ik_target_for(cxy, GRASP_Z);
        publish_encoders(
            &rig,
            [
                descend_target.0,
                descend_target.1,
                descend_target.2,
                descend_target.3,
            ],
        );
        rig.c.step(Time::ZERO).unwrap();
        for _ in 0..(super::CLOSE_HOLD_TICKS + 10) {
            rig.c.step(Time::ZERO).unwrap();
        }
        let ascend_target = rig.c.ik_target_for(cxy, super::PARK_Z);
        publish_encoders(
            &rig,
            [
                ascend_target.0,
                ascend_target.1,
                ascend_target.2,
                ascend_target.3,
            ],
        );
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::YawToBin));
        // Report joints at the bin target.
        let bin_target = rig.c.ik_target_for((0.0, 0.6), super::PARK_Z);
        publish_encoders(
            &rig,
            [bin_target.0, bin_target.1, bin_target.2, bin_target.3],
        );
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::OpenGripper(_)));
    }

    #[test]
    fn open_transitions_to_done() {
        let cxy = (0.55, 0.10);
        let mut rig = rig_at_descend(cxy);
        // March all the way to OpenGripper(0).
        let descend_target = rig.c.ik_target_for(cxy, GRASP_Z);
        publish_encoders(
            &rig,
            [
                descend_target.0,
                descend_target.1,
                descend_target.2,
                descend_target.3,
            ],
        );
        rig.c.step(Time::ZERO).unwrap();
        for _ in 0..(super::CLOSE_HOLD_TICKS + 10) {
            rig.c.step(Time::ZERO).unwrap();
        }
        let ascend_target = rig.c.ik_target_for(cxy, super::PARK_Z);
        publish_encoders(
            &rig,
            [
                ascend_target.0,
                ascend_target.1,
                ascend_target.2,
                ascend_target.3,
            ],
        );
        rig.c.step(Time::ZERO).unwrap();
        let bin_target = rig.c.ik_target_for((0.0, 0.6), super::PARK_Z);
        publish_encoders(
            &rig,
            [bin_target.0, bin_target.1, bin_target.2, bin_target.3],
        );
        rig.c.step(Time::ZERO).unwrap();
        assert!(matches!(rig.c.state(), SearchAndPlaceState::OpenGripper(_)));
        // Hold open for >= OPEN_HOLD_TICKS.
        for _ in 0..(super::OPEN_HOLD_TICKS + 10) {
            rig.c.step(Time::ZERO).unwrap();
        }
        assert!(matches!(rig.c.state(), SearchAndPlaceState::Done));
    }
}

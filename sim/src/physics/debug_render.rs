//! Rapier debug-render adapter: bridges `DebugRenderPipeline` (which calls
//! `draw_line` for collider outlines, joint frames, contacts, etc.) into a
//! flat `Vec<DebugLine>` that `PhysicsWorld::debug_render` can return up
//! through the snapshot layer.
//!
//! Rapier emits colors in HSLA space (per `DebugRenderStyle`), but the rest
//! of the visualization stack works in 8-bit RGBA, so the backend converts
//! HSL → RGB at capture time.

use nalgebra::Point3;
use rapier3d::math::{Point, Real};
use rapier3d::pipeline::{DebugRenderBackend, DebugRenderObject};

/// One colored line segment from Rapier's debug pipeline. Coordinates are
/// in world space (the pipeline already applies body + collider transforms),
/// `color` is RGBA in 0..=255.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DebugLine {
    pub from: Point3<f32>,
    pub to: Point3<f32>,
    pub color: [u8; 4],
}

/// Backend that captures every `draw_line` callback into a `Vec`. The vec
/// is appended-only and yielded back to the caller via `into_lines()`.
#[derive(Default)]
pub(crate) struct LineCapture {
    pub lines: Vec<DebugLine>,
}

impl LineCapture {
    pub fn into_lines(self) -> Vec<DebugLine> {
        self.lines
    }
}

impl DebugRenderBackend for LineCapture {
    fn draw_line(
        &mut self,
        _object: DebugRenderObject,
        a: Point<Real>,
        b: Point<Real>,
        color: [f32; 4],
    ) {
        self.lines.push(DebugLine {
            from: Point3::new(a.x, a.y, a.z),
            to: Point3::new(b.x, b.y, b.z),
            color: hsla_to_rgba_u8(color),
        });
    }
}

/// Convert a Rapier-style HSLA color (`[h_deg, s_01, l_01, a_01]`) to 8-bit
/// RGBA. The HSL → RGB formula is the standard one (CSS color spec).
fn hsla_to_rgba_u8(hsla: [f32; 4]) -> [u8; 4] {
    let h = hsla[0].rem_euclid(360.0);
    let s = hsla[1].clamp(0.0, 1.0);
    let l = hsla[2].clamp(0.0, 1.0);
    let a = hsla[3].clamp(0.0, 1.0);

    let c = (1.0 - (2.0 * l - 1.0).abs()) * s;
    let h_prime = h / 60.0;
    let x = c * (1.0 - (h_prime.rem_euclid(2.0) - 1.0).abs());

    let (r1, g1, b1) = match h_prime as i32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    let m = l - c / 2.0;
    [
        ((r1 + m) * 255.0).round().clamp(0.0, 255.0) as u8,
        ((g1 + m) * 255.0).round().clamp(0.0, 255.0) as u8,
        ((b1 + m) * 255.0).round().clamp(0.0, 255.0) as u8,
        (a * 255.0).round().clamp(0.0, 255.0) as u8,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hsla_red() {
        // h=0, s=1, l=0.5 → pure red.
        let rgba = hsla_to_rgba_u8([0.0, 1.0, 0.5, 1.0]);
        assert_eq!(rgba, [255, 0, 0, 255]);
    }

    #[test]
    fn hsla_green() {
        let rgba = hsla_to_rgba_u8([120.0, 1.0, 0.5, 1.0]);
        assert_eq!(rgba, [0, 255, 0, 255]);
    }

    #[test]
    fn hsla_blue() {
        let rgba = hsla_to_rgba_u8([240.0, 1.0, 0.5, 1.0]);
        assert_eq!(rgba, [0, 0, 255, 255]);
    }

    #[test]
    fn hsla_white() {
        // l=1.0 → white regardless of hue.
        let rgba = hsla_to_rgba_u8([180.0, 0.5, 1.0, 1.0]);
        assert_eq!(rgba, [255, 255, 255, 255]);
    }

    #[test]
    fn hsla_alpha_passes_through() {
        let rgba = hsla_to_rgba_u8([0.0, 1.0, 0.5, 0.5]);
        assert_eq!(rgba[3], 128);
    }
}

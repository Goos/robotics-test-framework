//! Project-wide color palette. Lives in `sim` so both `arm` (which sets
//! decoration colors) and `viz` (passive consumer via `Primitive::*.color`)
//! can use it without a dependency cycle. See
//! `.claude/plans/2026-05-12-realistic-shapes-design-final.md` §4.

use crate::primitive::Color;

pub const LINK_WHITE: Color = Color::rgba(0xF2, 0xF2, 0xF2, 0xFF);
pub const JOINT_BLACK: Color = Color::rgba(0x1A, 0x1A, 0x1A, 0xFF);
pub const TABLE_GRAY: Color = Color::rgba(0xB4, 0xAF, 0xA5, 0xFF); // warm
pub const BIN_GRAY: Color = Color::rgba(0x6E, 0x78, 0x8C, 0xFF); // cool

pub const BLOCK_PALETTE: &[Color] = &[
    Color::rgba(0xE0, 0x4A, 0x4A, 0xFF), // red
    Color::rgba(0x4A, 0xC0, 0x6A, 0xFF), // green
    Color::rgba(0x4A, 0x7A, 0xE0, 0xFF), // blue
    Color::rgba(0xE0, 0x9A, 0x4A, 0xFF), // orange
    Color::rgba(0x4A, 0xC0, 0xC0, 0xFF), // cyan
    Color::rgba(0xC0, 0x4A, 0xC0, 0xFF), // magenta
];

/// Deterministic per-object color. Wraps modulo `BLOCK_PALETTE.len()` so
/// scenarios that allocate >6 objects cycle back to red. Wrap is intentional
/// (design §7.1, S6).
pub fn block_color(object_id: u32) -> Color {
    BLOCK_PALETTE[(object_id as usize) % BLOCK_PALETTE.len()]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn block_color_is_deterministic_and_cycles() {
        assert_eq!(block_color(0), BLOCK_PALETTE[0]);
        assert_eq!(block_color(5), BLOCK_PALETTE[5]);
        assert_eq!(block_color(6), BLOCK_PALETTE[0]); // wrap
        assert_eq!(block_color(7), BLOCK_PALETTE[1]);
    }
}

//! rtf_core — populated starting in Phase 1.

pub mod clock;
pub mod port_id;
pub mod time;

pub use clock::{Clock, FakeClock};
pub use port_id::PortId;
pub use time::{Time, Duration};

#[cfg(test)]
mod tests {
    #[test]
    fn workspace_smoke() {
        assert_eq!(2 + 2, 4);
    }
}

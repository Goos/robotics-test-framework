//! rtf_core — populated starting in Phase 1.

pub mod clock;
pub mod controller;
pub mod goal;
pub mod port;
pub mod port_id;
pub mod score;
pub mod sensor_reading;
pub mod time;
pub mod world_view;

pub use clock::{Clock, FakeClock};
pub use controller::{ControlError, ControlErrorKind, Controller};
pub use goal::Goal;
pub use port::{port, PortReader, PortRx, PortTx};
pub use port_id::PortId;
pub use score::Score;
pub use sensor_reading::SensorReading;
pub use time::{Time, Duration};
pub use world_view::WorldView;

#[cfg(test)]
mod tests {
    #[test]
    fn workspace_smoke() {
        assert_eq!(2 + 2, 4);
    }
}

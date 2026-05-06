use crate::primitive::SceneSnapshot;

/// Optional per-controller event stream (gated by `controller_events` feature).
/// Per design v2 §7: zero overhead in default builds; opt-in for replay/debug.
#[cfg(feature = "controller_events")]
pub enum ControllerEvent {
    PortSend {
        port_id: rtf_core::PortId,
        t: rtf_core::Time,
        type_name: &'static str,
    },
    PortRecv {
        port_id: rtf_core::PortId,
        t: rtf_core::Time,
        type_name: &'static str,
    },
    ControllerError {
        t: rtf_core::Time,
        kind: rtf_core::ControlErrorKind,
        detail: String,
    },
}

/// Receives per-tick scene snapshots and (optionally) controller events.
/// Per design v2 §7: zero overhead unless the `controller_events` feature is enabled.
pub trait Recorder {
    fn record(&mut self, snapshot: &SceneSnapshot);

    #[cfg(feature = "controller_events")]
    fn record_event(&mut self, _event: &ControllerEvent) {}
}

/// No-op recorder for headless runs and tests. Cheaper than wrapping every
/// tick in `Option<Box<dyn Recorder>>`.
pub struct NullRecorder;

impl Recorder for NullRecorder {
    fn record(&mut self, _: &SceneSnapshot) {}

    #[cfg(feature = "controller_events")]
    fn record_event(&mut self, _: &ControllerEvent) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitive::SceneSnapshot;
    use rtf_core::time::Time;
    #[test]
    fn null_recorder_is_a_no_op() {
        let mut r = NullRecorder;
        let snap = SceneSnapshot {
            t: Time::from_nanos(0),
            items: vec![],
        };
        r.record(&snap);
    }
}

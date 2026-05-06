use std::borrow::Cow;

use crate::time::Time;

/// Closed-loop tick callback (design v2 §4). Called once per tick by the
/// harness; the implementor reads sensor ports and writes actuator commands.
pub trait Controller {
    fn step(&mut self, t: Time) -> Result<(), ControlError>;
}

/// Whether the harness should keep ticking or terminate the scenario.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ControlErrorKind {
    /// Transient — harness logs and keeps ticking.
    Recoverable,
    /// Fatal — harness aborts the scenario.
    Unrecoverable,
}

/// Error returned from `Controller::step`. Carries a `Cow<'static, str>` so
/// callers can pass either a borrowed `&'static str` literal (zero-alloc) or
/// an owned `String` for context-rich messages.
#[derive(Debug, Clone)]
pub struct ControlError {
    pub kind: ControlErrorKind,
    pub detail: Cow<'static, str>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::Time;

    struct Counter {
        ticks: u32,
    }
    impl Controller for Counter {
        fn step(&mut self, _t: Time) -> Result<(), ControlError> {
            self.ticks += 1;
            Ok(())
        }
    }

    #[test]
    fn step_increments_state() {
        let mut c = Counter { ticks: 0 };
        c.step(Time::from_nanos(0)).unwrap();
        c.step(Time::from_nanos(1)).unwrap();
        assert_eq!(c.ticks, 2);
    }

    #[test]
    fn unrecoverable_error_propagates() {
        struct Err1;
        impl Controller for Err1 {
            fn step(&mut self, _t: Time) -> Result<(), ControlError> {
                Err(ControlError {
                    kind: ControlErrorKind::Unrecoverable,
                    detail: "bad".into(),
                })
            }
        }
        assert!(Err1.step(Time::from_nanos(0)).is_err());
    }
}

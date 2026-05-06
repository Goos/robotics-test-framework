use crate::{score::Score, time::Time, world_view::WorldView};

/// A scenario goal evaluated against a `WorldView` snapshot (design v2 §4).
/// `tick` lets stateful goals (waypoint progress, integrators) update each
/// tick. `is_complete` lets the harness early-terminate. `evaluate` is the
/// only required method; the others have sensible defaults.
pub trait Goal<W: WorldView> {
    fn tick(&mut self, _t: Time, _world: &W) {}
    fn is_complete(&self, _world: &W) -> bool { false }
    fn evaluate(&self, world: &W) -> Score;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{score::Score, world_view::WorldView};
    struct W;
    impl WorldView for W {}
    struct AlwaysOne;
    impl Goal<W> for AlwaysOne {
        fn evaluate(&self, _w: &W) -> Score { Score::new(1.0) }
    }
    #[test]
    fn defaults_for_tick_and_is_complete() {
        let g = AlwaysOne;
        assert!(!g.is_complete(&W));
        assert_eq!(g.evaluate(&W).value, 1.0);
    }
}

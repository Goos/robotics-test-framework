/// Marker trait for "everything the harness reveals about world state"
/// (design v2 §4). Concrete world snapshots implement this; goals and
/// scoring functions are generic over `W: WorldView` so they don't bind
/// to a specific simulator/hardware backend.
pub trait WorldView {}

#[cfg(test)]
mod tests {
    use super::*;
    struct Empty;
    impl WorldView for Empty {}
    #[test]
    fn marker_trait_is_implementable() {
        fn takes<W: WorldView>(_: &W) {}
        takes(&Empty);
    }
}

/// Identifier for a sim entity. The kind tag distinguishes movable `Object`s
/// from immovable `Fixture`s — same numeric id is intentionally distinct
/// between kinds (design v2 §6).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EntityId {
    Object(u32),
    Fixture(u32),
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn entity_ids_distinguish_kinds() {
        assert_ne!(EntityId::Object(1), EntityId::Fixture(1));
    }
}

/// Identifier for a sim entity. The kind tag distinguishes movable `Object`s
/// from immovable `Fixture`s and visualization-only `Arm` parts — same numeric
/// id is intentionally distinct between kinds (design v2 §6).
///
/// Derived `Ord` sorts `Object(_) < Fixture(_) < Arm{..}` by variant
/// declaration order; downstream snapshot consumers may rely on this for
/// deterministic frame output.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum EntityId {
    Object(u32),
    Fixture(u32),
    /// Visualization parts owned by an arm (links, fingers, etc). `arm_id`
    /// namespaces multiple arms; `slot` distinguishes parts within one arm.
    /// Kept separate from `Object`/`Fixture` so arm visuals never collide
    /// with sim-entity ids in the rerun entity tree.
    Arm { arm_id: u32, slot: u32 },
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn entity_ids_distinguish_kinds() {
        assert_ne!(EntityId::Object(1), EntityId::Fixture(1));
    }

    #[test]
    fn arm_id_distinct_from_object_with_same_numeric_value() {
        assert_ne!(EntityId::Arm { arm_id: 0, slot: 1 }, EntityId::Object(1));
        assert_ne!(EntityId::Arm { arm_id: 0, slot: 1 }, EntityId::Fixture(1));
    }
}

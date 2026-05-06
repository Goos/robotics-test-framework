/// Stable, totally-ordered identifier for a HAL port (sensor, actuator, etc.).
/// Total ordering keeps tick-path iteration deterministic (design §10.2).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PortId(pub u32);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn port_ids_compare_and_sort() {
        assert!(PortId(1) < PortId(2));
        let mut ids = vec![PortId(3), PortId(1), PortId(2)];
        ids.sort();
        assert_eq!(ids, vec![PortId(1), PortId(2), PortId(3)]);
    }
}

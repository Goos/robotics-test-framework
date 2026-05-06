use smallvec::SmallVec;

/// Scenario evaluation result. `value` is the headline number (0..1 by
/// convention); `breakdown` carries optional named sub-scores for debugging
/// and Rerun overlays. Inline storage of 4 components avoids allocation in
/// the common case (design v2 §4).
#[derive(Debug, Clone, Default)]
pub struct Score {
    pub value: f64,
    pub breakdown: SmallVec<[(&'static str, f64); 4]>,
}

impl Score {
    pub fn new(value: f64) -> Self {
        Self { value, breakdown: SmallVec::new() }
    }

    pub fn with_component(mut self, name: &'static str, value: f64) -> Self {
        self.breakdown.push((name, value));
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn default_score_is_zero_no_breakdown() {
        let s = Score::default();
        assert_eq!(s.value, 0.0);
        assert!(s.breakdown.is_empty());
    }
    #[test]
    fn score_with_breakdown_round_trips() {
        let s = Score::new(0.85)
            .with_component("accuracy", 0.9)
            .with_component("smoothness", 0.8);
        assert_eq!(s.value, 0.85);
        assert_eq!(s.breakdown.len(), 2);
        assert_eq!(s.breakdown[0], ("accuracy", 0.9));
    }
}

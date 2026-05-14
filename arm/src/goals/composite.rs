use rtf_core::{goal::Goal, score::Score, time::Time};

use crate::world::ArmWorld;

/// Composite goal: completes when ALL inner goals complete.
/// Score is the minimum of all inner goal scores.
/// Empty list → complete with score 1.0.
pub struct All {
    goals: Vec<Box<dyn Goal<ArmWorld>>>,
}

impl All {
    pub fn new(goals: Vec<Box<dyn Goal<ArmWorld>>>) -> Self {
        Self { goals }
    }
}

impl Goal<ArmWorld> for All {
    fn tick(&mut self, t: Time, world: &ArmWorld) {
        for g in &mut self.goals {
            g.tick(t, world);
        }
    }

    fn is_complete(&self, world: &ArmWorld) -> bool {
        self.goals.iter().all(|g| g.is_complete(world))
    }

    fn evaluate(&self, world: &ArmWorld) -> Score {
        if self.goals.is_empty() {
            return Score::new(1.0);
        }
        let min = self
            .goals
            .iter()
            .map(|g| g.evaluate(world).value)
            .fold(f64::INFINITY, f64::min);
        Score::new(min)
    }
}

/// Composite goal: completes when ANY inner goal completes.
/// Score is the maximum of all inner goal scores.
/// Empty list → incomplete with score 0.0.
pub struct Any {
    goals: Vec<Box<dyn Goal<ArmWorld>>>,
}

impl Any {
    pub fn new(goals: Vec<Box<dyn Goal<ArmWorld>>>) -> Self {
        Self { goals }
    }
}

impl Goal<ArmWorld> for Any {
    fn tick(&mut self, t: Time, world: &ArmWorld) {
        for g in &mut self.goals {
            g.tick(t, world);
        }
    }

    fn is_complete(&self, world: &ArmWorld) -> bool {
        self.goals.iter().any(|g| g.is_complete(world))
    }

    fn evaluate(&self, world: &ArmWorld) -> Score {
        if self.goals.is_empty() {
            return Score::new(0.0);
        }
        let max = self
            .goals
            .iter()
            .map(|g| g.evaluate(world).value)
            .fold(f64::NEG_INFINITY, f64::max);
        Score::new(max)
    }
}

/// Composite goal: inverts the inner goal.
/// Completes when the inner goal is NOT complete.
/// Score is `1.0 - inner_score`.
pub struct Not {
    goal: Box<dyn Goal<ArmWorld>>,
}

impl Not {
    pub fn new(goal: Box<dyn Goal<ArmWorld>>) -> Self {
        Self { goal }
    }
}

impl Goal<ArmWorld> for Not {
    fn tick(&mut self, t: Time, world: &ArmWorld) {
        self.goal.tick(t, world);
    }

    fn is_complete(&self, world: &ArmWorld) -> bool {
        !self.goal.is_complete(world)
    }

    fn evaluate(&self, world: &ArmWorld) -> Score {
        Score::new(1.0 - self.goal.evaluate(world).value)
    }
}

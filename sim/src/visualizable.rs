use crate::{entity::EntityId, primitive::Primitive};

/// Object-safe trait for emitting `Primitive`s for visualization.
/// Per design v2 §7: anything that knows how to draw itself (objects, fixtures,
/// arms, mobile bases, …) implements this. The visualizer (`rtf_viz`) iterates
/// `Primitive`s only — it never imports robot-kind crates.
pub trait Visualizable {
    fn append_primitives(&self, out: &mut Vec<(EntityId, Primitive)>);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn visualizable_is_object_safe() {
        fn _it_compiles(_: &dyn Visualizable) {}
        // Body intentionally empty; success is the call type-checking.
    }
}

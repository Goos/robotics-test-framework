//! Physics integration — Rapier-backed object dynamics + arm-vs-object
//! contact (rapier-integration design §2). Rapier types are confined to
//! this module; downstream crates (`rtf_arm`, `rtf_core`, controllers,
//! viz) never name Rapier types directly.
//!
//! Gated behind `feature = "physics-rapier"`. With the feature off, this
//! module is empty — submodules and the `PhysicsWorld` re-export below are
//! cfg-gated so the default build doesn't pull in Rapier or its transitive
//! deps.

#[cfg(feature = "physics-rapier")]
pub mod debug_render;
#[cfg(feature = "physics-rapier")]
pub mod world;

#[cfg(feature = "physics-rapier")]
pub use debug_render::DebugLine;
#[cfg(feature = "physics-rapier")]
pub use world::PhysicsWorld;

#[cfg(all(test, feature = "physics-rapier"))]
mod tests {
    /// Smoke test that rapier3d is reachable when the feature is on.
    #[test]
    fn rapier_dependency_compiles() {
        let _g = rapier3d::math::Vector::<f32>::new(0.0, 0.0, -9.81);
    }
}

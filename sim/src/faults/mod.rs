//! Fault-injection wrappers (design v2 §13). Each wrapper takes any
//! `R: PortReader<T>` and itself impls `PortReader<T>`, so they compose
//! freely. Submodules land one per Phase 9 step.

//! rtf_core — populated starting in Phase 1.

pub mod time;

pub use time::{Time, Duration};

#[cfg(test)]
mod tests {
    #[test]
    fn workspace_smoke() {
        assert_eq!(2 + 2, 4);
    }
}

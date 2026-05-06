use nalgebra::Vector3;

/// Geometric primitive for sim entities. The `+Z` axis is up by convention
/// (design v2 §6); `half_height_z` returns the half-extent along that axis,
/// used to seat objects on surfaces without intersection.
///
/// Cylinders are local-+Z-aligned; `half_height_z` returns the local-axis
/// half-extent and does NOT account for the entity's pose rotation. v1
/// gravity-fall (design §5.5) assumes upright cylinders.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Shape {
    Sphere { radius: f32 },
    Aabb { half_extents: Vector3<f32> },
    Cylinder { radius: f32, half_height: f32 },
}

impl Shape {
    pub fn half_height_z(&self) -> f32 {
        match self {
            Shape::Sphere { radius } => *radius,
            Shape::Aabb { half_extents } => half_extents.z,
            Shape::Cylinder { half_height, .. } => *half_height,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    #[test]
    fn aabb_top_z() {
        let s = Shape::Aabb {
            half_extents: Vector3::new(0.05, 0.05, 0.10),
        };
        assert!((s.half_height_z() - 0.10).abs() < 1e-9);
    }
    #[test]
    fn sphere_top_z() {
        let s = Shape::Sphere { radius: 0.05 };
        assert!((s.half_height_z() - 0.05).abs() < 1e-9);
    }
    #[test]
    fn cylinder_top_z() {
        let s = Shape::Cylinder {
            radius: 0.02,
            half_height: 0.15,
        };
        assert!((s.half_height_z() - 0.15).abs() < 1e-9);
    }
}

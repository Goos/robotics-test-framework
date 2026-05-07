use nalgebra::{Isometry3, Point3, Vector3};

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

    /// Euclidean distance from `point_world` to the nearest surface point of
    /// this shape posed at `pose`. Returns 0.0 if the point is inside the
    /// shape. For `Aabb`, computes distance from the point (in shape-local
    /// frame) to the box; other variants are not yet implemented (the v1.x
    /// pressure sensor only needs Aabb).
    pub fn distance_to_surface(
        &self,
        pose: &Isometry3<f32>,
        point_world: &Point3<f32>,
    ) -> f32 {
        match self {
            Shape::Aabb { half_extents } => {
                let local = pose.inverse() * point_world;
                let nearest = Point3::new(
                    local.x.clamp(-half_extents.x, half_extents.x),
                    local.y.clamp(-half_extents.y, half_extents.y),
                    local.z.clamp(-half_extents.z, half_extents.z),
                );
                (local - nearest).norm()
            }
            Shape::Sphere { .. } | Shape::Cylinder { .. } => unimplemented!(
                "distance_to_surface for non-Aabb shapes is not yet needed"
            ),
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

    #[test]
    fn distance_to_aabb_surface_above_box_returns_height_above_top() {
        // Box at origin, half-extents (0.05, 0.05, 0.05). Point 0.05 m above
        // the top face.
        let s = Shape::Aabb { half_extents: Vector3::new(0.05, 0.05, 0.05) };
        let pose = Isometry3::translation(0.0, 0.0, 0.0);
        let p = Point3::new(0.0, 0.0, 0.10);
        let d = s.distance_to_surface(&pose, &p);
        assert!((d - 0.05).abs() < 1e-6, "expected 0.05, got {d}");
    }

    #[test]
    fn distance_to_aabb_surface_inside_box_returns_zero() {
        let s = Shape::Aabb { half_extents: Vector3::new(0.05, 0.05, 0.05) };
        let pose = Isometry3::translation(1.0, 2.0, 3.0);
        // Centre of the box is inside.
        let p = Point3::new(1.0, 2.0, 3.0);
        let d = s.distance_to_surface(&pose, &p);
        assert!(d.abs() < 1e-9, "expected 0, got {d}");
    }

    #[test]
    fn distance_to_aabb_surface_at_corner_returns_diagonal() {
        // Box centered at origin, half-extents (0.1, 0.1, 0.1). Point at
        // (0.2, 0.2, 0.2) — i.e., 0.1 outside each face on the +++ corner.
        // Distance is sqrt(0.1^2 + 0.1^2 + 0.1^2).
        let s = Shape::Aabb { half_extents: Vector3::new(0.1, 0.1, 0.1) };
        let pose = Isometry3::translation(0.0, 0.0, 0.0);
        let p = Point3::new(0.2, 0.2, 0.2);
        let d = s.distance_to_surface(&pose, &p);
        let expected = (0.03_f32).sqrt(); // sqrt(0.01 * 3)
        assert!((d - expected).abs() < 1e-6, "expected {expected}, got {d}");
    }
}

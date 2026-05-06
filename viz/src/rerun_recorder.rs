//! Rerun recorder — gated on the `rerun` feature. Maps `Primitive` variants
//! onto rerun archetypes, one entity path per `EntityId`. Steps 8.3a/8.3b
//! cover `Sphere` and `Box`; remaining variants (Capsule/Line/Label) land in
//! 8.3c–e.

use rtf_sim::{
    primitive::{Primitive, SceneSnapshot},
    recorder::Recorder,
};

pub struct RerunRecorder {
    stream: rerun::RecordingStream,
}

impl RerunRecorder {
    /// In-memory recorder useful for tests and quick smoke runs; data is
    /// buffered but never flushed to a viewer or disk. For viewer-attached
    /// or saved recordings, prefer constructors added in Step 8.4.
    pub fn in_memory(name: &str) -> Result<Self, rerun::RecordingStreamError> {
        let (stream, _storage) = rerun::RecordingStreamBuilder::new(name).memory()?;
        // Drop the storage handle — tests don't read back recorded bytes.
        Ok(Self { stream })
    }
}

impl Recorder for RerunRecorder {
    fn record(&mut self, snapshot: &SceneSnapshot) {
        self.stream.set_time_seconds(
            "sim",
            (snapshot.t.as_nanos() as f64) / 1.0e9_f64,
        );
        for (entity, prim) in &snapshot.items {
            let path = format!("{entity:?}");
            match prim {
                Primitive::Sphere { pose, radius, color: _ } => {
                    let pos = [
                        pose.translation.x,
                        pose.translation.y,
                        pose.translation.z,
                    ];
                    let _ = self.stream.log(
                        path.as_str(),
                        &rerun::archetypes::Points3D::new([pos]).with_radii([*radius]),
                    );
                }
                Primitive::Box { pose, half_extents, color: _ } => {
                    let center = [
                        pose.translation.x,
                        pose.translation.y,
                        pose.translation.z,
                    ];
                    let half_size = [half_extents.x, half_extents.y, half_extents.z];
                    let _ = self.stream.log(
                        path.as_str(),
                        &rerun::archetypes::Boxes3D::from_centers_and_half_sizes(
                            [center],
                            [half_size],
                        ),
                    );
                }
                _ => {}
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Isometry3;
    use rtf_core::time::Time;
    use rtf_sim::{entity::EntityId, primitive::{Color, Primitive, SceneSnapshot}};

    #[test]
    fn rerun_recorder_records_a_sphere() {
        let mut rec = RerunRecorder::in_memory("test").unwrap();
        rec.record(&SceneSnapshot {
            t: Time::from_nanos(0),
            items: vec![(
                EntityId::Object(1),
                Primitive::Sphere {
                    pose: Isometry3::identity(),
                    radius: 0.05,
                    color: Color::WHITE,
                },
            )],
        });
    }

    #[test]
    fn rerun_recorder_records_a_box() {
        use nalgebra::Vector3;
        let mut rec = RerunRecorder::in_memory("test").unwrap();
        rec.record(&SceneSnapshot {
            t: Time::from_nanos(0),
            items: vec![(
                EntityId::Fixture(0),
                Primitive::Box {
                    pose: Isometry3::identity(),
                    half_extents: Vector3::new(0.05, 0.05, 0.05),
                    color: Color::WHITE,
                },
            )],
        });
    }
}

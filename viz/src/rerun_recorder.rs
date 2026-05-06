//! Rerun recorder — gated on the `rerun` feature. Maps each `Primitive`
//! variant onto a rerun archetype, one entity path per `EntityId`. All five
//! variants (`Sphere`, `Box`, `Capsule`, `Line`, `Label`) are covered.

use rtf_sim::{
    primitive::{Primitive, SceneSnapshot},
    recorder::Recorder,
};

pub struct RerunRecorder {
    stream: rerun::RecordingStream,
}

impl RerunRecorder {
    /// In-memory recorder useful for tests and quick smoke runs; data is
    /// buffered but never flushed to a viewer or disk. The accompanying
    /// `MemorySinkStorage` handle is dropped, so the buffered bytes are
    /// inaccessible to callers — fine for tests that only check the recorder
    /// doesn't panic during a run.
    pub fn in_memory(name: &str) -> Result<Self, rerun::RecordingStreamError> {
        let (stream, _storage) = rerun::RecordingStreamBuilder::new(name).memory()?;
        Ok(Self { stream })
    }

    /// Save recorded ticks to an `.rrd` file at `path`. Open the file with
    /// `rerun <path>` (install the viewer once via `cargo install rerun-cli
    /// --version 0.21`).
    pub fn save_to_file(
        path: impl Into<std::path::PathBuf>,
        name: &str,
    ) -> Result<Self, rerun::RecordingStreamError> {
        let stream = rerun::RecordingStreamBuilder::new(name).save(path)?;
        Ok(Self { stream })
    }
}

impl Recorder for RerunRecorder {
    fn record(&mut self, snapshot: &SceneSnapshot) {
        self.stream
            .set_time_seconds("sim", (snapshot.t.as_nanos() as f64) / 1.0e9_f64);
        for (entity, prim) in &snapshot.items {
            let path = format!("{entity:?}");
            match prim {
                Primitive::Sphere {
                    pose,
                    radius,
                    color: _,
                } => {
                    let pos = [pose.translation.x, pose.translation.y, pose.translation.z];
                    let _ = self.stream.log(
                        path.as_str(),
                        &rerun::archetypes::Points3D::new([pos]).with_radii([*radius]),
                    );
                }
                Primitive::Box {
                    pose,
                    half_extents,
                    color: _,
                } => {
                    let center = [pose.translation.x, pose.translation.y, pose.translation.z];
                    let half_size = [half_extents.x, half_extents.y, half_extents.z];
                    let _ = self.stream.log(
                        path.as_str(),
                        &rerun::archetypes::Boxes3D::from_centers_and_half_sizes(
                            [center],
                            [half_size],
                        ),
                    );
                }
                Primitive::Capsule {
                    pose,
                    half_height,
                    radius,
                    color: _,
                } => {
                    // Fallback: render as two endpoint spheres + a connecting
                    // line (logged at a sub-path so it doesn't overwrite the
                    // points archetype). Rerun 0.21's
                    // `Capsules3D::from_endpoints_and_radii` is gated behind
                    // the `glam` feature, which the rerun crate doesn't
                    // enable by default; revisit if a future rerun version
                    // exposes endpoint construction unconditionally.
                    let dir_z = pose.rotation.transform_vector(&nalgebra::Vector3::z());
                    let p1 = pose.translation.vector + dir_z * (*half_height);
                    let p2 = pose.translation.vector - dir_z * (*half_height);
                    let _ = self.stream.log(
                        path.as_str(),
                        &rerun::archetypes::Points3D::new([[p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z]])
                            .with_radii([*radius, *radius]),
                    );
                    let line_path = format!("{path}/link");
                    let _ = self.stream.log(
                        line_path.as_str(),
                        &rerun::archetypes::LineStrips3D::new([vec![
                            [p1.x, p1.y, p1.z],
                            [p2.x, p2.y, p2.z],
                        ]]),
                    );
                }
                Primitive::Line { from, to, color: _ } => {
                    let from_arr = [from.x, from.y, from.z];
                    let to_arr = [to.x, to.y, to.z];
                    let _ = self.stream.log(
                        path.as_str(),
                        &rerun::archetypes::LineStrips3D::new([vec![from_arr, to_arr]]),
                    );
                }
                Primitive::Label {
                    pose,
                    text,
                    color: _,
                } => {
                    let pos = [pose.translation.x, pose.translation.y, pose.translation.z];
                    let _ = self.stream.log(
                        path.as_str(),
                        &rerun::archetypes::Points3D::new([pos]).with_labels([text.as_str()]),
                    );
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Isometry3;
    use rtf_core::time::Time;
    use rtf_sim::{
        entity::EntityId,
        primitive::{Color, Primitive, SceneSnapshot},
    };

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

    #[test]
    fn rerun_recorder_records_a_capsule() {
        let mut rec = RerunRecorder::in_memory("test").unwrap();
        rec.record(&SceneSnapshot {
            t: Time::from_nanos(0),
            items: vec![(
                EntityId::Object(2),
                Primitive::Capsule {
                    pose: Isometry3::identity(),
                    half_height: 0.1,
                    radius: 0.02,
                    color: Color::WHITE,
                },
            )],
        });
    }

    #[test]
    fn rerun_recorder_records_a_line() {
        use nalgebra::Point3;
        let mut rec = RerunRecorder::in_memory("test").unwrap();
        rec.record(&SceneSnapshot {
            t: Time::from_nanos(0),
            items: vec![(
                EntityId::Object(3),
                Primitive::Line {
                    from: Point3::new(0.0, 0.0, 0.0),
                    to: Point3::new(1.0, 0.0, 0.0),
                    color: Color::WHITE,
                },
            )],
        });
    }

    #[test]
    fn rerun_recorder_records_a_label() {
        let mut rec = RerunRecorder::in_memory("test").unwrap();
        rec.record(&SceneSnapshot {
            t: Time::from_nanos(0),
            items: vec![(
                EntityId::Object(4),
                Primitive::Label {
                    pose: Isometry3::identity(),
                    text: "block".to_string(),
                    color: Color::WHITE,
                },
            )],
        });
    }
}

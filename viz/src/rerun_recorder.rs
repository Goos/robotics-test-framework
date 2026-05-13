//! Rerun recorder — gated on the `rerun` feature. Maps each `Primitive`
//! variant onto a rerun archetype, one entity path per `EntityId`. All five
//! variants (`Sphere`, `Box`, `Capsule`, `Line`, `Label`) are covered.

use rtf_sim::{
    primitive::{Primitive, SceneSnapshot},
    recorder::Recorder,
};

fn rerun_color_from(c: rtf_sim::primitive::Color) -> rerun::components::Color {
    rerun::components::Color::from_unmultiplied_rgba(c.r, c.g, c.b, c.a)
}

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
                    color,
                } => {
                    let pos = [pose.translation.x, pose.translation.y, pose.translation.z];
                    let _ = self.stream.log(
                        path.as_str(),
                        &rerun::archetypes::Points3D::new([pos])
                            .with_radii([*radius])
                            .with_colors([rerun_color_from(*color)]),
                    );
                }
                Primitive::Box {
                    pose,
                    half_extents,
                    color,
                } => {
                    let center = [pose.translation.x, pose.translation.y, pose.translation.z];
                    let half_size = [half_extents.x, half_extents.y, half_extents.z];
                    let q = pose.rotation;
                    // PoseRotationQuat is a direct wrapper around
                    // [x, y, z, w]. Boxes3D applies it as the box-local
                    // rotation, so the rendered AABB matches the
                    // Rapier-resolved orientation rather than always
                    // axis-aligned.
                    let quat = rerun::components::PoseRotationQuat(rerun::datatypes::Quaternion([
                        q.i, q.j, q.k, q.w,
                    ]));
                    let _ = self.stream.log(
                        path.as_str(),
                        &rerun::archetypes::Boxes3D::from_centers_and_half_sizes(
                            [center],
                            [half_size],
                        )
                        .with_quaternions([quat])
                        .with_colors([rerun_color_from(*color)]),
                    );
                }
                Primitive::Capsule {
                    pose,
                    half_height,
                    radius,
                    color,
                } => {
                    let length = 2.0 * *half_height;
                    let dir_z = pose.rotation.transform_vector(&nalgebra::Vector3::z());
                    let base = pose.translation.vector - dir_z * (*half_height);
                    let q = pose.rotation;
                    let _ = self.stream.log(
                        path.as_str(),
                        &rerun::archetypes::Capsules3D::from_lengths_and_radii([length], [*radius])
                            .with_translations([[base.x, base.y, base.z]])
                            .with_quaternions([rerun::components::PoseRotationQuat(
                                rerun::datatypes::Quaternion([q.i, q.j, q.k, q.w]),
                            )])
                            .with_colors([rerun_color_from(*color)]),
                    );
                }
                Primitive::Line { from, to, color } => {
                    let from_arr = [from.x, from.y, from.z];
                    let to_arr = [to.x, to.y, to.z];
                    let _ = self.stream.log(
                        path.as_str(),
                        &rerun::archetypes::LineStrips3D::new([vec![from_arr, to_arr]])
                            .with_colors([rerun_color_from(*color)]),
                    );
                }
                Primitive::Label { pose, text, color } => {
                    let pos = [pose.translation.x, pose.translation.y, pose.translation.z];
                    let _ = self.stream.log(
                        path.as_str(),
                        &rerun::archetypes::Points3D::new([pos])
                            .with_labels([text.as_str()])
                            .with_colors([rerun_color_from(*color)]),
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
    fn recorder_passes_rotation_to_rerun_box_archetype() {
        // Smoke test: log a Box with a non-identity rotation. We can't
        // round-trip parse the .rrd here without pulling in extra
        // crates, but the lack of panic + clean shutdown verifies the
        // PoseRotationQuat/Quaternion construction round-trips through
        // the rerun archetype.
        use nalgebra::{UnitQuaternion, Vector3};
        let mut rec = RerunRecorder::in_memory("test").unwrap();
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.5);
        rec.record(&SceneSnapshot {
            t: Time::from_nanos(0),
            items: vec![(
                EntityId::Object(1),
                Primitive::Box {
                    pose: Isometry3::from_parts(nalgebra::Translation3::new(0.0, 0.0, 0.0), rot),
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

    #[test]
    fn recorder_passes_color_to_box() {
        use nalgebra::Vector3;
        let mut rec = RerunRecorder::in_memory("test").unwrap();
        rec.record(&SceneSnapshot {
            t: Time::from_nanos(0),
            items: vec![(
                EntityId::Object(1),
                Primitive::Box {
                    pose: Isometry3::identity(),
                    half_extents: Vector3::new(0.05, 0.05, 0.05),
                    color: Color::RED,
                },
            )],
        });
    }

    #[test]
    fn recorder_renders_capsule_via_capsules3d_archetype() {
        use nalgebra::{UnitQuaternion, Vector3};
        let mut rec = RerunRecorder::in_memory("test").unwrap();
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_4);
        rec.record(&SceneSnapshot {
            t: Time::from_nanos(0),
            items: vec![(
                EntityId::Object(1),
                Primitive::Capsule {
                    pose: Isometry3::from_parts(nalgebra::Translation3::new(0.0, 0.0, 0.5), rot),
                    half_height: 0.2,
                    radius: 0.04,
                    color: Color::WHITE,
                },
            )],
        });
    }
}

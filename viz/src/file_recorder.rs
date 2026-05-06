//! `FileRecorder` — writes one JSON-encoded `SceneSnapshot` per line. Cheap
//! offline format usable for diffing, replay, and consumption by external
//! tooling (Python/JS) without a rerun dep.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;

use rtf_sim::{primitive::SceneSnapshot, recorder::Recorder};

pub struct FileRecorder {
    writer: BufWriter<File>,
}

impl FileRecorder {
    pub fn create<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        Ok(Self { writer: BufWriter::new(File::create(path)?) })
    }
}

impl Recorder for FileRecorder {
    fn record(&mut self, snapshot: &SceneSnapshot) {
        // Best-effort: silently drop write errors so a recording failure
        // can't take down a sim run. Recording is observability, not
        // load-bearing — the tradeoff matches `NullRecorder`'s behavior.
        let _ = serde_json::to_writer(&mut self.writer, snapshot);
        let _ = writeln!(self.writer);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rtf_core::time::Time;
    use rtf_sim::primitive::SceneSnapshot;
    use std::fs::File;
    use std::io::{BufRead, BufReader};

    #[test]
    fn appends_one_line_per_snapshot() {
        let path = std::env::temp_dir().join("viz_test_appends_one_line.jsonl");
        let _ = std::fs::remove_file(&path);
        let mut rec = FileRecorder::create(&path).unwrap();
        rec.record(&SceneSnapshot { t: Time::from_nanos(0), items: vec![] });
        rec.record(&SceneSnapshot { t: Time::from_millis(1), items: vec![] });
        drop(rec);
        let lines: Vec<_> = BufReader::new(File::open(&path).unwrap()).lines().collect();
        assert_eq!(lines.len(), 2);
    }
}

//! Protocol-layer message capture (issue #31).
//!
//! Distinct from the FMSL event log (`replay.rs`): that log records
//! *inputs + state samples* for physics replay; this one records the raw
//! `AgentMsg`/`ServerMsg` envelopes each agent sent/received, stamped with
//! the physics tick. Replaying the file against a driver or test harness
//! reproduces the exact protocol traffic for debugging — it cannot
//! rebuild the world (use `--replay` for that).
//!
//! Format: one JSON object per line (JSON-lines):
//! ```text
//! {"tick":1234,"robot":0,"dir":"in"|"out","msg":{...AgentMsg|ServerMsg...}}
//! ```
//! `dir` is from the orchestrator's perspective: `in` = agent -> server
//! (Hello/Actuator), `out` = server -> agent (Welcome/Error/Sensor).
//! Messages that fail to parse as protocol types are logged as
//! `{"raw": "<text>"}` so nothing on the wire is lost.
use anyhow::Result;
use serde::Serialize;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;

/// Direction of a logged envelope, from the orchestrator's perspective.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Dir {
    /// agent -> orchestrator (Hello, Actuator)
    In,
    /// orchestrator -> agent (Welcome, Error, Sensor)
    Out,
}

#[derive(Serialize)]
struct Envelope<'a, T: Serialize + ?Sized> {
    tick: u64,
    robot: usize,
    dir: Dir,
    msg: &'a T,
}

/// Thread-safe append-only writer shared by all agent connections.
/// Tick is the world's current physics tick: inbound messages are stamped
/// with the most recent tick observed outbound (sensor frames carry their
/// own tick, so `note_tick` only matters before the first sensor frame).
pub struct ProtoLog {
    w: Mutex<BufWriter<File>>,
    tick: AtomicU64,
}

impl ProtoLog {
    /// Create (or truncate) the log file. A `{"header":{...}}` line is
    /// written first so a reader can identify the format/version.
    pub fn create(path: &Path) -> Result<Self> {
        let mut w = BufWriter::new(File::create(path)?);
        writeln!(
            w,
            "{}",
            serde_json::json!({
                "header": {
                    "format": "proto-log",
                    "format_version": 1,
                    "protocol_version": crate::PROTOCOL_VERSION,
                }
            })
        )?;
        Ok(ProtoLog {
            w: Mutex::new(w),
            tick: AtomicU64::new(0),
        })
    }

    /// Advance the observed world tick (monotonic max). Called from the
    /// physics loop / sensor broadcast so inbound envelopes get stamped
    /// correctly.
    pub fn note_tick(&self, tick: u64) {
        self.tick.fetch_max(tick, Ordering::Relaxed);
    }

    /// Current observed tick (for tests / inbound stamping).
    pub fn tick(&self) -> u64 {
        self.tick.load(Ordering::Relaxed)
    }

    /// Append one envelope at the log's current tick.
    pub fn log<T: Serialize + ?Sized>(&self, robot: usize, dir: Dir, msg: &T) {
        self.log_at(self.tick(), robot, dir, msg);
    }

    /// Append one envelope at an explicit tick (sensor frames stamp with
    /// `frame.tick` rather than the loop's latest).
    pub fn log_at<T: Serialize + ?Sized>(&self, tick: u64, robot: usize, dir: Dir, msg: &T) {
        let env = Envelope {
            tick,
            robot,
            dir,
            msg,
        };
        if let Ok(line) = serde_json::to_string(&env) {
            if let Ok(mut w) = self.w.lock() {
                let _ = writeln!(w, "{line}");
            }
        }
    }

    /// Append a raw text payload that didn't parse as a protocol message.
    pub fn log_raw(&self, robot: usize, dir: Dir, text: &str) {
        self.log(robot, dir, &serde_json::json!({ "raw": text }));
    }

    pub fn flush(&self) {
        if let Ok(mut w) = self.w.lock() {
            let _ = w.flush();
        }
    }
}

/// One decoded line for readers/tests.
#[derive(Debug, Clone)]
pub struct ProtoEntry {
    pub tick: u64,
    pub robot: usize,
    pub dir: Dir,
    pub msg: serde_json::Value,
}

/// Read a proto-log back into envelopes (skips the header line and any
/// malformed rows). For tests, debug tools, and protocol replay.
pub fn read_log(path: &Path) -> Result<Vec<ProtoEntry>> {
    let f = File::open(path)?;
    let mut out = Vec::new();
    for line in BufReader::new(f).lines() {
        let line = line?;
        let v: serde_json::Value = match serde_json::from_str(&line) {
            Ok(v) => v,
            Err(_) => continue,
        };
        if v.get("header").is_some() {
            continue;
        }
        let (Some(tick), Some(robot), Some(dir), Some(msg)) = (
            v.get("tick").and_then(|t| t.as_u64()),
            v.get("robot").and_then(|r| r.as_u64()),
            v.get("dir")
                .and_then(|d| serde_json::from_value::<Dir>(d.clone()).ok()),
            v.get("msg").cloned(),
        ) else {
            continue;
        };
        out.push(ProtoEntry {
            tick,
            robot: robot as usize,
            dir,
            msg,
        });
    }
    Ok(out)
}

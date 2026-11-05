//! Server configuration (PLAN A3/A4): time mode, seed, archetype files,
//! record/replay paths. Resolves CLI inputs into the `RobotSpec` table the
//! world is built from, and the config hash stored in replay headers.
use crate::archetype::Archetype;
use crate::world::pieces::PieceTable;
use crate::world::chassis::{self, RobotSpec, NUM_ROBOTS};
use crate::world::field::FieldSpec;
use clap::ValueEnum;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::path::PathBuf;

/// Physics-loop pacing (arch doc §2.1). The step itself is always driven by
/// the tick counter; modes differ only in how the loop is gated.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum TimeMode {
    /// Wall-clock locked at PHYSICS_HZ (tokio interval). Default.
    Realtime,
    /// No sleeps; after each agent tick, wait until every WS-connected
    /// agent has produced a fresh command (WPILib-style tick barrier,
    /// 20 ms stall timeout).
    Lockstep,
    /// No sleeps, no barriers — run as fast as the CPU allows (RL rollouts).
    Fast,
}

#[derive(Debug, Clone)]
pub struct SimConfig {
    pub time_mode: TimeMode,
    pub seed: u64,
    /// NT4 ports to bridge, indexed by robot id
    pub nt_ports: Vec<u16>,
    /// archetype TOML paths, indexed by robot id
    pub archetypes: Vec<PathBuf>,
    /// field layout TOML (issue #14); `None` -> built-in default layout
    pub field: Option<PathBuf>,
    /// game-piece spawn/scoring table TOML (issue #3); None = empty field
    pub pieces: Option<PathBuf>,
    /// game/match TOML (`[match]` section: phase durations, endgame rules,
    /// issue #10); `None` -> built-in REBUILT-style default
    pub game: Option<PathBuf>,
    /// write an event log here (record mode)
    pub record: Option<PathBuf>,
    /// re-run this log headless (replay mode)
    pub replay: Option<PathBuf>,
    /// diff replayed trajectory against this golden log
    pub diff: Option<PathBuf>,
    /// tolerance for --diff (0 = bitwise)
    pub diff_tol: f32,
}

impl SimConfig {
    /// Resolve robot specs: default lineup overridden per-slot by any
    /// loaded archetype files.
    pub fn robot_specs(&self) -> Vec<RobotSpec> {
        let mut specs = chassis::default_specs();
        specs.resize(NUM_ROBOTS.max(self.archetypes.len()), *specs.first().unwrap());
        for (i, path) in self.archetypes.iter().enumerate() {
            if i >= specs.len() {
                break;
            }
            match Archetype::load(path) {
                Ok(a) => {
                    println!("robot {} archetype {:?} ({})", i, a.name, path.display());
                    let x = specs[i].spawn[0];
                    specs[i] = RobotSpec::from_archetype(&a, x);
                }
                Err(e) => eprintln!("failed to load archetype {}: {e:#}", path.display()),
            }
        }
        specs
    }

    /// Resolve the field layout: `--field` TOML if given, else the
    /// historical default. Invalid files fail loudly (schema-checked).
    pub fn field_spec(&self) -> anyhow::Result<FieldSpec> {
        match &self.field {
            Some(p) => FieldSpec::load(p),
            None => Ok(FieldSpec::default()),
        }
    }

    /// Hash of everything that affects the trajectory besides inputs and
    /// seed: archetype file contents + resolved specs. Stored in replay
    /// headers so a log can't be silently replayed against a different
    /// robot/field setup.
    pub fn config_hash(specs: &[RobotSpec]) -> u64 {
        let mut h = DefaultHasher::new();
        for s in specs {
            s.mass.to_bits().hash(&mut h);
            for v in s.half_extents {
                v.to_bits().hash(&mut h);
            }
            for v in s.spawn {
                v.to_bits().hash(&mut h);
            }
            s.sensors.enabled.hash(&mut h);
            for v in [
                s.sensors.gyro_std,
                s.sensors.pose_std,
                s.sensors.vel_std,
                s.sensors.gyro_bias,
                s.sensors.pose_bias,
                s.sensors.vel_bias,
                s.sensors.vision_latency_s,
                s.sensors.vision_hz,
                s.sensors.vision_dropout,
                s.sensors.vision_std_min,
                s.sensors.vision_std_max,
            ] {
                v.to_bits().hash(&mut h);
            }
            if let Some(i) = s.intake {
                for v in [i.x_min, i.x_max, i.z_min, i.z_max, i.y_min, i.y_max] {
                    v.to_bits().hash(&mut h);
                }
            }
        }
        h.finish()
    }

    /// Load the piece spawn/scoring table (or an empty one when unset).
    pub fn piece_table(&self) -> PieceTable {
        match &self.pieces {
            Some(p) => match PieceTable::load(p) {
                Ok(t) => {
                    println!(
                        "piece table {:?}: {} pieces, {} hubs",
                        p,
                        t.piece_count(),
                        t.hubs.len()
                    );
                    t
                }
                Err(e) => {
                    eprintln!("failed to load piece table {}: {e:#}", p.display());
                    PieceTable::default()
                }
            },
            None => PieceTable::default(),
        }
    }

    /// Resolve the match timing/endgame rules: `--game` TOML if given,
    /// else the built-in default (issue #10). The match clock is a pure
    /// function of tick; this config is hashed into replay headers so a
    /// log can't be replayed against different match rules silently.
    pub fn match_config(&self) -> anyhow::Result<crate::r#match::MatchConfig> {
        match &self.game {
            Some(p) => crate::r#match::MatchConfig::load(p),
            None => Ok(crate::r#match::MatchConfig::default()),
        }
    }

    /// Fold the match config into the config hash (issue #10) — phase
    /// durations change when commands are gated, so they affect the
    /// trajectory exactly like specs do.
    pub fn hash_match(&self, h: u64, m: &crate::r#match::MatchConfig) -> u64 {
        let mut hasher = DefaultHasher::new();
        hasher.write_u64(h);
        hasher.write(&serde_json::to_vec(m).unwrap_or_default());
        if let Some(p) = &self.game {
            if let Ok(bytes) = std::fs::read(p) {
                hasher.write(&bytes);
            }
        }
        hasher.finish()
    }

    /// Hash raw archetype file contents too, so config edits that don't
    /// change the resolved spec still invalidate logs.
    pub fn hash_archetype_files(&self, mut h: u64) -> u64 {
        let mut hasher = DefaultHasher::new();
        hasher.write_u64(h);
        for p in self.archetypes.iter().chain(self.pieces.iter()) {
            if let Ok(bytes) = std::fs::read(p) {
                hasher.write(&bytes);
            }
        }
        h = hasher.finish();
        h
    }

    /// Fold the field spec into the config hash so replays pin the field
    /// layout (issue #14). Hashes the resolved spec (not just file bytes)
    /// so the default layout is pinned too.
    pub fn hash_field(&self, h: u64, field: &FieldSpec) -> u64 {
        let mut hasher = DefaultHasher::new();
        hasher.write_u64(h);
        hasher.write(&serde_json::to_vec(field).unwrap_or_default());
        if let Some(p) = &self.field {
            if let Ok(bytes) = std::fs::read(p) {
                hasher.write(&bytes);
            }
        }
        hasher.finish()
    }
}

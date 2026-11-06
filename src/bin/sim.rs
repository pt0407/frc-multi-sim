//! Headless sim-as-a-binary (issue #30): run `world::World` for N ticks
//! with no sockets, no wall-clock, and emit one JSON state line per robot
//! per agent tick on stdout — the batch interface for tests and RL
//! rollouts that don't need a live agent connection.
//!
//! Command input (pick one):
//!   --script <file>    JSON-lines of {"tick":N,"robot":R,"cmd":<ActuatorFrame|null>}
//!                      applied last-wins at each tick (alias: --scenario)
//!   --replay <file>    an FMSL event log (`server --record`); its spec
//!                      table, seed, and command records drive the run
//!   (neither)          all robots idle — still useful as a smoke test
//!
//! Output lines look like:
//!   {"tick":400,"robot":0,"sensor":{...SensorFrame...}}
//! Build/diagnostic chatter may appear on other lines; consumers should
//! parse only lines that decode to an object with a "sensor" key.
use anyhow::{Context, Result};
use clap::Parser;
use frc_sim_m0::config::{SimConfig, TimeMode};
use frc_sim_m0::replay::{self, Record};
use frc_sim_m0::world::{World, AGENT_EVERY};
use frc_sim_m0::ActuatorFrame;
use serde::Deserialize;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;

#[derive(Parser)]
struct Args {
    /// Physics ticks to run (400 Hz; default 400 = 1 s)
    #[arg(long, default_value_t = 400)]
    ticks: u64,
    /// Seed for the world's SimRng
    #[arg(long, default_value_t = 0)]
    seed: u64,
    /// Archetype TOML per robot slot (repeatable, indexed by robot id)
    #[arg(long)]
    archetype: Vec<PathBuf>,
    /// Field layout TOML (defaults to fields/rebuilt.toml when present)
    #[arg(long)]
    field: Option<PathBuf>,
    /// Game-piece spawn/scoring table TOML
    #[arg(long)]
    pieces: Option<PathBuf>,
    /// Game/match TOML with a `[match]` section (defaults to
    /// games/rebuilt.toml when present)
    #[arg(long)]
    game: Option<PathBuf>,
    /// Scripted command input: JSON-lines of
    /// {"tick":N,"robot":R,"cmd":<ActuatorFrame|null>}
    #[arg(long, alias = "scenario")]
    script: Option<PathBuf>,
    /// Run the commands embedded in this FMSL event log (header specs and
    /// seed override --archetype/--seed/--field/--pieces)
    #[arg(long)]
    replay: Option<PathBuf>,
}

/// One line of a --script file.
#[derive(Debug, Deserialize)]
struct ScriptEvent {
    tick: u64,
    robot: u32,
    cmd: Option<ActuatorFrame>,
}

fn load_script(path: &PathBuf) -> Result<Vec<ScriptEvent>> {
    let f = std::fs::File::open(path)
        .with_context(|| format!("open script {}", path.display()))?;
    let mut events = Vec::new();
    for (i, line) in BufReader::new(f).lines().enumerate() {
        let line = line?;
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        events.push(
            serde_json::from_str::<ScriptEvent>(line)
                .with_context(|| format!("{}:{}", path.display(), i + 1))?,
        );
    }
    events.sort_by_key(|e| e.tick);
    Ok(events)
}

fn emit(tick: u64, robot: usize, frame: &frc_sim_m0::SensorFrame) {
    println!(
        "{}",
        serde_json::json!({"tick": tick, "robot": robot, "sensor": frame})
    );
}

fn main() -> Result<()> {
    let args = Args::parse();

    // --replay: the log's header is the scenario (specs + seed + pieces);
    // its command records are the script. Matches `replay::replay`'s
    // world construction exactly.
    let (mut world, events): (World, Vec<(u64, u32, Option<ActuatorFrame>)>) =
        if let Some(path) = &args.replay {
            let (header, records) = replay::read_log(path)
                .with_context(|| format!("read replay log {}", path.display()))?;
            let empty = frc_sim_m0::world::pieces::PieceTable::default();
            let world = World::with_config(
                &header.specs,
                header.seed,
                frc_sim_m0::world::field::FieldSpec::default(),
                header.pieces.as_ref().unwrap_or(&empty),
            );
            let events = records
                .iter()
                .filter_map(|r| match r {
                    Record::Cmd { tick, robot, cmd } => {
                        Some((*tick, *robot, cmd.clone()))
                    }
                    _ => None,
                })
                .collect();
            (world, events)
        } else {
            // Reuse SimConfig resolution so flags behave identically to
            // `server` (issue #30: no third world-config copy).
            let cfg = SimConfig {
                time_mode: TimeMode::Fast,
                seed: args.seed,
                nt_ports: vec![],
                archetypes: args.archetype.clone(),
                num_robots: None,
                field: args.field.clone().or_else(|| {
                    let p = PathBuf::from("fields/rebuilt.toml");
                    p.exists().then_some(p)
                }),
                pieces: args.pieces.clone(),
                game: args.game.clone().or_else(|| {
                    let p = PathBuf::from("games/rebuilt.toml");
                    p.exists().then_some(p)
                }),
                record: None,
                replay: None,
                diff: None,
                diff_tol: 0.0,
            };
            let specs = cfg.robot_specs();
            let field = cfg.field_spec()?;
            let pieces = cfg.piece_table();
            let mut world =
                World::with_config(&specs, cfg.seed, field, &pieces);
            world.match_clock =
                frc_sim_m0::r#match::MatchClock::new(cfg.match_config()?);
            let events = match &args.script {
                Some(p) => load_script(p)?
                    .into_iter()
                    .map(|e| (e.tick, e.robot, e.cmd))
                    .collect(),
                None => Vec::new(),
            };
            (world, events)
        };

    let n = world.num_robots();
    let mut cmds: Vec<Option<ActuatorFrame>> = vec![None; n];
    let mut ev = 0;
    let out = std::io::stdout();
    let _ = &out; // lines go through println!
    for tick in 0..args.ticks {
        while ev < events.len() && events[ev].0 <= tick {
            let (_, robot, cmd) = &events[ev];
            if (*robot as usize) < n {
                cmds[*robot as usize] = cmd.clone();
            }
            ev += 1;
        }
        world.step(&cmds);
        if tick % AGENT_EVERY as u64 == 0 {
            for r in 0..n {
                let frame = world.sensor_frame(r, tick);
                emit(tick, r, &frame);
            }
        }
    }
    Ok(())
}

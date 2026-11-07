//! Match analyzer (issue: replay/metrics): runs the world in-process with
//! per-robot strategy interpreters and reports where cycle time goes —
//! distance, captures, shots, time stalled, time in each field zone.
//!
//! Usage:
//!   analyze --archetype a.toml --archetype b.toml ... \
//!           --strategy s0.json --strategy s1.json ... --ticks 64000
use anyhow::{Context, Result};
use clap::Parser;
use frc_sim_m0::config::{SimConfig, TimeMode};
use frc_sim_m0::world::{World, AGENT_EVERY};
use frc_sim_m0::{ActuatorFrame, MechCommand, SensorFrame, ShotCommand};
use serde::Deserialize;
use std::path::PathBuf;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    archetype: Vec<PathBuf>,
    #[arg(long)]
    strategy: Vec<PathBuf>,
    #[arg(long)]
    field: Option<PathBuf>,
    #[arg(long)]
    pieces: Option<PathBuf>,
    #[arg(long)]
    game: Option<PathBuf>,
    #[arg(long, default_value_t = 0)]
    seed: u64,
    #[arg(long)]
    num_robots: Option<usize>,
    /// physics ticks (400 Hz; 64000 = 160 s ≈ full match)
    #[arg(long, default_value_t = 64000)]
    ticks: u64,
    #[arg(long, default_value_t = 1200.0)]
    max_force: f32,
    #[arg(long, default_value_t = 0.3)]
    arrive: f32,
}

#[derive(Debug, Deserialize)]
struct Strategy {
    actions: Vec<Action>,
    #[serde(default)]
    loop_actions: bool,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
enum Action {
    Goto { x: f32, z: f32 },
    Intake { active: bool },
    Wait { seconds: f32 },
    Shoot(ShotParams),
    Mech { joint: usize, target: f32 },
    Hold,
}

#[derive(Debug, Deserialize)]
struct ShotParams {
    launch_velocity: f32,
    hood_angle: f32,
    turret_yaw: f32,
    launch_height: f32,
}

struct RobotBrain {
    strat: Strategy,
    idx: usize,
    intake_on: bool,
    mech_last: Option<MechCommand>,
    wait_until: Option<u64>,
    done: bool,
}

impl RobotBrain {
    fn load(path: &PathBuf) -> Result<Self> {
        let strat: Strategy = serde_json::from_str(
            &std::fs::read_to_string(path)
                .with_context(|| format!("read {}", path.display()))?,
        )?;
        Ok(Self { strat, idx: 0, intake_on: false, mech_last: None, wait_until: None, done: false })
    }

    /// Mirror of strategy_agent's per-tick action interpreter; returns the
    /// actuator frame to apply this agent tick.
    fn tick(&mut self, s: &SensorFrame, max_force: f32, arrive: f32) -> ActuatorFrame {
        let mut shoot = None;
        if !self.done && s.match_state.enabled() {
            match self.strat.actions.get(self.idx) {
                Some(Action::Goto { x, z }) => {
                    let d = ((x - s.position[0]).powi(2) + (z - s.position[2]).powi(2)).sqrt();
                    if d < arrive { self.idx += 1; }
                }
                Some(Action::Intake { active }) => {
                    self.intake_on = *active;
                    self.idx += 1;
                }
                Some(Action::Wait { seconds }) => {
                    if self.wait_until.is_none() {
                        self.wait_until = Some(s.tick + (seconds * 50.0) as u64);
                    } else if s.tick >= self.wait_until.unwrap() {
                        self.wait_until = None;
                        self.idx += 1;
                    }
                }
                Some(Action::Mech { joint, target }) => {
                    let m = MechCommand { joint: *joint, target: *target };
                    self.mech_last = Some(m);
                    self.idx += 1;
                }
                Some(Action::Shoot(p)) => {
                    if std::env::var("DEBUG_SHOOT").is_ok() {
                        eprintln!("tick {} robot shoot idx {}", s.tick, self.idx);
                    }
                    shoot = Some(ShotCommand {
                        launch_velocity: p.launch_velocity,
                        hood_angle: p.hood_angle,
                        turret_yaw: p.turret_yaw,
                        launch_height: p.launch_height,
                    });
                    self.idx += 1;
                }
                Some(Action::Hold) | None => {
                    self.done = !self.strat.loop_actions;
                    if self.strat.loop_actions { self.idx = 0; }
                }
            }
        }

        let (fx, fz) = match self.strat.actions.get(self.idx) {
            Some(Action::Goto { x, z }) if !self.done => {
                let ex = x - s.position[0];
                let ez = z - s.position[2];
                let mut fx = 300.0 * ex - 60.0 * s.linvel[0];
                let mut fz = 300.0 * ez - 60.0 * s.linvel[2];
                let m = (fx * fx + fz * fz).sqrt();
                if m > max_force {
                    fx *= max_force / m;
                    fz *= max_force / m;
                }
                (fx, fz)
            }
            _ => (-60.0 * s.linvel[0], -60.0 * s.linvel[2]),
        };
        ActuatorFrame {
            force: [fx, fz],
            torque: 0.0,
            modules: None,
            intake_active: self.intake_on,
            shoot,
            mech: self.mech_last,
        }
    }
}

fn zone(x: f32, z: f32) -> &'static str {
    // field layout: hubs at |x|~3.5, neutral pile |x|<2.6, spawn |x|>4
    if x.abs() > 3.6 { "spawn-side" }
    else if x.abs() < 2.6 && z.abs() < 2.0 { "pile" }
    else if x.abs() > 2.6 { "hub-approach" }
    else { "midfield" }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let cfg = SimConfig {
        time_mode: TimeMode::Fast,
        seed: args.seed,
        nt_ports: vec![],
        archetypes: args.archetype.clone(),
        num_robots: args.num_robots,
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
    let mut world = World::with_config(&specs, cfg.seed, field, &pieces);
    world.match_clock = frc_sim_m0::r#match::MatchClock::new(cfg.match_config()?);
    // same as server's start_match op: skip the staging-disabled window
    world.match_clock.config.disabled_s = 0.0;
    world.start_match();
    world.apply_preload();

    let n = world.num_robots();
    let mut brains: Vec<Option<RobotBrain>> = (0..n)
        .map(|i| {
            args.strategy.get(i).map(|p| RobotBrain::load(p)).transpose()
        })
        .collect::<Result<Vec<_>>>()?
        .into_iter()
        .collect();

    // per-robot accumulators
    let mut dist = vec![0f32; n];
    let mut prev = vec![[0f32; 3]; n];
    let mut held_prev = world.held.clone();
    let mut captures = vec![0u32; n];
    let mut shots = vec![0u32; n];
    let mut stalled_ticks = vec![0u64; n];
    let mut zone_ticks: Vec<std::collections::HashMap<&'static str, u64>> =
        (0..n).map(|_| Default::default()).collect();
    let mut last_score = world.scores;

    let mut cmds: Vec<Option<ActuatorFrame>> = vec![None; n];
    for tick in 0..args.ticks {
        if tick % AGENT_EVERY as u64 == 0 {
            for r in 0..n {
                let frame = world.sensor_frame(r, tick);
                if let Some(b) = brains[r].as_mut() {
                    cmds[r] = Some(b.tick(&frame, args.max_force, args.arrive));
                } else {
                    cmds[r] = None;
                }
                // metrics at agent rate (50 Hz is plenty for zone/stall)
                let p = frame.position;
                dist[r] += ((p[0]-prev[r][0]).powi(2)+(p[2]-prev[r][2]).powi(2)).sqrt();
                prev[r] = p;
                let sp = (frame.linvel[0].powi(2)+frame.linvel[2].powi(2)).sqrt();
                if sp < 0.05 && frame.match_state.enabled() {
                    stalled_ticks[r] += AGENT_EVERY as u64;
                }
                *zone_ticks[r].entry(zone(p[0], p[2])).or_default() += AGENT_EVERY as u64;
                if r == 0 && tick % 4000 == 0 {
                    eprintln!("t={} pos={:?} idx={:?} phase={:?}", tick, p, brains[r].as_ref().map(|b| b.idx), frame.match_state);
                }
                let h = world.held[r];
                if h > held_prev[r] { captures[r] += h - held_prev[r]; }
                if h < held_prev[r] { shots[r] += held_prev[r] - h; }
                held_prev[r] = h;
            }
        }
        world.step(&cmds);
        if world.scores != last_score {
            last_score = world.scores;
        }
    }

    println!("\n=== MATCH ANALYSIS ({} ticks = {:.0}s) ===", args.ticks, args.ticks as f32 / 400.0);
    println!("final scores: {:?}  held: {:?}", world.scores, world.held);
    for r in 0..n {
        println!(
            "robot {}: dist={:.1}m captures={} shots={} stalled={:.1}s held_end={} zones={:?}",
            r, dist[r], captures[r], shots[r],
            stalled_ticks[r] as f32 / 400.0, world.held[r], zone_ticks[r]
        );
    }
    Ok(())
}

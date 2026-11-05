//! Physics orchestrator binary — thin shell (PLAN A4). All logic lives in
//! `frc_sim_m0::{world, agents, config, replay}`; this file wires the
//! config, the deterministic step-count-driven physics loop, and the agent
//! services together.
use anyhow::Result;
use clap::Parser;
use frc_sim_m0::agents::{self, CommandSlots};
use frc_sim_m0::config::{SimConfig, TimeMode};
use frc_sim_m0::replay::{self, LogHeader, LogWriter, Record};
use frc_sim_m0::world::{World, AGENT_EVERY, PHYSICS_HZ};
use frc_sim_m0::{SensorFrame, PROTOCOL_VERSION};
use std::sync::Arc;
use tokio::net::TcpListener;
use tokio::time::{interval, Duration};

#[derive(Parser)]
struct Args {
    /// NT4 server ports to bridge, indexed by robot id (e.g. --nt 5810 --nt 5820)
    #[arg(long)]
    nt: Vec<u16>,
    /// Archetype TOML per robot slot (repeatable, indexed by robot id).
    /// e.g. --archetype archetypes/fast-light.toml --archetype archetypes/heavy-pusher.toml
    #[arg(long)]
    archetype: Vec<std::path::PathBuf>,
    /// Field layout TOML (issue #14). Defaults to the shipped 2026
    /// REBUILT-flavored layout; omit to use it, or pass another file.
    #[arg(long)]
    field: Option<std::path::PathBuf>,
    /// Physics pacing mode
    #[arg(long, value_enum, default_value_t = TimeMode::Realtime)]
    time_mode: TimeMode,
    /// Seed for the world's SimRng (recorded in replay logs)
    #[arg(long, default_value_t = 0)]
    seed: u64,
    /// Record the run's inputs + state samples to this binary log
    #[arg(long)]
    record: Option<std::path::PathBuf>,
    /// Re-run this log headless and exit (no sockets bound)
    #[arg(long)]
    replay: Option<std::path::PathBuf>,
    /// Golden log to diff against (with --replay: compares its replayed
    /// trajectory to this file's recorded states; alone: replays this file
    /// and checks it reproduces its own recorded states)
    #[arg(long)]
    diff: Option<std::path::PathBuf>,
    /// Divergence tolerance in meters (0 = bitwise)
    #[arg(long, default_value_t = 0.0)]
    diff_tol: f32,
}

fn run_replay(cfg: &SimConfig) -> Result<()> {
    // --diff without --replay is a self-check of the golden file
    let log_path = cfg.replay.as_ref().or(cfg.diff.as_ref()).unwrap();
    let (header, records) = replay::read_log(log_path)?;
    println!(
        "replaying {} (seed={}, protocol v{}, {} robots, {} records)",
        log_path.display(),
        header.seed,
        header.protocol_version,
        header.specs.len(),
        records.len()
    );
    let states = replay::replay(&header, &records);
    println!("replayed {} state samples", states.len());
    for (tick, robot, s) in states.iter().rev().take(header.specs.len()) {
        println!(
            "  final tick {} robot {}: pos=({:.3},{:.3},{:.3})",
            tick, robot, s.position[0], s.position[1], s.position[2]
        );
    }

    if let Some(golden_path) = &cfg.diff {
        let (_, golden_records) = replay::read_log(golden_path)?;
        let golden: Vec<(u64, u32, replay::StateSample)> = golden_records
            .iter()
            .filter_map(|r| match r {
                Record::State {
                    tick,
                    robot,
                    sample,
                } => Some((*tick, *robot, sample.clone())),
                _ => None,
            })
            .collect();
        match replay::diff(&golden, &states, cfg.diff_tol) {
            None => println!("DIFF OK: trajectory matches golden ({} samples)", golden.len()),
            Some(d) => {
                println!(
                    "DIVERGENCE at tick {} robot {}: magnitude {:.6} (after {} samples)",
                    d.tick, d.robot, d.magnitude, d.compared
                );
                std::process::exit(1);
            }
        }
    }
    Ok(())
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let cfg = SimConfig {
        time_mode: args.time_mode,
        seed: args.seed,
        nt_ports: args.nt.clone(),
        archetypes: args.archetype.clone(),
        field: args.field.clone().or_else(|| {
            let p = std::path::PathBuf::from("fields/rebuilt.toml");
            p.exists().then_some(p)
        }),
        record: args.record.clone(),
        replay: args.replay.clone(),
        diff: args.diff.clone(),
        diff_tol: args.diff_tol,
    };

    if cfg.replay.is_some() || cfg.diff.is_some() {
        return run_replay(&cfg);
    }

    let specs = cfg.robot_specs();
    let num_robots = specs.len();
    let field = cfg.field_spec()?;
    println!(
        "field {:?}: {:.2} x {:.2} m, {} walls, {} obstacles",
        field.name.as_deref().unwrap_or("unnamed"),
        field.half_x * 2.0,
        field.half_z * 2.0,
        field.walls.len(),
        field.obstacles.len()
    );
    let mut world = World::with_field(&specs, field.clone(), cfg.seed);

    // optional event log: header first, then per-tick records in the loop
    let mut log = match &cfg.record {
        Some(path) => {
            let header = LogHeader {
                protocol_version: PROTOCOL_VERSION,
                seed: cfg.seed,
                config_hash: cfg.hash_field(
                    cfg.hash_archetype_files(SimConfig::config_hash(&specs)),
                    &field,
                ),
                specs: specs.clone(),
            };
            println!("recording event log -> {}", path.display());
            Some(LogWriter::create(path, &header)?)
        }
        None => None,
    };

    let commands = Arc::new(CommandSlots::new(num_robots));
    let (sensor_tx, _) = tokio::sync::broadcast::channel::<(usize, SensorFrame)>(256);

    // spectator world-state stream for the web viz (port 9771)
    tokio::spawn(frc_sim_m0::viz::run(sensor_tx.clone(), field.clone()));

    for (robot_id, port) in cfg.nt_ports.iter().enumerate() {
        if robot_id >= num_robots {
            break;
        }
        tokio::spawn(agents::nt_bridge::run(
            robot_id,
            *port,
            commands.clone(),
            sensor_tx.clone(),
        ));
        println!("nt bridge for robot {} -> :{}", robot_id, port);
    }

    // physics loop: step-count driven (issue #5). `tick` is the only clock.
    {
        let commands = commands.clone();
        let sensor_tx = sensor_tx.clone();
        let mode = cfg.time_mode;
        tokio::spawn(async move {
            let mut tick: u64 = 0;
            let mut last_cmds: Vec<Option<frc_sim_m0::ActuatorFrame>> = vec![None; num_robots];
            let mut iv = interval(Duration::from_micros(1_000_000 / PHYSICS_HZ as u64));
            loop {
                match mode {
                    TimeMode::Realtime => {
                        iv.tick().await;
                    }
                    TimeMode::Fast => {
                        // no sleeps; yield periodically so agent tasks can run
                        if tick % 64 == 0 {
                            tokio::task::yield_now().await;
                        }
                    }
                    TimeMode::Lockstep => {
                        if tick % AGENT_EVERY as u64 == 0 && tick > 0 {
                            // WPILib-style barrier: wait for every connected
                            // WS agent to push a fresh command (20ms timeout).
                            let baseline = commands.seqs();
                            let waiters = commands.ws_connected_robots();
                            let deadline = tokio::time::Instant::now()
                                + Duration::from_millis(20);
                            while tokio::time::Instant::now() < deadline {
                                if waiters
                                    .iter()
                                    .all(|&r| commands.seq(r) > baseline[r])
                                {
                                    break;
                                }
                                tokio::time::sleep(Duration::from_millis(1)).await;
                            }
                        }
                    }
                }

                let cmds = commands.snapshot();
                // event-source the inputs: log a command record on change
                if let Some(w) = log.as_mut() {
                    for (r, (new, old)) in
                        cmds.iter().zip(last_cmds.iter()).enumerate()
                    {
                        if new != old {
                            let _ = w.record(&Record::Cmd {
                                tick,
                                robot: r as u32,
                                cmd: new.clone(),
                            });
                        }
                    }
                }
                last_cmds = cmds.clone();
                world.step(&cmds);
                if tick % AGENT_EVERY as u64 == 0 {
                    for r in 0..num_robots {
                        let frame = world.sensor_frame(r, tick);
                        if let Some(w) = log.as_mut() {
                            let _ = w.record(&Record::State {
                                tick,
                                robot: r as u32,
                                sample: replay::StateSample::from(&frame),
                            });
                        }
                        let _ = sensor_tx.send((r, frame));
                    }
                    // periodic flush so a killed server loses at most ~1 s
                    if tick % PHYSICS_HZ as u64 == 0 {
                        if let Some(w) = log.as_mut() {
                            let _ = w.flush();
                        }
                    }
                }
                tick += 1;
            }
        });
    }

    let listener = TcpListener::bind("127.0.0.1:9770").await?;
    println!("orchestrator listening on ws://127.0.0.1:9770/robot/<id>");
    agents::ws::serve(listener, num_robots, commands, sensor_tx).await
}

//! Physics orchestrator binary — thin shell (PLAN A4). All logic lives in
//! `frc_sim_m0::{world, agents, config, replay}`; this file wires the
//! config, the deterministic step-count-driven physics loop, and the agent
//! services together.
use anyhow::Result;
use clap::Parser;
use frc_sim_m0::agents::proto_log::ProtoLog;
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
    /// number of robots in the lineup (default 6)
    #[arg(long)]
    num_robots: Option<usize>,
    /// Field layout TOML (issue #14). Defaults to the shipped 2026
    /// REBUILT-flavored layout; omit to use it, or pass another file.
    #[arg(long)]
    field: Option<std::path::PathBuf>,
    /// Game-piece spawn/scoring table TOML (issue #3)
    #[arg(long)]
    pieces: Option<std::path::PathBuf>,
    /// Game/match TOML with a `[match]` section: phase durations +
    /// endgame/climb rules (issue #10). Defaults to games/rebuilt.toml
    /// when present, else the built-in REBUILT-style default.
    #[arg(long)]
    game: Option<std::path::PathBuf>,
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
    /// Run an NT4 debug server on this port republishing all robots' world
    /// state under /Sim/* so AdvantageScope can connect (avoids the robot
    /// bridge ports 5810/5820)
    #[arg(long)]
    debug_nt: Option<u16>,
    /// Log every AgentMsg/ServerMsg envelope to this JSON-lines file
    /// (issue #31): one `{"tick","robot","dir","msg"}` line per message,
    /// covering both the WS and NT-bridge agent paths. Distinct from
    /// --record (physics replay) — this captures protocol traffic.
    #[arg(long)]
    proto_log: Option<std::path::PathBuf>,
    /// Match control channel port (issue #33): JSON WS API for
    /// set_phase / clear_phase / add_score / status. 0 disables.
    #[arg(long, default_value_t = 9772)]
    control_port: u16,
    /// Robot slots driven by the in-process RL policy agent (issue #12).
    /// Repeatable, e.g. --policy 0 --policy 2 --policy 4. These slots run
    /// the deterministic linear-stub policy behind the `Policy` trait;
    /// each agent tick appends an (obs, action, reward, done) row to the
    /// experience log. Combine with `--time-mode fast` for rollouts.
    #[arg(long)]
    policy: Vec<usize>,
    /// Fault-injection TOML (issue #32): per-robot-slot loss/latency/
    /// jitter/disconnect rules on agent links — `[[faults.link]]` tables.
    #[arg(long)]
    faults: Option<std::path::PathBuf>,
    /// Experience log path for --policy agents (JSONL; default
    /// experiences.jsonl, only opened when at least one --policy is set)
    #[arg(long)]
    exp_log: Option<std::path::PathBuf>,
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

    // Issue #23 / PLAN C4: a single `--game games/<game>.toml` names the
    // field layout + piece table + match rules in one shot. The explicit
    // --field/--pieces flags still override the game file's refs.
    let game_path = args.game.clone().or_else(|| {
        let p = std::path::PathBuf::from("games/rebuilt.toml");
        p.exists().then_some(p)
    });
    let game_spec = match &game_path {
        Some(p) => Some(frc_sim_m0::r#match::GameSpec::load(p)?),
        None => None,
    };
    if let (Some(g), Some(p)) = (&game_spec, &game_path) {
        println!(
            "game {:?} ({}): field={:?} pieces={:?}",
            g.name(p),
            p.display(),
            g.field,
            g.pieces
        );
        // Archetype compat check: archetypes may declare `compatible_games`;
        // a non-empty list that doesn't contain this game warns (issue #23).
        for path in &args.archetype {
            if let Ok(a) = frc_sim_m0::archetype::Archetype::load(path) {
                let name = g.name(p);
                if !a.compatible_games.is_empty()
                    && !a.compatible_games.iter().any(|c| *c == name)
                {
                    eprintln!(
                        "warning: archetype {:?} ({}) declares compatible_games={:?}; \
                         game {:?} is not listed",
                        a.name,
                        path.display(),
                        a.compatible_games,
                        name
                    );
                }
            }
        }
    }

    let cfg = SimConfig {
        time_mode: args.time_mode,
        seed: args.seed,
        nt_ports: args.nt.clone(),
        archetypes: args.archetype.clone(),
        num_robots: args.num_robots,
        field: args
            .field
            .clone()
            .or_else(|| game_spec.as_ref().and_then(|g| g.field.clone()))
            .or_else(|| {
                let p = std::path::PathBuf::from("fields/rebuilt.toml");
                p.exists().then_some(p)
            }),
        pieces: args
            .pieces
            .clone()
            .or_else(|| game_spec.as_ref().and_then(|g| g.pieces.clone())),
        game: game_path,
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
    let piece_table = cfg.piece_table();
    let match_cfg = cfg.match_config()?;
    println!(
        "match {:?}: disabled={}s auto={}s teleop={}s endgame={}s",
        cfg.game,
        match_cfg.disabled_s,
        match_cfg.auto_s,
        match_cfg.teleop_s,
        match_cfg.endgame_s
    );
    let mut world = World::with_config(&specs, cfg.seed, field.clone(), &piece_table);
    world.match_clock = frc_sim_m0::r#match::MatchClock::new(match_cfg);
    world.apply_preload();
    // shared with the control channel (issue #33); the physics loop locks
    // it briefly each step — step() is fully synchronous
    let world = Arc::new(std::sync::Mutex::new(world));
    // shared match lifecycle: pause gate + reset factory (issue #33)
    let ctl = std::sync::Arc::new(frc_sim_m0::control::MatchCtl {
        paused: std::sync::atomic::AtomicBool::new(false),
        make_world: std::sync::Mutex::new(Box::new({
            let specs = specs.clone();
            let seed = cfg.seed;
            let field = field.clone();
            let piece_table = piece_table.clone();
            let match_cfg = match_cfg;
            move || {
                let mut w = World::with_config(&specs, seed, field.clone(), &piece_table);
                w.match_clock = frc_sim_m0::r#match::MatchClock::new(match_cfg);
                w.apply_preload();
                w
            }
        })),
    });

    // optional event log: header first, then per-tick records in the loop
    let mut log = match &cfg.record {
        Some(path) => {
            let header = LogHeader {
                protocol_version: PROTOCOL_VERSION,
                seed: cfg.seed,
                config_hash: cfg.hash_match(
                    cfg.hash_field(
                        cfg.hash_archetype_files(SimConfig::config_hash(&specs)),
                        &field,
                    ),
                    &match_cfg,
                ),
                specs: specs.clone(),
                pieces: Some(piece_table.clone()),
                match_cfg: Some(match_cfg),
            };
            println!("recording event log -> {}", path.display());
            Some(LogWriter::create(path, &header)?)
        }
        None => None,
    };

    // optional protocol-layer message log (issue #31)
    let proto_log = match &args.proto_log {
        Some(path) => {
            println!("protocol log -> {}", path.display());
            Some(Arc::new(ProtoLog::create(path)?))
        }
        None => None,
    };

    let commands = Arc::new(CommandSlots::new(num_robots));

    // seeded per-link fault injectors (issue #32); shared by the WS server
    // and NT bridge tasks below
    let faults = match &args.faults {
        Some(path) => Some(Arc::new(agents::faults::Faults::load(
            path,
            num_robots,
            cfg.seed,
        )?)),
        None => None,
    };

    // in-process RL policy agents (issue #12): these slots are owned by
    // PolicyAgents running inside the physics loop — obs is built from
    // each robot's filtered SensorFrame + the shared ObsWorld snapshot,
    // and (obs, action, reward, done) rows go to --exp-log JSONL.
    let mut policy_agents: Vec<frc_sim_m0::agents::policy::PolicyAgent> = Vec::new();
    for &r in &args.policy {
        if r >= num_robots {
            eprintln!("--policy {} out of range ({num_robots} robots), skipped", r);
            continue;
        }
        policy_agents.push(frc_sim_m0::agents::policy::PolicyAgent::linear(
            r,
            frc_sim_m0::obs::ObsConfig::default(),
            cfg.seed.wrapping_add(r as u64),
        ));
    }
    let exp_log = if !policy_agents.is_empty() {
        let path = args
            .exp_log
            .clone()
            .unwrap_or_else(|| std::path::PathBuf::from("experiences.jsonl"));
        match frc_sim_m0::agents::policy::ExperienceLogger::create(&path) {
            Ok(l) => {
                println!("experience log -> {}", path.display());
                Some(l)
            }
            Err(e) => {
                eprintln!("cannot open experience log {}: {e}", path.display());
                None
            }
        }
    } else {
        None
    };

    let (sensor_tx, _) = tokio::sync::broadcast::channel::<(usize, SensorFrame)>(256);
    let (world_tx, _) = tokio::sync::broadcast::channel::<frc_sim_m0::viz::VizFrame>(64);

    // spectator world-state stream for the web viz (port 9771)
    tokio::spawn(frc_sim_m0::viz::run(
        sensor_tx.clone(),
        world_tx.clone(),
        field.clone(),
        specs.clone(),
    ));

    // AdvantageScope-compatible NT4 debug publisher (issue #22)
    if let Some(port) = args.debug_nt {
        let tx = sensor_tx.clone();
        tokio::spawn(async move {
            if let Err(e) = frc_sim_m0::debug_nt::run(tx, port).await {
                eprintln!("debug NT server failed: {e}");
            }
        });
    }

    for (robot_id, port) in cfg.nt_ports.iter().enumerate() {
        if robot_id >= num_robots {
            break;
        }
        tokio::spawn(agents::nt_bridge::run(
            robot_id,
            *port,
            commands.clone(),
            sensor_tx.clone(),
            proto_log.clone(),
            faults.clone(),
        ));
        println!("nt bridge for robot {} -> :{}", robot_id, port);
    }

    // physics loop: step-count driven (issue #5). `tick` is the only clock.
    {
        let commands = commands.clone();
        let sensor_tx = sensor_tx.clone();
        let proto_log = proto_log.clone();
        let world_tx = world_tx.clone();
        let world = world.clone();
        let mode = cfg.time_mode;
        let ctl = ctl.clone();
        let mut policy_agents = policy_agents;
        let mut exp_log = exp_log;
        tokio::spawn(async move {
            let mut tick: u64 = 0;
            let mut last_cmds: Vec<Option<frc_sim_m0::ActuatorFrame>> = vec![None; num_robots];
            let mut iv = interval(Duration::from_micros(1_000_000 / PHYSICS_HZ as u64));
            loop {
                if ctl.paused.load(std::sync::atomic::Ordering::Relaxed) {
                    iv.tick().await;
                    continue;
                }
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
                {
                    let mut world = world.lock().unwrap();
                    world.step(&cmds);
                    if tick % AGENT_EVERY as u64 == 0 {
                        // entity snapshot shared by all policy agents this
                        // agent tick (taken before sensor_frame's &mut borrows)
                        let obs_world = if policy_agents.is_empty() {
                            None
                        } else {
                            Some(world.obs_world())
                        };
                        let mut frames: Vec<Option<SensorFrame>> =
                            (0..num_robots).map(|_| None).collect();
                        for r in 0..num_robots {
                            let frame = world.sensor_frame(r, tick);
                            if let Some(w) = log.as_mut() {
                                let _ = w.record(&Record::State {
                                    tick,
                                    robot: r as u32,
                                    sample: replay::StateSample::from(&frame),
                                });
                            }
                            frames[r] = Some(frame.clone());
                            let _ = sensor_tx.send((r, frame));
                        }
                        if let Some(p) = &proto_log {
                            p.note_tick(tick);
                        }
                        // policy agents: filtered frame + shared snapshot ->
                        // command + experience row (deterministic, issue #12)
                        if let Some(ow) = &obs_world {
                            for agent in policy_agents.iter_mut() {
                                if let Some(frame) = &frames[agent.robot] {
                                    let (cmd, exp) = agent.tick(frame, ow);
                                    commands.set(agent.robot, cmd);
                                    if let Some(l) = exp_log.as_mut() {
                                        let _ = l.log(&exp);
                                    }
                                }
                            }
                        }
                        // spectator piece/score stream — 50 Hz (every 8
                        // physics ticks); was 400 Hz and was the lag source
                        if tick % 8 == 0 {
                        let _ = world_tx.send(frc_sim_m0::viz::VizFrame {
                            tick,
                            pieces: world.piece_positions(),
                            scores: world.scores,
                            held: world.held.clone(),
                            // phase_at_tick/time_remaining subtract
                            // match_epoch internally — use those, not the
                            // raw clock, so staging doesn't leak into the
                            // match timer (issue: "complete" at startup)
                            match_state: world.phase_at_tick(tick),
                            match_time: world.time_remaining(),
                            hub_active: if world.match_clock.config.shift_s
                                > 0.0
                                && world.phase_at_tick(tick)
                                    == frc_sim_m0::r#match::MatchPhase::Teleop
                            {
                                Some(if world.hub_active_for_viz(0, tick) {
                                    0
                                } else {
                                    1
                                })
                            } else {
                                None
                            },
                            mech_angles: (0..world.num_robots())
                                .map(|r| world.mech_angle(r))
                                .collect(),
                            intake_active: (0..world.num_robots())
                                .map(|r| world.intake_active(r))
                                .collect(),
                        });
                        }
                    }
                }
                // periodic flush so a killed server loses at most ~1 s
                if tick % PHYSICS_HZ as u64 == 0 {
                    if let Some(w) = log.as_mut() {
                        let _ = w.flush();
                    }
                    if let Some(p) = &proto_log {
                        p.flush();
                    }
                    if let Some(l) = exp_log.as_mut() {
                        let _ = l.flush();
                    }
                }
                tick += 1;
            }
        });
    }

    // match control channel (issue #33)
    if args.control_port != 0 {
        let listener = TcpListener::bind(("127.0.0.1", args.control_port)).await?;
        println!("match control on ws://127.0.0.1:{}", args.control_port);
        tokio::spawn(frc_sim_m0::control::serve(listener, world.clone(), ctl.clone()));
    }

    let listener = TcpListener::bind("127.0.0.1:9770").await?;
    println!("orchestrator listening on ws://127.0.0.1:9770/robot/<id>");
    agents::ws::serve(listener, num_robots, commands, sensor_tx, proto_log, faults).await
}

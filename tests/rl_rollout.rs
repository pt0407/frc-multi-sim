//! Fast-mode rollout smoke test (issue #12): two in-process policy agents
//! drive robots headless for N agent ticks with no wall-clock pacing —
//! the same step-count-driven loop `--time-mode fast` runs, minus the
//! sockets. Verifies the obs/act/log pipeline end-to-end and that the obs
//! stream is deterministic across identical rollouts.
use frc_sim_m0::agents::policy::{ExperienceLogger, PolicyAgent};
use frc_sim_m0::obs::ObsConfig;
use frc_sim_m0::world::pieces::PieceTable;
use frc_sim_m0::world::{chassis, World, AGENT_EVERY};
use frc_sim_m0::{ActuatorFrame, SensorFrame};

const AGENT_TICKS: u64 = 200; // 1600 physics ticks = 4 s sim time

/// One headless rollout: returns (final positions, obs stream hash rows).
fn rollout(seed: u64) -> (Vec<[f32; 3]>, Vec<Vec<f32>>) {
    let mut world = World::with_config(
        &chassis::default_specs(),
        seed,
        Default::default(),
        &PieceTable::default(),
    );
    let mut agents = vec![
        PolicyAgent::linear(0, ObsConfig::default(), seed),
        PolicyAgent::linear(1, ObsConfig::default(), seed + 1),
    ];
    let mut cmds: Vec<Option<ActuatorFrame>> = vec![None; world.num_robots()];
    let mut obs_stream: Vec<Vec<f32>> = Vec::new();

    for agent_tick in 0..AGENT_TICKS {
        let ow = world.obs_world();
        for agent in agents.iter_mut() {
            // filtered sensor truth only — same call the server makes
            let frame: SensorFrame =
                world.sensor_frame(agent.robot, agent_tick * AGENT_EVERY as u64);
            let (cmd, _exp) = agent.tick(&frame, &ow);
            obs_stream.push(_exp.obs);
            cmds[agent.robot] = Some(cmd);
        }
        for _ in 0..AGENT_EVERY {
            world.step(&cmds);
        }
    }
    (
        (0..world.num_robots()).map(|r| world.position(r)).collect(),
        obs_stream,
    )
}

#[test]
fn fast_rollout_completes_and_moves_robots() {
    let (positions, obs) = rollout(42);
    assert_eq!(obs.len(), 2 * AGENT_TICKS as usize);
    assert!(obs.iter().flatten().all(|x| x.is_finite() && x.abs() <= 1.0));
    // policy output must actually move the chassis (nonzero commands)
    let p0 = positions[0];
    assert!(
        (p0[0] - -4.0).abs() > 0.01 || p0[2].abs() > 0.01,
        "robot 0 never moved: {p0:?}"
    );
}

#[test]
fn rollout_is_deterministic() {
    let (pos_a, obs_a) = rollout(7);
    let (pos_b, obs_b) = rollout(7);
    assert_eq!(pos_a, pos_b);
    assert_eq!(obs_a, obs_b);
}

#[test]
fn experience_log_written_during_rollout() {
    let path = std::env::temp_dir().join(format!("fms-rollout-{}.jsonl", std::process::id()));
    let _ = std::fs::remove_file(&path);
    let mut world = World::new();
    let mut agent = PolicyAgent::linear(0, ObsConfig::default(), 0);
    let mut log = ExperienceLogger::create(&path).unwrap();
    let mut cmds = vec![None; world.num_robots()];
    for agent_tick in 0..20u64 {
        let ow = world.obs_world();
        let frame = world.sensor_frame(0, agent_tick * AGENT_EVERY as u64);
        let (cmd, exp) = agent.tick(&frame, &ow);
        log.log(&exp).unwrap();
        cmds[0] = Some(cmd);
        for _ in 0..AGENT_EVERY {
            world.step(&cmds);
        }
    }
    log.flush().unwrap();
    drop(log);
    let lines: Vec<_> = std::fs::read_to_string(&path)
        .unwrap()
        .lines()
        .map(|l| l.to_string())
        .collect();
    assert_eq!(lines.len(), 20);
    for (i, l) in lines.iter().enumerate() {
        let row: serde_json::Value = serde_json::from_str(l).unwrap();
        assert_eq!(row["tick"], i as u64 * AGENT_EVERY as u64);
        assert!(row["obs"].as_array().unwrap().len() > 0);
    }
    let _ = std::fs::remove_file(&path);
}

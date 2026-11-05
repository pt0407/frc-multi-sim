//! Determinism/replay contract tests (issue #5, PLAN A1):
//! - same seed + same inputs -> bit-identical trajectory
//! - event log round-trips through the binary format
//! - replay-diff reports the first divergent tick after an injected change
use frc_sim_m0::replay::{self, LogHeader, LogWriter, Record, StateSample};
use frc_sim_m0::world::chassis;
use frc_sim_m0::world::{World, AGENT_EVERY};
use frc_sim_m0::{ActuatorFrame, PROTOCOL_VERSION};

const SEED: u64 = 0x5eed;
const TICKS: u64 = 800; // 2 s at 400 Hz
const PUSH: f32 = 900.0;

fn scripted_cmd(tick: u64, robot: usize) -> Option<ActuatorFrame> {
    // robot 0 pushes +x then stops; robot 1 pushes -x throughout
    let force = match (robot, tick) {
        (0, t) if t < 600 => [PUSH, 0.0],
        (1, _) => [-PUSH, 0.0],
        _ => [0.0, 0.0],
    };
    Some(ActuatorFrame {
        force,
        torque: 0.0,
        modules: None,
        intake_active: false,
        shoot: None,
    })
}

/// Run the scripted scenario headless, returning (state samples, records).
fn run_and_collect() -> (Vec<(u64, u32, StateSample)>, Vec<Record>) {
    let specs = chassis::default_specs();
    let mut world = World::with_specs(&specs, SEED);
    let n = world.num_robots();
    let mut states = Vec::new();
    let mut records = Vec::new();
    let mut last: Vec<Option<ActuatorFrame>> = vec![None; n];
    for tick in 0..TICKS {
        let cmds: Vec<_> = (0..n).map(|r| scripted_cmd(tick, r)).collect();
        for r in 0..n {
            if cmds[r] != last[r] {
                records.push(Record::Cmd {
                    tick,
                    robot: r as u32,
                    cmd: cmds[r].clone(),
                });
            }
        }
        last = cmds;
        world.step(&last);
        if tick % AGENT_EVERY as u64 == 0 {
            for r in 0..n {
                let s = StateSample::from(&world.sensor_frame(r, tick));
                states.push((tick, r as u32, s.clone()));
                records.push(Record::State {
                    tick,
                    robot: r as u32,
                    sample: s,
                });
            }
        }
    }
    (states, records)
}

fn header() -> LogHeader {
    LogHeader {
        protocol_version: PROTOCOL_VERSION,
        seed: SEED,
        config_hash: frc_sim_m0::config::SimConfig::config_hash(&chassis::default_specs()),
        specs: chassis::default_specs(),
        pieces: None,
    }
}

#[test]
fn same_seed_and_inputs_are_bit_identical() {
    let (a, _) = run_and_collect();
    let (b, _) = run_and_collect();
    assert_eq!(a.len(), b.len());
    for ((ta, ra, sa), (tb, rb, sb)) in a.iter().zip(b.iter()) {
        assert_eq!((ta, ra), (tb, rb));
        assert_eq!(sa, sb, "state diverged at tick {ta} robot {ra}");
    }
}

#[test]
fn log_roundtrips_and_replays_identically() {
    let dir = std::env::temp_dir().join(format!("fms-replay-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("run.fmsl");

    let (_, records) = run_and_collect();
    {
        let mut w = LogWriter::create(&path, &header()).unwrap();
        for r in &records {
            w.record(r).unwrap();
        }
        w.flush().unwrap();
    }

    let (h, loaded) = replay::read_log(&path).unwrap();
    assert_eq!(h.seed, SEED);
    assert_eq!(h.protocol_version, PROTOCOL_VERSION);
    assert_eq!(h.specs.len(), 2);
    assert_eq!(loaded.len(), records.len());

    // replay must reproduce the recorded states bitwise
    let states = replay::replay(&h, &loaded);
    let golden: Vec<_> = loaded
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
    assert!(
        replay::diff(&golden, &states, 0.0).is_none(),
        "replayed trajectory diverged from its own log"
    );
}

#[test]
fn diff_detects_injected_divergence() {
    let (states, records) = run_and_collect();
    // replay with a perturbed command stream: robot 0 stops 100 ticks early
    let mut tampered = records.clone();
    for r in tampered.iter_mut() {
        if let Record::Cmd { tick, robot, cmd } = r {
            if *robot == 0 && *tick == 600 {
                *tick = 500; // stop pushing earlier
                let _ = cmd;
            }
        }
    }
    let states2 = replay::replay(&header(), &tampered);
    let d = replay::diff(&states, &states2, 0.0).expect("expected divergence");
    assert!(d.tick >= 500, "divergence reported at tick {}", d.tick);
    assert!(d.magnitude > 0.0);
}

#[test]
fn diff_reports_magnitude_not_panic_on_extra_samples() {
    let (states, _) = run_and_collect();
    // golden shorter than actual is fine
    let short = &states[..states.len() / 2];
    assert!(replay::diff(short, &states, 0.0).is_none());
}

//! Game-piece layer tests (issue #3, PLAN B3):
//! - a robot driving through its intake region collects the piece
//! - a ballistic shot into a hub zone increments the alliance score
//! - seeded piece spawns + piece dynamics are bit-identical across runs
//! - perf smoke: 200 pieces + 2 chassis at 400 Hz on one core
use frc_sim_m0::archetype::IntakeSpec;
use frc_sim_m0::world::chassis;
use frc_sim_m0::world::field::FieldSpec;
use frc_sim_m0::world::pieces::{HubZone, PieceTable, SpawnZone, FUEL_RADIUS};
use frc_sim_m0::world::World;
use frc_sim_m0::{ActuatorFrame, ShotCommand};

const SEED: u64 = 0xf3e1;

fn intake_spec() -> IntakeSpec {
    // forward mouth: 0.3–0.9 m ahead of chassis center, ±0.4 m wide,
    // near floor level
    IntakeSpec {
        x_min: 0.3,
        x_max: 0.9,
        z_min: -0.4,
        z_max: 0.4,
        y_min: -0.45,
        y_max: 0.0,
    }
}

fn cmd() -> ActuatorFrame {
    ActuatorFrame {
        force: [0.0, 0.0],
        torque: 0.0,
        modules: None,
        intake_active: false,
        shoot: None,
    }
}

/// A single resting piece ~1.2 m ahead of robot 0's spawn (no hub — used
/// for intake tests so the piece can't be scored out from under us).
fn intake_table() -> PieceTable {
    PieceTable {
        zones: vec![SpawnZone {
            center: [-2.8, 0.0],
            half_extents: [0.01, 0.01],
            count: 1,
        }],
        hubs: vec![],
    }
}

/// A hub well clear of any resting piece (for shot tests).
fn hub_table() -> PieceTable {
    PieceTable {
        zones: vec![SpawnZone {
            center: [4.0, 2.0],
            half_extents: [0.01, 0.01],
            count: 1,
        }],
        hubs: vec![HubZone {
            center: [-2.0, 0.0],
            radius: 0.8,
            entry_height: 1.0,
            alliance: 0,
        }],
    }
}

#[test]
fn intake_region_captures_piece() {
    let mut specs = chassis::default_specs();
    specs[0].intake = Some(intake_spec());
    let mut w = World::with_config(&specs, SEED, FieldSpec::default(), &intake_table());
    assert_eq!(w.pieces.len(), 1);

    // robot 0 drives +x with the intake running until it eats the piece
    let mut drive = cmd();
    drive.force = [600.0, 0.0];
    drive.intake_active = true;
    let mut collected_tick = None;
    for tick in 0..1600 {
        w.step(&[Some(drive.clone()), None]);
        if w.held[0] == 1 && collected_tick.is_none() {
            collected_tick = Some(tick);
            break;
        }
    }
    assert!(
        collected_tick.is_some(),
        "piece was never collected (pieces left: {})",
        w.pieces.len()
    );
    assert_eq!(w.pieces.len(), 0, "captured piece should be removed");
    assert_eq!(w.sensor_frame(0, 0).pieces_held, 1);
}

#[test]
fn inactive_intake_does_not_capture() {
    let mut specs = chassis::default_specs();
    specs[0].intake = Some(intake_spec());
    // piece parked inside the intake region already — but intake is off
    let t = PieceTable {
        zones: vec![SpawnZone {
            center: [-4.0 + 0.6, 0.0],
            half_extents: [0.01, 0.01],
            count: 1,
        }],
        hubs: vec![],
    };
    let mut w = World::with_config(&specs, SEED, FieldSpec::default(), &t);
    for _ in 0..200 {
        w.step(&[Some(cmd()), None]);
    }
    assert_eq!(w.held[0], 0);
    assert_eq!(w.pieces.len(), 1);
}

#[test]
fn ballistic_shot_into_hub_scores() {
    let specs = chassis::default_specs();
    let mut w = World::with_config(&specs, SEED, FieldSpec::default(), &hub_table());
    // give robot 0 a piece and fire a lob toward the hub at x=-2
    w.held[0] = 1;
    let mut shot = cmd();
    shot.shoot = Some(ShotCommand {
        launch_velocity: 4.0,
        hood_angle: 0.9, // ~52 deg lob
        turret_yaw: 0.0,
        launch_height: 0.6,
    });
    w.step(&[Some(shot), None]);
    assert_eq!(w.held[0], 0, "shot should consume the held piece");
    assert_eq!(w.pieces.len(), 2); // spawned shot + the table's piece

    for _ in 0..2000 {
        w.step(&[None, None]);
        if w.scores[0] > 0 {
            break;
        }
    }
    assert_eq!(w.scores[0], 1, "shot piece should score in the hub");
    assert_eq!(w.scores[1], 0);
    assert_eq!(w.sensor_frame(0, 0).score, 1);
}

#[test]
fn empty_inventory_cannot_shoot() {
    let specs = chassis::default_specs();
    let mut w = World::with_config(&specs, SEED, FieldSpec::default(), &PieceTable::default());
    let mut shot = cmd();
    shot.shoot = Some(ShotCommand {
        launch_velocity: 10.0,
        hood_angle: 0.5,
        turret_yaw: 0.0,
        launch_height: 0.6,
    });
    w.step(&[Some(shot), None]);
    assert_eq!(w.pieces.len(), 0);
}

#[test]
fn piece_run_is_bit_identical() {
    // seeded spawn table + drag/bounce dynamics must replay identically
    let t = PieceTable {
        zones: vec![SpawnZone {
            center: [0.0, 0.0],
            half_extents: [3.0, 1.5],
            count: 60,
        }],
        hubs: vec![],
    };
    let mut push = cmd();
    push.force = [400.0, 0.0];
    let positions = || {
        let mut w = World::with_config(&chassis::default_specs(), SEED, FieldSpec::default(), &t);
        for _ in 0..400 {
            w.step(&[Some(push.clone()), Some(push.clone())]);
        }
        w.piece_positions()
    };
    assert_eq!(positions(), positions());
}

/// Perf smoke for the §0 budget (~200 pieces at 400 Hz, one core).
/// Run with: cargo test --release --test game_pieces perf -- --nocapture
/// (ignored by default: debug builds are ~100x slower and the measurement
/// belongs in CI's release job anyway)
#[test]
#[ignore]
fn perf_200_pieces_400hz() {
    let t = PieceTable {
        zones: vec![SpawnZone {
            center: [0.0, 0.0],
            half_extents: [6.0, 3.0],
            count: 200,
        }],
        hubs: vec![],
    };
    let mut w = World::with_config(&chassis::default_specs(), SEED, FieldSpec::default(), &t);
    assert_eq!(w.pieces.len(), 200);
    // warm-up settle
    for _ in 0..400 {
        w.step(&[None, None]);
    }
    let start = std::time::Instant::now();
    const TICKS: u32 = 400; // 1 simulated second
    let mut push = cmd();
    push.force = [300.0, 0.0];
    for _ in 0..TICKS {
        w.step(&[Some(push.clone()), Some(push.clone())]);
    }
    let per_step = start.elapsed() / TICKS;
    eprintln!(
        "200 pieces + 2 chassis: {:.1} us/step (400 Hz budget = 2500 us)",
        per_step.as_secs_f64() * 1e6
    );
    // generous bound for debug builds; release runs should be ~50-200 us
    assert!(per_step.as_micros() < 2500, "step too slow: {per_step:?}");
}

/// Replay determinism on a seeded piece-spawn run: log -> read -> replay
/// reproduces the recorded states bitwise (FMSL v2 header carries the
/// piece table).
#[test]
fn replay_log_roundtrip_with_pieces() {
    use frc_sim_m0::replay::{self, LogHeader, LogWriter, Record, StateSample};
    use frc_sim_m0::world::AGENT_EVERY;
    use frc_sim_m0::PROTOCOL_VERSION;

    let mut specs = chassis::default_specs();
    specs[0].intake = Some(intake_spec());
    let t = PieceTable {
        zones: vec![
            SpawnZone {
                center: [-2.8, 0.0],
                half_extents: [0.01, 0.01],
                count: 1,
            },
            SpawnZone {
                center: [0.0, 1.0],
                half_extents: [2.0, 0.5],
                count: 30,
            },
        ],
        hubs: vec![HubZone {
            center: [-2.0, 0.0],
            radius: 0.8,
            entry_height: 1.0,
            alliance: 0,
        }],
    };

    // run: robot 0 intakes + drives, then shoots
    let mut w = World::with_config(&specs, SEED, FieldSpec::default(), &t);
    let mut records = Vec::new();
    let mut last: Option<ActuatorFrame> = None;
    for tick in 0..1600u64 {
        let mut c = cmd();
        c.force = [600.0, 0.0];
        c.intake_active = true;
        if w.held[0] > 0 {
            c.shoot = Some(ShotCommand {
                launch_velocity: 6.0,
                hood_angle: 0.9,
                turret_yaw: 0.0,
                launch_height: 0.6,
            });
        }
        let c = Some(c);
        if c != last {
            records.push(Record::Cmd {
                tick,
                robot: 0,
                cmd: c.clone(),
            });
            last = c;
        }
        w.step(&[last.clone(), None]);
        if tick % AGENT_EVERY as u64 == 0 {
            for r in 0..2 {
                records.push(Record::State {
                    tick,
                    robot: r as u32,
                    sample: StateSample::from(&w.sensor_frame(r, tick)),
                });
            }
        }
    }
    assert!(w.scores[0] >= 1, "run should have scored");

    let dir = std::env::temp_dir().join(format!("fms-pieces-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("pieces.fmsl");
    {
        let header = LogHeader {
            protocol_version: PROTOCOL_VERSION,
            seed: SEED,
            config_hash: 0,
            specs: specs.clone(),
            pieces: Some(t.clone()),
        };
        let mut lw = LogWriter::create(&path, &header).unwrap();
        for r in &records {
            lw.record(r).unwrap();
        }
        lw.flush().unwrap();
    }

    let (h, loaded) = replay::read_log(&path).unwrap();
    assert_eq!(h.pieces.as_ref().unwrap().piece_count(), 31);
    assert_eq!(h.specs[0].intake, Some(intake_spec()));
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
        "piece run did not replay bit-identically"
    );
}

#[test]
fn piece_radius_and_table_parse() {
    let t: PieceTable = toml::from_str(
        r#"
[[zone]]
center = [0.0, 0.0]
half_extents = [2.0, 1.0]
count = 40

[[hub]]
center = [4.0, 0.0]
radius = 0.6
entry_height = 1.0
alliance = 1
"#,
    )
    .unwrap();
    assert_eq!(t.piece_count(), 40);
    assert_eq!(t.hubs.len(), 1);
    assert!(FUEL_RADIUS > 0.07 && FUEL_RADIUS < 0.08);
}

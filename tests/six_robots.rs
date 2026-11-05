//! Issue #11 (PLAN C2): the default lineup is 6 robots — 3 per alliance,
//! alliance = robot_id % 2 — and every per-robot stream (sensor frames,
//! command slots, spawn layout) scales to N without cross-talk.
use frc_sim_m0::world::chassis;
use frc_sim_m0::world::pieces::{PieceTable, SpawnZone};
use frc_sim_m0::world::{FieldSpec, World};
use frc_sim_m0::ActuatorFrame;

const SEED: u64 = 0x515;

fn cmd(force: [f32; 2]) -> ActuatorFrame {
    ActuatorFrame {
        force,
        torque: 0.0,
        modules: None,
        intake_active: false,
        shoot: None,
    }
}

#[test]
fn default_lineup_is_six_robots_in_alliance_rows() {
    let specs = chassis::default_specs();
    assert_eq!(specs.len(), 6);
    // alliance = robot_id % 2: evens on -x, odds on +x
    for (i, s) in specs.iter().enumerate() {
        let want_x = if i % 2 == 0 {
            -chassis::ALLIANCE_X
        } else {
            chassis::ALLIANCE_X
        };
        assert_eq!(s.spawn[0], want_x, "robot {i} alliance x");
        assert_eq!(
            s.spawn[2],
            chassis::STATION_Z[(i / 2) % chassis::STATION_Z.len()],
            "robot {i} station z"
        );
    }
    // robots 0/1 keep the historical 2-robot spawns
    assert_eq!(specs[0].spawn[0], -4.0);
    assert_eq!(specs[0].spawn[2], 0.0);
    assert_eq!(specs[1].spawn[0], 4.0);
    assert_eq!(specs[1].spawn[2], 0.0);
    // all six spawn poses are distinct
    for i in 0..specs.len() {
        for j in (i + 1)..specs.len() {
            assert_ne!(specs[i].spawn, specs[j].spawn);
        }
    }
}

#[test]
fn six_robots_have_independent_sensor_streams() {
    let mut w = World::with_specs(&chassis::default_specs(), SEED);
    assert_eq!(w.num_robots(), 6);

    // give every robot a different command; each SensorFrame must reflect
    // only its own body's pose/velocity
    let cmds: Vec<Option<ActuatorFrame>> = (0..6)
        .map(|r| {
            let dir = if r % 2 == 0 { 1.0 } else { -1.0 };
            Some(cmd([dir * 400.0 * (r as f32 + 1.0), 0.0]))
        })
        .collect();
    for tick in 0..400 {
        w.step(&cmds);
        if tick % 8 == 0 {
            for r in 0..6 {
                let f = w.sensor_frame(r, tick);
                assert_eq!(
                    f.position,
                    w.position(r),
                    "robot {r} sensor pose diverges from its own body"
                );
            }
        }
    }

    // distinct commands -> distinct trajectories: no two robots share a
    // final position
    let positions: Vec<_> = (0..6).map(|r| w.sensor_frame(r, 0).position).collect();
    for i in 0..6 {
        for j in (i + 1)..6 {
            assert_ne!(
                positions[i], positions[j],
                "robots {i} and {j} report identical positions"
            );
        }
    }
}

#[test]
fn contact_impulse_aggregates_all_other_robots() {
    // robot 0 sandwiched between robots 1 and 2, both pressing inward;
    // its contact impulse must aggregate manifolds vs BOTH neighbors,
    // and isolated robot 3 must contribute nothing
    let specs = vec![
        chassis::RobotSpec::at_x(0.0),
        chassis::RobotSpec::at_x(-0.9),
        chassis::RobotSpec::at_x(0.9),
        chassis::RobotSpec::at_x(6.0),
    ];
    let mut w = World::with_specs(&specs, SEED);
    let cmds = [
        None,
        Some(cmd([900.0, 0.0])),  // press +x into robot 0
        Some(cmd([-900.0, 0.0])), // press -x into robot 0
        None,
    ];
    // manifold points exist while the collision is dynamic; resting
    // stacks can settle just outside the manifold prediction distance,
    // so sample every step rather than only at the end
    let mut max0 = 0.0f32;
    let mut max3 = 0.0f32;
    for _ in 0..400 {
        w.step(&cmds);
        max0 = max0.max(w.sensor_frame(0, 0).contact_impulse);
        max3 = max3.max(w.sensor_frame(3, 0).contact_impulse);
    }
    assert!(max0 > 0.0, "robot 0 should feel its neighbors, got {max0}");
    assert_eq!(max3, 0.0, "isolated robot 3 should feel nothing, got {max3}");
}

/// Perf spot-check (PLAN §0): 6 chassis + 200 pieces must hold 400 Hz
/// on one core (<= 2500 µs/step). Prints the measured average; run with
/// `cargo test --release --test six_robots perf -- --ignored --nocapture`.
/// Ignored by default: the budget only makes sense in a release build.
#[test]
#[ignore]
fn perf_six_robots_200_pieces_400hz() {
    let table = PieceTable {
        zones: vec![SpawnZone {
            center: [0.0, 0.0],
            half_extents: [6.0, 3.0],
            count: 200,
        }],
        hubs: vec![],
    };
    let mut w = World::with_config(
        &chassis::default_specs(),
        SEED,
        FieldSpec::default(),
        &table,
    );
    assert_eq!(w.pieces.len(), 200);
    let cmds: Vec<Option<ActuatorFrame>> = (0..6).map(|_| Some(cmd([300.0, 50.0]))).collect();
    // warm-up
    for _ in 0..400 {
        w.step(&cmds);
    }
    let n = 4000; // 10 s of physics
    let t0 = std::time::Instant::now();
    for _ in 0..n {
        w.step(&cmds);
    }
    let per_step_us = t0.elapsed().as_micros() as f64 / n as f64;
    eprintln!("perf: {per_step_us:.1} µs/step (6 chassis + 200 pieces, budget 2500 µs)");
    assert!(
        per_step_us < 2500.0,
        "physics step {per_step_us:.0} µs exceeds the 400 Hz budget"
    );
}

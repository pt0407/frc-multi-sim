//! Self-testable sim (P0 pipeline): assert the invariants users keep
//! hitting by eye — spawn overlap, floating robots, scattered fuel.
use frc_sim_m0::archetype::Archetype;
use frc_sim_m0::world::chassis::RobotSpec;
use frc_sim_m0::world::field::FieldSpec;
use frc_sim_m0::world::pieces::PieceTable;
use frc_sim_m0::world::World;
use std::path::Path;

fn load_game_world(specs: &[RobotSpec]) -> World {
    let field = FieldSpec::load(Path::new("fields/rebuilt.toml")).unwrap();
    let table = PieceTable::load(Path::new("games/fuel.toml")).unwrap();
    World::with_config(specs, 7, field, &table)
}

fn spec_2910() -> RobotSpec {
    let a = Archetype::load(Path::new("archetypes/2910.toml")).unwrap();
    RobotSpec::from_archetype(&a, [-4.0, 0.0])
}

/// No two game pieces overlap at spawn — that was the "balls fly
/// everywhere" bug. Radius 0.075 m => centers must be > ~0.15 m apart.
#[test]
fn pieces_spawn_non_overlapping() {
    let spec = spec_2910();
    let w = load_game_world(&[spec]);
    let p = w.piece_positions();
    assert_eq!(p.len(), 504);
    for i in 0..p.len() {
        for j in i + 1..p.len() {
            let dx = p[i][0] - p[j][0];
            let dz = p[i][2] - p[j][2];
            let dist2 = dx * dx + dz * dz;
            assert!(dist2 > 0.15 * 0.15,
                "pieces {i},{j} overlap at spawn (dist {:.3})", dist2.sqrt());
        }
    }
}

/// Robots spawn with the chassis bottom at the floor — no mid-air plop.
#[test]
fn robots_spawn_grounded() {
    let spec = spec_2910();
    let w = load_game_world(&[spec]);
    let t = w.position(0);
    let half = 0.315;
    assert!((t[1] - half).abs() < 0.02,
        "robot bottom at y={:.3}, half={:.3} — floats", t[1] - half, half);
}

/// 504 fuel, all finite positions, after 1 s of sim.
#[test]
fn match_state_stable_after_1s() {
    let spec = spec_2910();
    let mut w = load_game_world(&[spec]);
    for _ in 0..400 { w.step(&[None]); }
    assert_eq!(w.piece_positions().len(), 504);
    for (i, p) in w.piece_positions().iter().enumerate() {
        assert!(p[0].is_finite() && p[2].is_finite(),
            "piece {i} NaN after 1 s");
    }
}

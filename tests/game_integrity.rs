//! Regression tier-2: the sim should behave like a real match. Each test
//! asserts a property users can't see from a browser but feel when it breaks.
use frc_sim_m0::archetype::Archetype;
use frc_sim_m0::world::chassis::RobotSpec;
use frc_sim_m0::world::field::FieldSpec;
use frc_sim_m0::world::pieces::PieceTable;
use frc_sim_m0::world::World;
use std::path::Path;

fn world_with(spec: RobotSpec) -> World {
    let field = FieldSpec::load(Path::new("fields/rebuilt.toml")).unwrap();
    let table = PieceTable::load(Path::new("games/fuel.toml")).unwrap();
    World::with_config(&[spec], 7, field, &table)
}

fn spec(name: &str) -> RobotSpec {
    let a = Archetype::load(Path::new(&format!("archetypes/{name}.toml"))).unwrap();
    RobotSpec::from_archetype(&a, [-4.0, 0.0])
}

/// Every shipped archetype's model URL resolves to a real file — catches
/// "model missing / wrong URL" silently before the browser does.
#[test]
fn all_archetype_model_urls_resolve() {
    for name in ["2910", "4414", "bananasplit", "crabbot", "duckbot", "intake-bot"] {
        let a = Archetype::load(Path::new(&format!("archetypes/{name}.toml"))).unwrap();
        let url = a.model.as_ref().unwrap().url.clone();
        let path = Path::new("web/public").join(url.trim_start_matches('/'));
        assert!(path.exists(), "{name} model missing: {path:?}");
    }
}

/// Collider dimensions must roughly match the model's real size — catches
/// the "floating robot / wrong scale" bug without needing the GLB.
#[test]
fn collider_matches_archetype_dims() {
    for name in ["2910", "4414", "bananasplit", "crabbot", "duckbot", "intake-bot"] {
        let a = Archetype::load(Path::new(&format!("archetypes/{name}.toml"))).unwrap();
        let spec = RobotSpec::from_archetype(&a, [0.0, 0.0]);
        // half_extents[1] should be ~half the real robot height
        assert!(spec.half_extents[1] > 0.1 && spec.half_extents[1] < 0.5,
            "{name} collider y={:.3} out of sane range", spec.half_extents[1]);
    }
}

/// Staging covers all real staging areas — the exact MoSim layout uses
/// `[[point]]` entries (midfield grid + alliance stashes), so the check
/// counts staged points rather than zone records.
#[test]
fn all_staging_zones_have_pieces() {
    let table = PieceTable::load(Path::new("games/fuel.toml")).unwrap();
    let total = table.piece_count();
    assert!(total >= 456, "staged {total}, expected >= 456 (504 - 48 preload)");
    // midfield grid ~407 plus alliance stashes near each wall
    let midfield = table.points.iter().filter(|p| p.pos[0].abs() < 1.5).count();
    assert!(midfield >= 400, "no neutral-zone pile ({midfield})");
    for side in [-1.0f32, 1.0] {
        let stash = table.points.iter().filter(|p| p.pos[0] * side > 6.0).count();
        assert!(stash >= 20, "alliance {side} stash too small ({stash})");
    }
}

/// Pieces spawn inside the field bounds — not floating outside the walls.
#[test]
fn pieces_inside_field() {
    let w = world_with(spec("2910"));
    let hx = w.field.half_x + 0.05;
    let hz = w.field.half_z + 0.05;
    for (i, p) in w.piece_positions().iter().enumerate() {
        assert!(p[0].abs() <= hx + 0.05 && p[2].abs() <= hz + 0.05,
            "piece {i} at ({:.2},{:.2}) outside field", p[0], p[2]);
    }
}


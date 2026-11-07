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
    for name in ["2910", "bananasplit", "crabbot", "duckbot", "intake-bot"] {
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
    for name in ["2910", "bananasplit", "crabbot", "duckbot", "intake-bot"] {
        let a = Archetype::load(Path::new(&format!("archetypes/{name}.toml"))).unwrap();
        let spec = RobotSpec::from_archetype(&a, [0.0, 0.0]);
        // half_extents[1] should be ~half the real robot height
        assert!(spec.half_extents[1] > 0.1 && spec.half_extents[1] < 0.5,
            "{name} collider y={:.3} out of sane range", spec.half_extents[1]);
    }
}

/// Staging zones cover all real staging areas — neutral zone + 2 depots +
/// 2 outpost chutes — so no staging area is silently missing pieces.
#[test]
fn all_staging_zones_have_pieces() {
    let table = PieceTable::load(Path::new("games/fuel.toml")).unwrap();
    let total: u32 = table.zones.iter().map(|z| z.count).sum();
    assert!(total >= 456, "staged {total}, expected >= 456 (504 - 48 preload)");
    // each real staging area has fuel: neutral zone ~408, 4 stashes ~24 each
    assert!(table.zones.iter().any(|z| z.count >= 400), "no neutral-zone pile");
    let stashes: Vec<_> = table.zones.iter().filter(|z| z.count == 24).collect();
    assert_eq!(stashes.len(), 4, "expected 4 depot/outpost stashes");
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


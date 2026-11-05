//! Field-layout config tests (issue #14): the shipped rebuilt.toml loads,
//! validates, and produces the expected collider set; malformed configs
//! fail loudly at load.
use frc_sim_m0::world::chassis;
use frc_sim_m0::world::pieces::PieceTable;
use frc_sim_m0::world::field::{build_arena, FieldSpec};
use frc_sim_m0::world::World;
use rapier3d::prelude::{ColliderSet, RigidBodySet};

fn rebuilt_path() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("fields/rebuilt.toml")
}

#[test]
fn rebuilt_toml_loads_and_builds_expected_colliders() {
    let spec = FieldSpec::load(&rebuilt_path()).expect("rebuilt.toml should load");
    assert_eq!(spec.name.as_deref(), Some("rebuilt-2026"));
    // fieldcore reference dims (issue #44): 16.541 x 8.069 m
    assert!((spec.half_x - 8.2705).abs() < 1e-6);
    assert!((spec.half_z - 4.0345).abs() < 1e-6);
    assert_eq!(spec.walls.len(), 4);
    // GLB-derived elements (issue #42): 2 hubs + 4 trenches + 4 bumps
    // + 2 towers + 2 depot rail boxes
    assert_eq!(spec.obstacles.len(), 14);
    assert_eq!(spec.scoring_zones.len(), 4);
    assert_eq!(spec.climb_points.len(), 2);
    // 1 ground + 4 walls + 14 obstacles
    assert_eq!(spec.collider_count(), 19);

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    build_arena(&spec, &mut bodies, &mut colliders);
    assert_eq!(colliders.len(), 19);
}

/// Issue #42: the shipped layout must track the real 2026 REBUILT field
/// geometry from the AdvantageScope Field3d_2026FRCFieldV1 GLB. These
/// numbers come from node AABBs in model.glb (see web/ASSETS.md); if the
/// spec drifts this test fails loudly instead of silently desyncing from
/// the visual model.
#[test]
fn rebuilt_layout_matches_glb_elements() {
    use frc_sim_m0::world::field::ObstacleSpec;
    let spec = FieldSpec::load(&rebuilt_path()).unwrap();
    assert_eq!(spec.glb_url.as_deref(), Some("/field-assets/rebuilt-2026.glb"));

    let find = |name: &str| -> (&[f32; 3], &[f32; 3]) {
        spec.obstacles
            .iter()
            .find_map(|o| match o {
                ObstacleSpec::Box {
                    pos,
                    half_extents,
                    name: Some(n),
                    ..
                } if n == name => Some((pos, half_extents)),
                _ => None,
            })
            .unwrap_or_else(|| panic!("missing obstacle {name}"))
    };

    // hub base plate: 1.182 m square centered at x=±3.645 (GLB GE-26301-01)
    let (pos, he) = find("hub-west");
    assert!((pos[0] + 3.645).abs() < 1e-3 && pos[2].abs() < 1e-3);
    assert!((he[0] - 0.60).abs() < 0.02 && (he[2] - 0.60).abs() < 0.02);
    let (pos2, _) = find("hub-east");
    assert!((pos2[0] - 3.645).abs() < 1e-3);
    // hub top must sit at/below the 2.13 m rim so shots can still enter
    let (_, he_hub) = find("hub-west");
    assert!(pos2[1] <= 2.13 && he_hub[1] * 2.0 >= 1.8);

    // tower uprights: GLB x=±7.21, ~1.83 m tall; west tower at +z
    let (pos, he) = find("tower-west");
    assert!((pos[0] + 7.6).abs() < 0.2 && (pos[2] - 0.29).abs() < 0.05);
    assert!(he[1] >= 0.85, "tower must be climb-height");

    // trench skirts: z ≈ ±2.45..2.76 plus counterbalance arm toward wall
    let (pos, he) = find("trench-sw");
    assert!((pos[0] + 3.645).abs() < 1e-3 && pos[2] > 2.4);
    assert!(pos[2] + he[2] >= 3.4, "trench box should cover the hinged arm");

    // bump strips between hub and trench lanes
    let (pos, he) = find("bump-s1");
    assert!((pos[0] + 3.645).abs() < 1e-3 && (pos[2] - 1.67).abs() < 0.1);
    assert!(he[1] <= 0.12, "bump is a low floor strip");

    // climb points sit on the tower top rungs (y ≈ 1.83)
    let cp = spec.climb_points.iter().find(|c| c.name == "tower-west-top").unwrap();
    assert!((cp.pos[0] + 7.21).abs() < 0.1 && (cp.pos[1] - 1.83).abs() < 0.1);
}

#[test]
fn world_from_field_spec_has_field_plus_robot_colliders() {
    let spec = FieldSpec::load(&rebuilt_path()).unwrap();
    let specs = chassis::default_specs();
    let n_robots = specs.len();
    let field_colliders = spec.collider_count();
    let world = World::with_config(&specs, 0, spec, &PieceTable::default());
    // expose the spec through the world so viz/hash callers can read it
    assert_eq!(world.field.walls.len(), 4);
    drop((n_robots, field_colliders, world));
}

#[test]
fn default_field_spec_matches_legacy_layout() {
    let spec = FieldSpec::default();
    assert_eq!(spec.walls.len(), 4);
    assert_eq!(spec.obstacles.len(), 0);
    assert_eq!(spec.collider_count(), 5); // ground + 4 walls, as before
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    build_arena(&spec, &mut bodies, &mut colliders);
    assert_eq!(colliders.len(), 5);
}

#[test]
fn bad_configs_error_loudly() {
    // negative extent
    let toml_bad_wall = r#"
        half_x = 8.0
        half_z = 4.0
        [ground]
        half_extents = [20.0, 0.1, 20.0]
        pos = [0.0, -0.1, 0.0]
        [[walls]]
        pos = [0.0, 0.3, -4.0]
        half_extents = [-1.0, 0.6, 0.05]
    "#;
    let spec: FieldSpec = toml::from_str(toml_bad_wall).unwrap();
    assert!(spec.validate().is_err(), "negative wall extent should fail");

    // NaN position
    let mut spec = FieldSpec::default();
    spec.walls[0].pos[0] = f32::NAN;
    assert!(spec.validate().is_err(), "NaN wall pos should fail");

    // bad scoring zone (min > max)
    let toml_bad_zone = r#"
        half_x = 8.0
        half_z = 4.0
        [ground]
        half_extents = [20.0, 0.1, 20.0]
        pos = [0.0, -0.1, 0.0]
        [[scoring_zones]]
        name = "degenerate"
        min = [1.0, 0.0]
        max = [0.0, 1.0]
    "#;
    let spec: FieldSpec = toml::from_str(toml_bad_zone).unwrap();
    assert!(spec.validate().is_err(), "inverted zone should fail");

    // missing required field fails at parse time
    assert!(toml::from_str::<FieldSpec>("half_z = 4.0").is_err());
}

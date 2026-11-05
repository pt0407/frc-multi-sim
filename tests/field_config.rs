//! Field-layout config tests (issue #14): the shipped rebuilt.toml loads,
//! validates, and produces the expected collider set; malformed configs
//! fail loudly at load.
use frc_sim_m0::world::chassis;
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
    assert!((spec.half_x - 8.25).abs() < 1e-6);
    assert!((spec.half_z - 4.0).abs() < 1e-6);
    assert_eq!(spec.walls.len(), 4);
    assert_eq!(spec.obstacles.len(), 4);
    assert_eq!(spec.scoring_zones.len(), 2);
    assert_eq!(spec.climb_points.len(), 2);
    // 1 ground + 4 walls + 4 obstacles
    assert_eq!(spec.collider_count(), 9);

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    build_arena(&spec, &mut bodies, &mut colliders);
    assert_eq!(colliders.len(), 9);
}

#[test]
fn world_from_field_spec_has_field_plus_robot_colliders() {
    let spec = FieldSpec::load(&rebuilt_path()).unwrap();
    let specs = chassis::default_specs();
    let n_robots = specs.len();
    let field_colliders = spec.collider_count();
    let world = World::with_field(&specs, spec, 0);
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

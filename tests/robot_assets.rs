//! Robot visual-asset pipeline tests (issue #43): archetype `[model]`
//! parsing, propagation onto `RobotSpec`, and the viz config message's
//! per-robot model refs. The GLB itself is display-only — physics never
//! reads these fields.
use frc_sim_m0::archetype::Archetype;
use frc_sim_m0::viz;
use frc_sim_m0::world::chassis::RobotSpec;
use frc_sim_m0::world::field::FieldSpec;

const ARCH_WITH_MODEL: &str = r#"
name = "glb-bot"
mass = 55.0
half_extents = [0.4, 0.4, 0.4]
max_force = 1500.0

[model]
url = "/robot-assets/kitbot-2026.glb"
scale = 1.2
offset = [-0.3, 0.0, 0.05]
rotation_deg = [90.0, 0.0, 90.0]
"#;

fn load(tag: &str, text: &str) -> Archetype {
    let p = std::env::temp_dir().join(format!("fms-arch-{tag}-{}.toml", std::process::id()));
    std::fs::write(&p, text).unwrap();
    let a = Archetype::load(&p).unwrap();
    let _ = std::fs::remove_file(&p);
    a
}

#[test]
fn model_table_parses_and_reaches_robot_spec() {
    let a = load("full", ARCH_WITH_MODEL);
    let m = a.model.as_ref().expect("[model] should parse");
    assert_eq!(m.url, "/robot-assets/kitbot-2026.glb");
    assert_eq!(m.scale, 1.2);
    assert_eq!(m.offset, [-0.3, 0.0, 0.05]);
    assert_eq!(m.rotation_deg, [90.0, 0.0, 90.0]);

    let spec = RobotSpec::from_archetype(&a, [-4.0, 0.0]);
    assert_eq!(spec.model.as_ref().unwrap().url, m.url);
}

#[test]
fn model_defaults_and_absence() {
    // [model] with only a URL gets display defaults
    let a = load(
        "defaults",
        "mass = 50.0\nhalf_extents = [0.4, 0.4, 0.4]\nmax_force = 1000.0\n[model]\nurl = \"/x.glb\"\n",
    );
    let m = a.model.unwrap();
    assert_eq!(m.scale, 1.0);
    assert_eq!(m.offset, [0.0; 3]);
    assert_eq!(m.rotation_deg, [0.0; 3]);

    // no [model] -> None, box fallback stays
    let a2 = load(
        "none",
        "mass = 50.0\nhalf_extents = [0.4, 0.4, 0.4]\nmax_force = 1000.0\n",
    );
    assert!(a2.model.is_none());
    assert!(RobotSpec::from_archetype(&a2, [0.0, 0.0]).model.is_none());
}

#[test]
fn shipped_archetypes_with_models_load() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("archetypes");
    for name in ["balanced", "intake-bot"] {
        let a = Archetype::load(&dir.join(format!("{name}.toml"))).unwrap();
        let m = a
            .model
            .unwrap_or_else(|| panic!("{name} should declare [model]"));
        assert!(m.url.starts_with("/robot-assets/"), "{name} url");
    }
    for name in ["fast-light", "heavy-pusher"] {
        let a = Archetype::load(&dir.join(format!("{name}.toml"))).unwrap();
        assert!(a.model.is_none());
    }
}

#[test]
fn viz_config_carries_per_robot_model_refs() {
    let field = FieldSpec::default();
    let mut specs = vec![RobotSpec::at_x(-4.0), RobotSpec::at_x(4.0)];
    specs[0].model = Some(frc_sim_m0::archetype::ModelSpec {
        url: "/robot-assets/kitbot-2026.glb".into(),
        scale: 1.0,
        offset: [-0.3, 0.0, 0.05],
        rotation_deg: [90.0, 0.0, 90.0],
    });
    let msg: serde_json::Value =
        serde_json::from_str(&viz::config_msg(&field, &specs)).unwrap();
    assert_eq!(msg["type"], "config");
    let robots = msg["robots"].as_array().expect("robots array");
    assert_eq!(robots.len(), 2);
    assert_eq!(robots[0]["id"], 0);
    assert_eq!(robots[0]["model"]["url"], "/robot-assets/kitbot-2026.glb");
    // robot 1 has no model -> null, client keeps the primitive box
    assert!(robots[1]["model"].is_null());
    // per-robot half extents + intake proxy info ride along
    assert_eq!(robots[0]["half_extents"].as_array().unwrap().len(), 3);
    assert!(robots[0]["intake"].is_null());
}

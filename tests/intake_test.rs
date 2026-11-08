use frc_sim_m0::world::World;
use frc_sim_m0::world::chassis::RobotSpec;
use frc_sim_m0::world::field::FieldSpec;
use frc_sim_m0::world::pieces::PieceTable;
use frc_sim_m0::archetype::IntakeSpec;
use frc_sim_m0::{ActuatorFrame};

#[test]
fn intake_captures_in_pile() {
    let mut spec = RobotSpec::at_x(0.0);
    spec.spawn = [0.0, 0.033, 0.8]; // inside neutral pile
    spec.intake = Some(IntakeSpec {
        x_min: 0.3, x_max: 0.95, z_min: -0.55, z_max: 0.55,
        y_min: -0.35, y_max: 0.05,
    });
    let field = FieldSpec::load(std::path::Path::new("fields/rebuilt.toml")).unwrap();
    let table = PieceTable::load(std::path::Path::new("games/fuel.toml")).unwrap();
    let mut w = World::with_config(&[spec], 7, field, &table);
    w.match_clock.config.disabled_s = 0.0;
    w.start_match();
    let cmd = ActuatorFrame {
        force: [0.0,0.0], torque: 0.0, modules: None,
        intake_active: true, shoot: None, mech: None,
        joints: None, shooter_on: false,
    };
    for _ in 0..400 { w.step(&[Some(cmd.clone()), None]); }
    eprintln!("held={} pieces={}", w.held[0], w.pieces.len());
    assert!(w.held[0] > 0, "intake captured nothing");
}

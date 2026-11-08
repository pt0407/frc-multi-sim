use frc_sim_m0::world::World;
use frc_sim_m0::{ActuatorFrame, ShotCommand};

#[test]
fn trace_shot() {
    use frc_sim_m0::archetype::Archetype;
    use frc_sim_m0::world::chassis::RobotSpec;
    use frc_sim_m0::world::field::FieldSpec;
    use frc_sim_m0::world::pieces::PieceTable;
    let a = Archetype::load(std::path::Path::new("archetypes/2910.toml")).unwrap();
    let spec = RobotSpec::from_archetype(&a, [-4.8, 0.0]);
    let field = FieldSpec::load(std::path::Path::new("fields/rebuilt.toml")).unwrap();
    let table = PieceTable::load(std::path::Path::new("games/fuel.toml")).unwrap();
    let mut w = World::with_config(&[spec], 7, field, &table);
    w.held[0] = 4;
    let shot = ActuatorFrame {
        force: [0.0,0.0], torque: 0.0, modules: None, intake_active: false,
        shoot: Some(ShotCommand { launch_velocity: 6.5, hood_angle: 1.25, turret_yaw: 0.0, launch_height: 0.6 }),
        mech: None,
        joints: None,
        shooter_on: false,
    };
    w.step(&[Some(shot)]);
    for t in 0..600 {
        w.step(&[None]);
        if t % 80 == 0 {
            let p = w.piece_positions();
            eprintln!("t={t} npieces={} last={:?} score={:?}", p.len(), p.last(), w.scores);
        }
    }
    eprintln!("final scores: {:?}", w.scores);
}

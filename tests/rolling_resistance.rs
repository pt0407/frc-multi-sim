//! Issue #40 regression tests: the chassis must roll on its wheels, not
//! scrape like a sliding box.
//!
//! Before the fix, the chassis collider's Coulomb friction (~0.9) stalled
//! any commanded force under ~530 N. Now the collider carries only a low
//! rolling-resistance coefficient (0.04, fieldcore) and the per-module
//! tire model supplies lateral grip — so small commanded forces roll the
//! robot while pushes broadside are still resisted at ~mu*m*g.
use frc_sim_m0::world::chassis;
use frc_sim_m0::world::World;
use frc_sim_m0::ActuatorFrame;

const SEED: u64 = 0x40;

fn cmd(force: [f32; 2], torque: f32) -> ActuatorFrame {
    ActuatorFrame {
        force,
        torque,
        modules: None,
        intake_active: false,
        shoot: None,
        mech: None,
    }
}

/// A 150 N commanded force (well under the old ~530 N breakaway) must
/// still accelerate a 60 kg robot: ~2.9 m/s^2 net of rolling resistance,
/// so 3 s of physics should cover several meters.
#[test]
fn small_commanded_force_rolls_the_robot() {
    let mut w = World::with_specs(&[chassis::RobotSpec::at_x(-4.0)], SEED);
    let cmds = [Some(cmd([150.0, 0.0], 0.0))];
    for _ in 0..1200 {
        w.step(&cmds);
    }
    let p = w.position(0);
    assert!(
        p[0] > -4.0 + 2.0,
        "150 N should roll a 60 kg robot >2 m in 3 s, got x={}",
        p[0]
    );
}

/// Sanity upper bound: the same force must not have run away either —
/// traction caps module force at ~4*176 N, but 150 N is far below that
/// and rolling resistance is small, so the robot should be well past
/// midfield but not teleporting.
#[test]
fn small_force_respects_traction_model() {
    let mut w = World::with_specs(&[chassis::RobotSpec::at_x(-4.0)], SEED);
    let cmds = [Some(cmd([150.0, 0.0], 0.0))];
    for _ in 0..1200 {
        w.step(&cmds);
    }
    let p = w.position(0);
    assert!(
        p[0] < 8.0,
        "150 N for 3 s shouldn't cross the whole field, got x={}",
        p[0]
    );
}

/// Lateral grip: an uncommanded robot rammed broadside must resist. Robot
/// 0 sits at midfield with wheels at their default heading (+x); robot 1
/// drives +z into its flank. Sliding friction is ~0.04 now, so without the
/// module tire model robot 0 would glide sideways ~freely; with ~mu*m*g
/// lateral grip it should stay within a fraction of a meter while robot 1
/// presses 300 N (well under the ~706 N grip limit — pushing still works
/// at the limit, but below the limit the tire holds static).
#[test]
fn broadside_push_is_resisted_by_lateral_grip() {
    let specs = vec![
        chassis::RobotSpec::at_x(0.0),
        {
            let mut s = chassis::RobotSpec::at_x(0.0);
            s.spawn = [0.0, s.spawn[1], -3.0];
            s
        },
    ];
    let mut w = World::with_specs(&specs, SEED);
    let cmds = [None, Some(cmd([0.0, 300.0], 0.0))];
    for _ in 0..2000 {
        w.step(&cmds);
    }
    let p0 = w.position(0);
    // free sliding at 0.04*mu would displace many meters under sustained
    // contact; lateral tire grip should hold it under ~0.75 m
    assert!(
        p0[2].abs() < 0.75,
        "broadside push should be resisted by wheel lateral grip, z={}",
        p0[2]
    );
    // and it must not be shoved along its rolling axis either
    assert!(p0[0].abs() < 1.0, "robot drifted forward, x={}", p0[0]);
}

/// The yaw channel still works: a torque command through the module
/// allocation should rotate the chassis (forces applied off-center).
#[test]
fn torque_command_still_rotates() {
    let mut w = World::with_specs(&[chassis::RobotSpec::at_x(0.0)], SEED);
    let cmds = [Some(cmd([0.0, 0.0], 40.0))];
    for _ in 0..800 {
        w.step(&cmds);
    }
    let f = w.sensor_frame(0, 0);
    let q = f.rotation;
    // nonzero rotation about y after 2 s of spin-up torque
    assert!(
        q[1].abs() > 0.05,
        "torque command should rotate the chassis, quat={q:?}"
    );
}

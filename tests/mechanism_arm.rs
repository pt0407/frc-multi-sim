//! Headless spike test for issue #57 "articulated mechanism bodies":
//! ONE revolute-jointed intake arm on robot 0, position-servoed via
//! `ActuatorFrame::mech`. Verifies:
//!   - the arm deploys from its stow angle to the commanded target
//!   - the swinging arm physically contacts a resting piece and moves it
//!   - robots without `[mech]` keep the single-body path (mech_angle None)
//!   - perf smoke: 6 mech'd robots at 400 Hz (prints µs/step, nocapture)
use frc_sim_m0::archetype::{JointDriver, JointMode, JointSpec, MechSpec};
use frc_sim_m0::world::chassis;
use frc_sim_m0::world::field::FieldSpec;
use frc_sim_m0::world::pieces::{PieceTable, SpawnZone};
use frc_sim_m0::world::World;
use frc_sim_m0::{ActuatorFrame, MechCommand};

const SEED: u64 = 0x57;

/// Floor-reaching intake arm off the front bumper: pivot low enough and
/// the bar long enough that the deployed tip sweeps the ground where
/// fuel rests (the old spec bottomed out ~0.2 m above the pieces).
fn arm_spec() -> MechSpec {
    MechSpec {
        joint_type: frc_sim_m0::archetype::JointType::Revolute,
        pivot: [0.40, 0.15, 0.0],
        half_extents: [0.50, 0.03, 0.35],
        mass: 4.0,
        stow_angle: 1.3,
        limits: [-0.7, 1.4],
        motor_stiffness: 300.0,
        motor_damping: 30.0,
        motor_max_force: 400.0,
    }
}

fn deploy_cmd() -> ActuatorFrame {
    ActuatorFrame {
        force: [0.0, 0.0],
        torque: 0.0,
        modules: None,
        intake_active: false,
        shoot: None,
        mech: Some(MechCommand { joint: 0, target: -0.5 }),
        joints: None,
        shooter_on: false,
    }
}

/// Robot 0 (station 0: x=-5.1, z=-1.517) gets an arm; pieces rest in its
/// floor sweep path on the same z lane, under the deployed arm tip.
fn rig() -> (World, usize) {
    let mut specs = chassis::default_specs();
    specs[0].mechs = vec![arm_spec()];
    let table = PieceTable {
            points: vec![],
        zones: vec![SpawnZone {
            center: [-3.9, -1.517],
            half_extents: [0.15, 0.10],
            count: 4,
        }],
        hubs: vec![],
    };
    let w = World::with_config(&specs, SEED, FieldSpec::default(), &table);
    (w, table.zones[0].count as usize)
}

#[test]
fn arm_deploys_and_pushes_piece() {
    let (mut w, n) = rig();
    assert_eq!(w.pieces.len(), n);

    // stowed: arm tipped up (angle near stow_angle, not at the floor)
    let a0 = w.mech_angle(0).expect("robot 0 has a mech");
    assert!(a0 > 0.8, "arm should start stowed, got {a0}");
    // robot 1 has no mechanism -> single-body path
    assert_eq!(w.mech_angle(1), None);
    assert_eq!(w.mech_arm_position(1), None);

    let before = w.piece_positions();
    let mut cmds: Vec<Option<ActuatorFrame>> = vec![None; w.num_robots()];
    cmds[0] = Some(deploy_cmd());
    for _ in 0..800 {
        // 2 s at 400 Hz
        w.step(&cmds);
    }

    let a = w.mech_angle(0).unwrap();
    eprintln!("arm angle stow={a0:.3} deployed={a:.3}");
    assert!(a < 0.0, "arm should deploy downward, got {a}");

    // the swing must have physically shoved at least one piece
    let after = w.piece_positions();
    let max_disp = before
        .iter()
        .zip(after.iter())
        .map(|(b, a)| {
            ((a[0] - b[0]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
        })
        .fold(0.0f32, f32::max);
    eprintln!("max piece displacement = {max_disp:.3} m");
    assert!(
        max_disp > 0.1,
        "arm should have physically displaced a piece (disp={max_disp})"
    );

    // chassis stayed upright & on the field (no solver blowup)
    let p = w.position(0);
    assert!(p[1] > 0.1 && p[1] < 1.0, "chassis y sane: {p:?}");
    assert!(p[0].abs() < 8.3, "chassis on field: {p:?}");
}

#[test]
fn arm_holds_last_target_without_command() {
    let (mut w, _) = rig();
    let mut cmds: Vec<Option<ActuatorFrame>> = vec![None; w.num_robots()];
    cmds[0] = Some(deploy_cmd());
    for _ in 0..400 {
        w.step(&cmds);
    }
    let deployed = w.mech_angle(0).unwrap();
    // stop commanding — the motor holds its last target (legacy agents
    // that never set `mech` leave a stowed arm stowed)
    let idle: Vec<Option<ActuatorFrame>> = vec![None; w.num_robots()];
    for _ in 0..400 {
        w.step(&idle);
    }
    let held = w.mech_angle(0).unwrap();
    eprintln!("deployed={deployed:.3} held={held:.3}");
    assert!(
        (held - deployed).abs() < 0.15,
        "arm should hold target without fresh commands"
    );
}

/// Arm-mounted roller joint: `roller_pos` is arm-local (origin at the
/// mech pivot, +x along the arm) and must rotate with the arm angle.
fn arm_roller_joint() -> JointSpec {
    JointSpec {
        name: "arm-roller".into(),
        mv: "roller".into(),
        scope: None,
        mode: JointMode::Spin,
        driver: JointDriver::Intake,
        axis: None,
        pivot: None,
        max_vel: 10.0,
        accel: 20.0,
        limits: None,
        wheel_radius: 0.0,
        roller_pos: Some([0.3, 0.0, 0.0]),
        roller_radius: Some(0.3),
        launch_radius: None,
        muzzle_pos: None,
        feed_time: None,
        on_arm: true,
    }
}

#[test]
fn on_arm_roller_anchor_follows_arm_angle() {
    let mut specs = chassis::default_specs();
    specs[0].mechs = vec![arm_spec()];
    specs[0].joints = vec![arm_roller_joint()];
    let table = PieceTable::default();
    let mut w = World::with_config(&specs, SEED, FieldSpec::default(), &table);

    let stowed = w.joint_roller_anchor(0, 0).expect("roller anchor");
    let mut cmds: Vec<Option<ActuatorFrame>> = vec![None; w.num_robots()];
    cmds[0] = Some(deploy_cmd());
    for _ in 0..800 {
        w.step(&cmds);
    }
    let deployed = w.joint_roller_anchor(0, 0).expect("roller anchor");

    // the anchor must have swung with the arm (stow ~1.3 rad -> deploy ~-0.6)
    let disp = ((deployed[0] - stowed[0]).powi(2)
        + (deployed[1] - stowed[1]).powi(2)
        + (deployed[2] - stowed[2]).powi(2))
    .sqrt();
    assert!(disp > 0.3, "arm-mounted anchor should move, disp={disp}");

    // and it must equal pivot + Rz(arm_angle) * roller_pos in chassis frame
    // (robot 0 spawns unyawed, so chassis frame == world x/y offsets)
    let th = w.mech_angle(0).unwrap();
    let (s, c) = th.sin_cos();
    let p = arm_spec().pivot;
    let rp = [0.3f32, 0.0, 0.0];
    let expected = [
        w.position(0)[0] + p[0] + rp[0] * c,
        w.position(0)[1] + p[1] + rp[0] * s,
        w.position(0)[2] + p[2],
    ];
    let err = ((deployed[0] - expected[0]).powi(2)
        + (deployed[1] - expected[1]).powi(2)
        + (deployed[2] - expected[2]).powi(2))
    .sqrt();
    assert!(err < 0.05, "anchor {deployed:?} vs expected {expected:?} (err {err})");
}

/// Perf smoke: six mechanism-bearing robots vs the six single-body
/// baseline, 400 Hz, 2 s each — wall times are printed for the PR report.
/// Asserts the mechanism overhead stays within a generous multiple of the
/// baseline rather than an absolute budget (debug-profile CI machines
/// vary wildly; the 400 Hz budget applies to release builds).
#[test]
fn six_mech_robots_perf_smoke() {
    fn bench(mech: bool) -> f64 {
        let mut specs = chassis::default_specs();
        if mech {
            for s in &mut specs {
                s.mechs = vec![arm_spec()];
            }
        }
        let mut w = World::with_config(
            &specs,
            SEED,
            FieldSpec::default(),
            &PieceTable::default(),
        );
        let cmds: Vec<Option<ActuatorFrame>> =
            (0..6).map(|_| Some(deploy_cmd())).collect();
        let t0 = std::time::Instant::now();
        for _ in 0..800 {
            w.step(&cmds);
        }
        t0.elapsed().as_secs_f64() / 800.0 * 1e6
    }
    let base = bench(false);
    let mechd = bench(true);
    eprintln!(
        "6 robots: baseline {base:.1} µs/step, with arms {mechd:.1} µs/step \
         ({:.1}x)",
        mechd / base
    );
    assert!(
        mechd < base * 3.0,
        "mechanism overhead >3x baseline ({mechd:.1} vs {base:.1} µs)"
    );
}

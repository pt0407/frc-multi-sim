//! Headless regression test for issue #9: run the two-robot collision
//! scenario deterministically for a fixed number of physics ticks, assert
//! the robots make contact and end up near recorded golden positions.
//!
//! Reproduce the goldens by running:
//!   cargo test --test headless_collision -- --nocapture
//! and reading the printed positions.
use frc_sim_m0::sim::World;
use frc_sim_m0::ActuatorFrame;

const TICKS: usize = 1200; // 3 s at 400 Hz
const PUSH_FORCE: f32 = 1200.0;
const EPS: f32 = 0.1; // meters

// Golden final positions recorded from this exact scenario.
// Re-recorded after the issue #40/#44 physics pass (rolling-resistance
// chassis + fieldcore damping): robots accelerate faster, contact first
// observed around tick 242 (was 368), and the symmetric head-on push
// settles them near-vertically aligned where the old scraping friction
// left them offset.
const GOLDEN: [[f32; 2]; 2] = [[-0.3991, 0.0031], [0.3992, -0.0032]];

#[test]
fn two_robots_collide_and_match_golden() {
    let mut world = World::new();

    // Robot 0 pushes +x, robot 1 pushes -x: head-on collision at midfield.
    let cmds = [
        Some(ActuatorFrame {
            force: [PUSH_FORCE, 0.0],
            torque: 0.0,
            modules: None,
            intake_active: false,
            shoot: None,
            mech: None,
            joints: None,
            shooter_on: false,
        }),
        Some(ActuatorFrame {
            force: [-PUSH_FORCE, 0.0],
            torque: 0.0,
            modules: None,
            intake_active: false,
            shoot: None,
            mech: None,
            joints: None,
            shooter_on: false,
        }),
    ];

    let mut saw_contact = false;
    let mut contact_tick = None;
    for tick in 0..TICKS {
        world.step(&cmds);
        if world.robot_contact_points(0, 1) > 0 {
            saw_contact = true;
            if contact_tick.is_none() {
                contact_tick = Some(tick);
            }
        }
    }

    let p0 = world.position(0);
    let p1 = world.position(1);
    eprintln!(
        "contact_tick={contact_tick:?} final robot0=({:.4},{:.4}) robot1=({:.4},{:.4})",
        p0[0], p0[2], p1[0], p1[2]
    );

    assert!(saw_contact, "robots never made contact in {TICKS} ticks");

    // symmetric head-on push: they should meet near midfield
    for (got, want) in [([p0[0], p0[2]], GOLDEN[0]), ([p1[0], p1[2]], GOLDEN[1])] {
        assert!(
            (got[0] - want[0]).abs() < EPS && (got[1] - want[1]).abs() < EPS,
            "final position ({:.4},{:.4}) differs from golden ({:.4},{:.4})",
            got[0],
            got[1],
            want[0],
            want[1]
        );
    }
}

//! Headless multi-robot sim: N robots + scripted agents, zero networking.
//! For CI stress tests, RL rollout prototyping, and "what happens when all
//! six bots charge?" experiments. Deterministic given --seed (issue #30).
//!
//! Built-in agent: pure-pursuit-lite P-controller. Default mode `headon`
//! sends every robot at the point diametrically opposite its spawn, so the
//! whole field collides mid-match.
use anyhow::Result;
use clap::Parser;
use frc_sim_m0::world::{chassis, World};
use frc_sim_m0::ActuatorFrame;

const AGENT_EVERY: u64 = 8; // match the orchestrator's 50 Hz agent tick

#[derive(Parser)]
struct Args {
    /// physics ticks to run (400 Hz; 4000 = 10 s)
    #[arg(long, default_value_t = 4000)]
    ticks: u64,
    /// world RNG seed
    #[arg(long, default_value_t = 0)]
    seed: u64,
    /// per-robot force cap (N)
    #[arg(long, default_value_t = 1200.0)]
    max_force: f32,
    /// print a status line every N agent ticks (0 = silent)
    #[arg(long, default_value_t = 40)]
    status_every: u64,
}

/// P-controller toward `target` in the ground plane, clamped to max_force.
fn drive_toward(pos: [f32; 3], vel: [f32; 3], target: [f32; 2], max_force: f32) -> ActuatorFrame {
    let ex = target[0] - pos[0];
    let ez = target[1] - pos[2];
    let mut fx = 800.0 * ex - 80.0 * vel[0];
    let mut fz = 800.0 * ez - 80.0 * vel[2];
    let mag = (fx * fx + fz * fz).sqrt();
    if mag > max_force {
        fx *= max_force / mag;
        fz *= max_force / mag;
    }
    ActuatorFrame {
        force: [fx, fz],
        torque: 0.0,
        modules: None,
        intake_active: false,
        shoot: None,
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let specs = chassis::default_specs();
    let n = specs.len();
    // head-on targets: mirror each spawn x through the field center, keep z
    let targets: Vec<[f32; 2]> = specs
        .iter()
        .map(|s| [-s.spawn[0], s.spawn[2]])
        .collect();

    let mut world = World::with_specs(&specs, args.seed);
    let mut cmds: Vec<Option<ActuatorFrame>> = vec![None; n];
    let mut arrived = vec![false; n];

    for tick in 0..args.ticks {
        if tick % AGENT_EVERY == 0 {
            for r in 0..n {
                if arrived[r] {
                    continue;
                }
                let s = world.sensor_frame(r, tick);
                let pos = s.position;
                let vel = s.linvel;
                let tgt = targets[r];
                let dx = tgt[0] - pos[0];
                let dz = tgt[1] - pos[2];
                if (dx * dx + dz * dz).sqrt() < 0.3 {
                    arrived[r] = true;
                    cmds[r] = Some(ActuatorFrame {
                        force: [0.0, 0.0],
                        torque: 0.0,
                        modules: None,
                        intake_active: false,
                        shoot: None,
                    });
                } else {
                    cmds[r] = Some(drive_toward(pos, vel, tgt, args.max_force));
                }
            }
        }
        world.step(&cmds);

        if args.status_every > 0 && tick % (args.status_every * AGENT_EVERY) == 0 {
            let poses: Vec<String> = (0..n)
                .map(|r| {
                    let p = world.sensor_frame(r, tick).position;
                    format!("r{}=({:.1},{:.1})", r, p[0], p[2])
                })
                .collect();
            println!("tick {:>5}  {}", tick, poses.join("  "));
        }
    }

    println!("\nfinal state after {} ticks ({:.1}s):", args.ticks, args.ticks as f32 / 400.0);
    for r in 0..n {
        let s = world.sensor_frame(r, args.ticks);
        println!(
            "  robot {}: pos=({:.2},{:.2}) vel=({:.2},{:.2}) contacts={} held={} score={}",
            r, s.position[0], s.position[2], s.linvel[0], s.linvel[2],
            s.contact_points, s.pieces_held, s.score
        );
    }
    println!("alliance scores: {:?}", world.scores);
    Ok(())
}

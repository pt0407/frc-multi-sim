//! M4 scaffold (issue #25): replay a recorded FMSL log under swept physics
//! parameters and report trajectory divergence vs the recorded states.
//! Currently sweeps chassis ground-friction; extend to restitution/damping.
use anyhow::Result;
use clap::Parser;
use frc_sim_m0::replay::{self, Record, StateSample};
use frc_sim_m0::world::field::FieldSpec;
use frc_sim_m0::world::pieces::PieceTable;
use frc_sim_m0::world::World;
use frc_sim_m0::ActuatorFrame;

#[derive(Parser)]
struct Args {
    /// recorded log to tune against
    log: std::path::PathBuf,
    /// chassis friction values to sweep
    #[arg(long, value_delimiter = ',')]
    friction: Vec<f32>,
    /// drivetrain mu values to sweep
    #[arg(long, value_delimiter = ',')]
    mu: Vec<f32>,
}

fn run(header: &replay::LogHeader, cmds: &[(u64, u32, Option<ActuatorFrame>)], last: u64, friction: f32, mu: f32) -> Vec<(u64, u32, StateSample)> {
    let empty = PieceTable::default();
    let mut world = World::with_config(
        &header.specs,
        header.seed,
        FieldSpec::default(),
        header.pieces.as_ref().unwrap_or(&empty),
    );
    world.set_chassis_friction(friction);
    world.set_drivetrain_mu(mu);
    let n = world.num_robots();
    let mut cur: Vec<Option<ActuatorFrame>> = vec![None; n];
    let mut ev = 0;
    let mut out = Vec::new();
    for tick in 0..=last {
        while ev < cmds.len() && cmds[ev].0 <= tick {
            if (cmds[ev].1 as usize) < n {
                cur[cmds[ev].1 as usize] = cmds[ev].2.clone();
            }
            ev += 1;
        }
        world.step(&cur);
        if tick % 8 == 0 {
            for r in 0..n {
                out.push((tick, r as u32, StateSample::from(&world.sensor_frame(r, tick))));
            }
        }
    }
    out
}

fn main() -> Result<()> {
    let args = Args::parse();
    let (header, records) = replay::read_log(&args.log)?;
    let golden: Vec<(u64, u32, StateSample)> = records
        .iter()
        .filter_map(|r| match r {
            Record::State { tick, robot, sample } => Some((*tick, *robot, sample.clone())),
            _ => None,
        })
        .collect();
    let cmds: Vec<_> = records
        .iter()
        .filter_map(|r| match r {
            Record::Cmd { tick, robot, cmd } => Some((*tick, *robot, cmd.clone())),
            _ => None,
        })
        .collect();
    let last = golden.iter().map(|g| g.0).max().unwrap_or(0);

    let frictions = if args.friction.is_empty() { vec![0.04] } else { args.friction };
    let mus = if args.mu.is_empty() { vec![1.2] } else { args.mu };
    for (&f, &mu) in frictions.iter().zip(mus.iter().cycle()) {
        let actual = run(&header, &cmds, last, f, mu);
        let div = replay::diff(&golden, &actual, 0.0);
        // mean position error as the divergence metric
        use std::collections::HashMap;
        let amap: HashMap<(u64, u32), &StateSample> =
            actual.iter().map(|(t, r, s)| ((*t, *r), s)).collect();
        let (mut sum, mut n) = (0.0f64, 0u64);
        for (t, r, g) in &golden {
            if let Some(a) = amap.get(&(*t, *r)) {
                let dx = (a.position[0] - g.position[0]) as f64;
                let dz = (a.position[2] - g.position[2]) as f64;
                sum += (dx * dx + dz * dz).sqrt();
                n += 1;
            }
        }
        println!(
            "friction={:.3} mu={:.3} mean_pos_err={:.4}m samples={} first_divergence={:?}",
            f, mu,
            sum / n.max(1) as f64,
            n,
            div.map(|d| (d.tick, d.magnitude))
        );
    }
    Ok(())
}

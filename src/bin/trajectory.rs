//! Trajectory follower agent: drives a sequence of [x,z] waypoints using
//! pure pursuit — the "my auto routine" stand-in for Choreo/PathPlanner.
//! Connects over the same WS protocol as `drive`; sends chassis-level
//! force commands (holonomic net force, `modules: None`).
use anyhow::Result;
use clap::Parser;
use frc_sim_m0::{ActuatorFrame, AgentMsg, ServerMsg, PROTOCOL_VERSION};
use futures_util::{SinkExt, StreamExt};
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    robot_id: usize,
    /// waypoints as x,z pairs; repeat the flag per waypoint:
    /// --waypoints=-3,-2 --waypoints=3,-2 --waypoints=3,2
    #[arg(long, value_delimiter = ',', allow_hyphen_values = true)]
    waypoints: Vec<f32>,
    /// path file: one "x,z" waypoint per line (# comments ok)
    #[arg(long, conflicts_with = "waypoints")]
    path: Option<std::path::PathBuf>,
    /// repeat the path forever instead of stopping at the end
    #[arg(long, default_value_t = false)]
    loop_path: bool,
    /// pure-pursuit lookahead distance (m)
    #[arg(long, default_value_t = 0.6)]
    lookahead: f32,
    /// waypoint reached threshold (m)
    #[arg(long, default_value_t = 0.3)]
    arrive: f32,
    /// max drive force (N)
    #[arg(long, default_value_t = 1200.0)]
    max_force: f32,
    /// hold position at the final waypoint
    #[arg(long, default_value_t = true)]
    hold: bool,
    /// archetype TOML; its max_force overrides --max-force
    #[arg(long)]
    archetype: Option<std::path::PathBuf>,
}

fn load_waypoints(args: &Args) -> Result<Vec<[f32; 2]>> {
    let flat: Vec<f32> = if let Some(p) = &args.path {
        let text = std::fs::read_to_string(p)?;
        text.lines()
            .map(str::trim)
            .filter(|l| !l.is_empty() && !l.starts_with('#'))
            .flat_map(|l| {
                l.split(',')
                    .filter_map(|s| s.trim().parse::<f32>().ok())
                    .collect::<Vec<_>>()
            })
            .collect()
    } else {
        args.waypoints.clone()
    };
    if flat.len() < 2 || flat.len() % 2 != 0 {
        anyhow::bail!("need an even number of waypoint coords (x,z pairs)");
    }
    Ok(flat.chunks_exact(2).map(|c| [c[0], c[1]]).collect())
}

/// Pure pursuit: find the point on the polyline `lookahead` meters of arc
/// length ahead of the closest point on the path to `pos`.
fn pursuit_point(path: &[[f32; 2]], pos: [f32; 2], lookahead: f32) -> [f32; 2] {
    // closest point on each segment -> (seg_idx, point, arc_len_at_point)
    let mut best = (usize::MAX, pos, 0.0f32);
    let mut best_d2 = f32::MAX;
    let mut arc = 0.0f32;
    for j in 0..path.len() - 1 {
        let a = path[j];
        let b = path[j + 1];
        let ab = [b[0] - a[0], b[1] - a[1]];
        let len2 = ab[0] * ab[0] + ab[1] * ab[1];
        let t = if len2 > 1e-6 {
            (((pos[0] - a[0]) * ab[0] + (pos[1] - a[1]) * ab[1]) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let p = [a[0] + ab[0] * t, a[1] + ab[1] * t];
        let d2 = (pos[0] - p[0]).powi(2) + (pos[1] - p[1]).powi(2);
        if d2 < best_d2 {
            best_d2 = d2;
            best = (j, p, arc + len2.sqrt() * t);
        }
        arc += len2.sqrt();
    }
    if best.0 == usize::MAX {
        return path[path.len() - 1];
    }
    // walk the polyline `lookahead` meters past the closest point
    let target_arc = best.2 + lookahead;
    let mut arc = 0.0f32;
    for j in best.0..path.len() - 1 {
        let a = path[j];
        let b = path[j + 1];
        let seglen = dist(a, b);
        if arc + seglen >= target_arc {
            let t = if seglen > 1e-6 { (target_arc - arc) / seglen } else { 0.0 };
            return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
        }
        arc += seglen;
    }
    path[path.len() - 1]
}

fn dist(a: [f32; 2], b: [f32; 2]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let mut path = load_waypoints(&args)?;
    if args.loop_path {
        // append start point so pursuit walks the path back to the beginning
        let first = path[0];
        path.push(first);
    }
    let arch = frc_sim_m0::archetype::Archetype::load_opt(&args.archetype)?;
    let max_force = arch.as_ref().map(|a| a.max_force).unwrap_or(args.max_force);

    let (ws, _) = connect_async("ws://127.0.0.1:9770").await?;
    let (mut sink, mut stream) = ws.split();
    sink.send(Message::Text(
        serde_json::to_string(&AgentMsg::Hello {
            name: format!("traj-{}", args.robot_id),
            robot_id: args.robot_id,
            protocol_version: PROTOCOL_VERSION,
        })
        .unwrap()
        .into(),
    ))
    .await?;

    println!(
        "robot {} following {} waypoint(s), loop={}, lookahead={}",
        args.robot_id,
        path.len(),
        args.loop_path,
        args.lookahead
    );
    for (i, w) in path.iter().enumerate() {
        println!("  wp{}: ({:.2}, {:.2})", i, w[0], w[1]);
    }

    let mut finished = false;
    while let Some(Ok(msg)) = stream.next().await {
        let Message::Text(txt) = msg else { continue };
        let Ok(smsg) = serde_json::from_str::<ServerMsg>(&txt) else {
            continue;
        };
        let ServerMsg::Sensor(s) = smsg else { continue };

        let pos = [s.position[0], s.position[2]];
        let vel = [s.linvel[0], s.linvel[2]];

        if finished {
            if args.hold {
                // PD hold at final waypoint
                let [gx, gz] = path[path.len() - 1];
                let ex = gx - pos[0];
                let ez = gz - pos[1];
                let (mut fx, mut fz) = (800.0 * ex - 80.0 * vel[0], 800.0 * ez - 80.0 * vel[1]);
                let mag = (fx * fx + fz * fz).sqrt();
                if mag > max_force {
                    fx *= max_force / mag;
                    fz *= max_force / mag;
                }
                sink.send(Message::Text(
                    serde_json::to_string(&AgentMsg::Actuator(ActuatorFrame {
                        force: [fx, fz],
                        torque: 0.0,
                        modules: None,
                        intake_active: false,
                        shoot: None,
                        mech: None,
                    }))
                    .unwrap()
                    .into(),
                ))
                .await?;
            }
            continue;
        }

        // reached the last waypoint?
        if dist(pos, path[path.len() - 1]) < args.arrive {
            if args.loop_path {
                // re-run the path; closest-point pursuit picks up wherever we are
            } else {
                finished = true;
                println!("robot {}: path complete", args.robot_id);
            }
        }

        let target = pursuit_point(&path, pos, args.lookahead);

        // Bang-bang drive: full force toward the pursuit point. A PD
        // regulator stalls on this sim's chassis ground friction (~530 N
        // breakaway for a 60 kg box) — real robots roll, this box scrapes,
        // so commands must exceed the breakaway force to move at all.
        let ex = target[0] - pos[0];
        let ez = target[1] - pos[1];
        let d = (ex * ex + ez * ez).sqrt();
        let (fx, fz) = if d > 1e-3 {
            (max_force * ex / d, max_force * ez / d)
        } else {
            (0.0, 0.0)
        };

        if s.tick % 100 == 0 {
            eprintln!(
                "t={} pos=({:.2},{:.2}) tgt=({:.2},{:.2}) f=({:.0},{:.0}) v=({:.2},{:.2})",
                s.tick, pos[0], pos[1], target[0], target[1], fx, fz, vel[0], vel[1]
            );
        }
        sink.send(Message::Text(
            serde_json::to_string(&AgentMsg::Actuator(ActuatorFrame {
                force: [fx, fz],
                torque: 0.0,
                modules: None,
                intake_active: false,
                shoot: None,
                mech: None,
            }))
            .unwrap()
            .into(),
        ))
        .await?;
    }
    Ok(())
}

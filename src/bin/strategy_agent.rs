//! Strategy agent (issue #59): executes a machine-readable strategy file —
//! the seam for external tools (e.g. video scouting) to inject robot behavior.
//! Connects over the WS agent protocol like `drive`.
//!
//! Strategy JSON:
//! {
//!   "name": "example",
//!   "actions": [
//!     {"type":"goto","x":-2.0,"z":1.0},          // drive to point
//!     {"type":"intake","active":true},          // toggle intake
//!     {"type":"wait","seconds":1.5},            // hold in place
//!     {"type":"shoot","launch_velocity":12.0,"hood_angle":0.8,"turret_yaw":0.0,"launch_height":0.5},
//!     {"type":"hold"}                            // hold position forever
//!   ]
//! }
use anyhow::Result;
use clap::Parser;
use frc_sim_m0::{ActuatorFrame, AgentMsg, ServerMsg, ShotCommand, PROTOCOL_VERSION};
use futures_util::{SinkExt, StreamExt};
use serde::Deserialize;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    robot_id: usize,
    /// strategy JSON file
    #[arg(long)]
    strategy: std::path::PathBuf,
    #[arg(long, default_value_t = 1200.0)]
    max_force: f32,
    /// goto reached threshold (m)
    #[arg(long, default_value_t = 0.3)]
    arrive: f32,
    #[arg(long, default_value = "ws://127.0.0.1:9770")]
    server: String,
}

#[derive(Debug, Deserialize)]
struct Strategy {
    name: Option<String>,
    actions: Vec<Action>,
    #[serde(default)]
    loop_actions: bool,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
enum Action {
    Goto { x: f32, z: f32 },
    Intake { active: bool },
    Wait { seconds: f32 },
    Shoot(ShotParams),
    /// drive a mechanism joint to a target (issue #57): joint = index
    /// into the robot's [[mech]] list, target = angle/extent
    Mech { joint: usize, target: f32 },
    Hold,
}

#[derive(Debug, Deserialize)]
struct ShotParams {
    launch_velocity: f32,
    hood_angle: f32,
    turret_yaw: f32,
    launch_height: f32,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let strat: Strategy = serde_json::from_str(&std::fs::read_to_string(&args.strategy)?)?;
    eprintln!(
        "strategy {:?}: {} actions{}",
        strat.name.as_deref().unwrap_or("unnamed"),
        strat.actions.len(),
        if strat.loop_actions { " (loop)" } else { "" }
    );

    let (ws, _) = connect_async(&args.server).await?;
    let (mut sink, mut stream) = ws.split();
    sink.send(Message::Text(
        serde_json::to_string(&AgentMsg::Hello {
            name: format!("strategy-{}", args.robot_id),
            robot_id: args.robot_id,
            protocol_version: PROTOCOL_VERSION,
        })
        .unwrap()
        .into(),
    ))
    .await?;

    let mut idx = 0usize;
    let mut intake_on = false;
    let mut mech_last: Option<frc_sim_m0::MechCommand> = None;
    let mut wait_until: Option<u64> = None;
    let mut done = false;

    while let Some(Ok(msg)) = stream.next().await {
        let Message::Text(txt) = msg else { continue };
        let Ok(ServerMsg::Sensor(s)) = serde_json::from_str::<ServerMsg>(&txt) else {
            continue;
        };

        // advance the action queue
        if !done {
            match strat.actions.get(idx) {
                Some(Action::Goto { x, z }) => {
                    let d = ((x - s.position[0]).powi(2) + (z - s.position[2]).powi(2)).sqrt();
                    if d < args.arrive {
                        idx += 1;
                    }
                }
                Some(Action::Intake { active }) => {
                    intake_on = *active;
                    idx += 1;
                }
                Some(Action::Wait { seconds }) => {
                    if wait_until.is_none() {
                        wait_until = Some(s.tick + (seconds * 50.0) as u64); // 50Hz agent ticks
                    } else if s.tick >= wait_until.unwrap() {
                        wait_until = None;
                        idx += 1;
                    }
                }
                Some(Action::Mech { joint, target }) => {
                    let m = frc_sim_m0::MechCommand { joint: *joint, target: *target };
                    mech_last = Some(m);
                    let act = ActuatorFrame {
                        force: [0.0, 0.0], torque: 0.0, modules: None,
                        intake_active: intake_on, shoot: None, mech: Some(m),
                    };
                    sink.send(Message::Text(
                        serde_json::to_string(&AgentMsg::Actuator(act)).unwrap().into(),
                    ))
                    .await?;
                    idx += 1;
                    continue;
                }
                Some(Action::Shoot(p)) => {
                    let act = ActuatorFrame {
                        force: [0.0, 0.0],
                        torque: 0.0,
                        modules: None,
                        intake_active: intake_on,
                        shoot: Some(ShotCommand {
                            launch_velocity: p.launch_velocity,
                            hood_angle: p.hood_angle,
                            turret_yaw: p.turret_yaw,
                            launch_height: p.launch_height,
                        }),
                        mech: mech_last,
                    };
                    sink.send(Message::Text(
                        serde_json::to_string(&AgentMsg::Actuator(act)).unwrap().into(),
                    ))
                    .await?;
                    idx += 1;
                    continue;
                }
                Some(Action::Hold) | None => {
                    done = !strat.loop_actions;
                    if strat.loop_actions {
                        idx = 0;
                        continue;
                    }
                }
            }
        }

        // emit force toward current goto target, else damp to rest/hold
        let (fx, fz) = match strat.actions.get(idx) {
            Some(Action::Goto { x, z }) if !done => {
                let ex = x - s.position[0];
                let ez = z - s.position[2];
                let mut fx = 300.0 * ex - 60.0 * s.linvel[0];
                let mut fz = 300.0 * ez - 60.0 * s.linvel[2];
                let m = (fx * fx + fz * fz).sqrt();
                if m > args.max_force {
                    fx *= args.max_force / m;
                    fz *= args.max_force / m;
                }
                (fx, fz)
            }
            _ => (-60.0 * s.linvel[0], -60.0 * s.linvel[2]),
        };

        sink.send(Message::Text(
            serde_json::to_string(&AgentMsg::Actuator(ActuatorFrame {
                force: [fx, fz],
                torque: 0.0,
                modules: None,
                intake_active: intake_on,
                shoot: None,
                mech: mech_last,
            }))
            .unwrap()
            .into(),
        ))
        .await?;
    }
    Ok(())
}

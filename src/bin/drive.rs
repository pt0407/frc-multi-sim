use anyhow::Result;
use clap::Parser;
use frc_sim_m0::{ActuatorFrame, AgentMsg, ServerMsg};
use futures_util::{SinkExt, StreamExt};
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

/// Stand-in for a WPILib robot process: only sees its own SensorFrame,
/// commands world-frame force + yaw torque like a holonomic drivetrain.
#[derive(Parser)]
struct Args {
    #[arg(long)]
    robot_id: usize,
    /// waypoint [x,z] the simple P-controller drives toward
    #[arg(long, num_args = 2, allow_hyphen_values = true)]
    target: Vec<f32>,
    /// hold position after reaching waypoint (plays defense)
    #[arg(long, default_value_t = false)]
    hold: bool,
    /// max drive force (N) — the "archetype" stat
    #[arg(long, default_value_t = 1200.0)]
    max_force: f32,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let (ws, _) = connect_async("ws://127.0.0.1:9770").await?;
    let (mut sink, mut stream) = ws.split();

    sink.send(Message::Text(
        serde_json::to_string(&AgentMsg::Hello {
            name: format!("driver-{}", args.robot_id),
            robot_id: args.robot_id,
        })
        .unwrap()
        .into(),
    ))
    .await?;

    let [tx, tz] = [args.target[0], args.target[1]];
    println!(
        "robot {} driving toward ({}, {}) max_force={}",
        args.robot_id, tx, tz, args.max_force
    );

    while let Some(Ok(msg)) = stream.next().await {
        let Message::Text(txt) = msg else { continue };
        let Ok(ServerMsg::Sensor(s)) = serde_json::from_str::<ServerMsg>(&txt) else {
            continue;
        };

        // pure pursuit-lite: P controller on position error, damped by velocity
        let ex = tx - s.position[0];
        let ez = tz - s.position[2];
        let dist = (ex * ex + ez * ez).sqrt();

        let (fx, fz) = if !args.hold && dist < 0.25 {
            (0.0, 0.0)
        } else {
            // PD: force toward target minus velocity damping
            let k_p = 300.0;
            let k_d = 60.0;
            let mut fx = k_p * ex - k_d * s.linvel[0];
            let mut fz = k_p * ez - k_d * s.linvel[2];
            let mag = (fx * fx + fz * fz).sqrt();
            if mag > args.max_force {
                fx *= args.max_force / mag;
                fz *= args.max_force / mag;
            }
            (fx, fz)
        };

        if s.contact_impulse > 0.0 {
            println!(
                "robot {} tick {}: CONTACT pos=({:.2},{:.2}) vel={:.2}",
                args.robot_id, s.tick, s.position[0], s.position[2],
                (s.linvel[0].powi(2) + s.linvel[2].powi(2)).sqrt()
            );
        }

        sink.send(Message::Text(
            serde_json::to_string(&AgentMsg::Actuator(ActuatorFrame {
                force: [fx, fz],
                torque: 0.0,
            }))
            .unwrap()
            .into(),
        ))
        .await?;
    }
    Ok(())
}

//! NT4 bridge: orchestrator acts as an NT client to a robot's own NT
//! server — the same topology as a real WPILib sim. Subscribes to actuator
//! topics, publishes sensor topics. Uses our minimal client
//! (`nt_client` 0.5.1's GenericPublisher has a pubuid-latching bug, see
//! issue #1).
use crate::agents::CommandSlots;
use crate::nt_client_min::NtClient;
use crate::{ActuatorFrame, SensorFrame};
use std::sync::Arc;
use tokio::time::Duration;

pub async fn run(
    robot_id: usize,
    port: u16,
    commands: Arc<CommandSlots>,
    sensor_tx: tokio::sync::broadcast::Sender<(usize, SensorFrame)>,
) {
    loop {
        let Ok(client) = NtClient::connect(port).await else {
            tokio::time::sleep(Duration::from_millis(500)).await;
            continue;
        };
        println!("robot {} nt bridge connected -> :{}", robot_id, port);
        for n in ["pose_x", "pose_z", "yaw", "vel_x", "vel_z", "contact"] {
            client.publish(&format!("/sim/{n}"), 3);
        }
        // poll actuator topics into the command slot
        {
            let client = client.clone();
            let commands = commands.clone();
            tokio::spawn(async move {
                loop {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                    let fx = client.get_f64("/sim/cmd_fx").unwrap_or(0.0) as f32;
                    let fz = client.get_f64("/sim/cmd_fz").unwrap_or(0.0) as f32;
                    let tq = client.get_f64("/sim/cmd_torque").unwrap_or(0.0) as f32;
                    commands.set(
                        robot_id,
                        ActuatorFrame {
                            force: [fx, fz],
                            torque: tq,
                            modules: None,
                        },
                    );
                }
            });
        }
        let mut rx = sensor_tx.subscribe();
        while let Ok((r, s)) = rx.recv().await {
            if !client.alive() {
                break;
            }
            if r != robot_id {
                continue;
            }
            let [qx, qy, qz, qw] = s.rotation;
            let yaw = 2.0 * (qw * qy + qx * qz).atan2(1.0 - 2.0 * (qy * qy + qx * qx));
            let vals = [
                s.position[0],
                s.position[2],
                yaw,
                s.linvel[0],
                s.linvel[2],
                s.contact_impulse,
            ];
            for (i, n) in ["pose_x", "pose_z", "yaw", "vel_x", "vel_z", "contact"]
                .iter()
                .enumerate()
            {
                client.set_f64(&format!("/sim/{n}"), vals[i] as f64);
            }
        }
        eprintln!("robot {} nt bridge dropped, reconnecting", robot_id);
    }
}

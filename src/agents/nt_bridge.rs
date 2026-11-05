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
    let mut failures = 0u32;
    loop {
        let Ok(client) = NtClient::connect(port).await else {
            failures += 1;
            if failures == 1 || failures % 20 == 0 {
                eprintln!(
                    "robot {} nt bridge: connect to :{} failed ({} attempts)",
                    robot_id, port, failures
                );
            }
            tokio::time::sleep(Duration::from_millis(500)).await;
            continue;
        };
        if failures > 0 {
            println!(
                "robot {} nt bridge connected -> :{} after {} failures",
                robot_id, port, failures
            );
            failures = 0;
        } else {
            println!("robot {} nt bridge connected -> :{}", robot_id, port);
        }
        for n in ["pose_x", "pose_z", "yaw", "vel_x", "vel_z", "contact"] {
            client.publish(&format!("/sim/{n}"), 3);
        }
        // poll actuator topics into the command slot; the task exits when
        // the client dies so it can't write stale commands (issue #26)
        let poll = {
            let client = client.clone();
            let commands = commands.clone();
            tokio::spawn(async move {
                loop {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                    if !client.alive() {
                        break;
                    }
                    let fx = client.get_f64("/sim/cmd_fx").unwrap_or(0.0) as f32;
                    let fz = client.get_f64("/sim/cmd_fz").unwrap_or(0.0) as f32;
                    let tq = client.get_f64("/sim/cmd_torque").unwrap_or(0.0) as f32;
                    commands.set(
                        robot_id,
                        ActuatorFrame {
                            force: [fx, fz],
                            torque: tq,
                            modules: None,
                            intake_active: false,
                            shoot: None,
                        },
                    );
                }
            })
        };
        let mut rx = sensor_tx.subscribe();
        while client.alive() {
            match rx.recv().await {
                Ok((r, s)) => {
                    if r != robot_id {
                        continue;
                    }
                    let [qx, qy, qz, qw] = s.rotation;
                    let yaw =
                        2.0 * (qw * qy + qx * qz).atan2(1.0 - 2.0 * (qy * qy + qx * qx));
                    let vals = [
                        s.position[0],
                        s.position[2],
                        yaw,
                        s.linvel[0],
                        s.linvel[2],
                        s.contact_points,
                    ];
                    for (i, n) in ["pose_x", "pose_z", "yaw", "vel_x", "vel_z", "contact"]
                        .iter()
                        .enumerate()
                    {
                        client.set_f64(&format!("/sim/{n}"), vals[i] as f64);
                    }
                }
                Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                Err(_) => break,
            }
        }
        // bridge dropped: stop the poller and release the command slot so the
        // chassis coasts to a stop instead of driving on the last command
        poll.abort();
        commands.clear(robot_id);
        eprintln!("robot {} nt bridge dropped, reconnecting", robot_id);
    }
}

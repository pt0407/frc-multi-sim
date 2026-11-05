//! NT4 bridge: orchestrator acts as an NT client to a robot's own NT
//! server — the same topology as a real WPILib sim. Subscribes to actuator
//! topics, publishes sensor topics. Uses our minimal client
//! (`nt_client` 0.5.1's GenericPublisher has a pubuid-latching bug, see
//! issue #1).
use crate::agents::proto_log::{Dir, ProtoLog};
use crate::agents::CommandSlots;
use crate::nt_client_min::NtClient;
use crate::{ActuatorFrame, AgentMsg, SensorFrame, ServerMsg};
use std::sync::Arc;
use tokio::time::Duration;

/// Initial retry delay after a failed connect; doubles up to this cap.
const RECONNECT_BASE_MS: u64 = 500;
const RECONNECT_MAX_MS: u64 = 5_000;

pub async fn run(
    robot_id: usize,
    port: u16,
    commands: Arc<CommandSlots>,
    sensor_tx: tokio::sync::broadcast::Sender<(usize, SensorFrame)>,
    proto: Option<Arc<ProtoLog>>,
) {
    let mut failures: u32 = 0;
    loop {
        let client = match NtClient::connect(port).await {
            Ok(c) => {
                failures = 0;
                c
            }
            Err(e) => {
                failures += 1;
                // log every failure — a permanently wrong port must not
                // spin silently (issue #29)
                eprintln!(
                    "robot {} nt bridge connect -> :{} failed (attempt {}): {e}",
                    robot_id, port, failures
                );
                let backoff = (RECONNECT_BASE_MS << failures.min(4).saturating_sub(1))
                    .min(RECONNECT_MAX_MS);
                tokio::time::sleep(Duration::from_millis(backoff)).await;
                continue;
            }
        };
        println!("robot {} nt bridge connected -> :{}", robot_id, port);
        for n in ["pose_x", "pose_z", "yaw", "vel_x", "vel_z", "contact"] {
            client.publish(&format!("/sim/{n}"), 3);
        }
        // poll actuator topics into the command slot; the task dies with
        // the connection so a dropped robot can't drive on a stale
        // command forever (issue #26)
        let poller = {
            let client = client.clone();
            let commands = commands.clone();
            let proto = proto.clone();
            tokio::spawn(async move {
                let mut last: Option<ActuatorFrame> = None;
                loop {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                    if !client.alive() {
                        break;
                    }
                    let fx = client.get_f64("/sim/cmd_fx").unwrap_or(0.0) as f32;
                    let fz = client.get_f64("/sim/cmd_fz").unwrap_or(0.0) as f32;
                    let tq = client.get_f64("/sim/cmd_torque").unwrap_or(0.0) as f32;
                    let cmd = ActuatorFrame {
                        force: [fx, fz],
                        torque: tq,
                        modules: None,
                        intake_active: false,
                        shoot: None,
                    };
                    // NT topics are polled, not message-oriented: log only
                    // on change so the file reflects logical traffic
                    if proto.is_some() && last.as_ref() != Some(&cmd) {
                        if let Some(p) = &proto {
                            p.log(robot_id, Dir::In, &AgentMsg::Actuator(cmd.clone()));
                        }
                    }
                    last = Some(cmd.clone());
                    commands.set(robot_id, cmd);
                }
            })
        };
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
            // protocol-log the logical ServerMsg envelope (the NT topic
            // writes above are its wire encoding, issue #31)
            if let Some(p) = &proto {
                p.log_at(s.tick, robot_id, Dir::Out, &ServerMsg::Sensor(s.clone()));
            }
        }
        // connection dropped: kill the poller and clear the command slot,
        // matching the WS path (issue #26)
        poller.abort();
        commands.clear(robot_id);
        eprintln!("robot {} nt bridge dropped, reconnecting", robot_id);
    }
}

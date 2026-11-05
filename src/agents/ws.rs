//! WebSocket agent protocol server (PLAN A4). Handles Hello/Welcome
//! handshakes with protocol-version enforcement (PLAN §0), streams each
//! agent its own `SensorFrame`s, and writes incoming `ActuatorFrame`s into
//! the shared command slots.
use crate::agents::CommandSlots;
use crate::{AgentMsg, SensorFrame, ServerMsg, PROTOCOL_VERSION};
use futures_util::{SinkExt, StreamExt};
use std::sync::Arc;
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::broadcast;
use tokio_tungstenite::tungstenite::Message;

/// Accept loop: one task per connected agent. Runs forever.
pub async fn serve(
    listener: TcpListener,
    num_robots: usize,
    commands: Arc<CommandSlots>,
    sensor_tx: broadcast::Sender<(usize, SensorFrame)>,
) -> anyhow::Result<()> {
    loop {
        let (stream, addr) = listener.accept().await?;
        let commands = commands.clone();
        let sensor_rx = sensor_tx.subscribe();
        tokio::spawn(async move {
            handle_conn(stream, addr, num_robots, commands, sensor_rx).await;
        });
    }
}

async fn handle_conn(
    stream: TcpStream,
    addr: std::net::SocketAddr,
    num_robots: usize,
    commands: Arc<CommandSlots>,
    mut sensor_rx: broadcast::Receiver<(usize, SensorFrame)>,
) {
    let Ok(ws) = tokio_tungstenite::accept_async(stream).await else {
        return;
    };
    let (mut sink, mut stream) = ws.split();

    // first message must be Hello
    let robot_id = match stream.next().await {
        Some(Ok(Message::Text(txt))) => match serde_json::from_str::<AgentMsg>(&txt) {
            Ok(AgentMsg::Hello {
                robot_id,
                protocol_version,
                ..
            }) => {
                if protocol_version != PROTOCOL_VERSION {
                    let err = ServerMsg::Error {
                        message: format!(
                            "protocol version mismatch: agent={}, server={}",
                            protocol_version, PROTOCOL_VERSION
                        ),
                    };
                    eprintln!("agent {addr} rejected: {err:?}");
                    let _ = sink
                        .send(Message::Text(serde_json::to_string(&err).unwrap().into()))
                        .await;
                    return;
                }
                if robot_id >= num_robots {
                    let err = ServerMsg::Error {
                        message: format!("robot_id {robot_id} out of range ({num_robots} robots)"),
                    };
                    let _ = sink
                        .send(Message::Text(serde_json::to_string(&err).unwrap().into()))
                        .await;
                    return;
                }
                robot_id
            }
            _ => return,
        },
        _ => return,
    };
    println!("agent {} connected -> robot {}", addr, robot_id);
    if !commands.claim_ws(robot_id) {
        // issue #29: a second agent must not silently steal the command slot
        let err = ServerMsg::Error {
            message: format!("robot_id {robot_id} already has a connected agent"),
        };
        eprintln!("agent {addr} rejected: {err:?}");
        let _ = sink
            .send(Message::Text(serde_json::to_string(&err).unwrap().into()))
            .await;
        return;
    }
    sink.send(Message::Text(
        serde_json::to_string(&ServerMsg::Welcome {
            robot_id,
            protocol_version: PROTOCOL_VERSION,
        })
        .unwrap()
        .into(),
    ))
    .await
    .ok();

    let cmd_writer = commands.clone();
    let read_task = tokio::spawn(async move {
        while let Some(Ok(msg)) = stream.next().await {
            if let Message::Text(txt) = msg {
                if let Ok(AgentMsg::Actuator(a)) = serde_json::from_str::<AgentMsg>(&txt) {
                    cmd_writer.set(robot_id, a);
                }
            }
        }
    });

    loop {
        match sensor_rx.recv().await {
            Ok((r, frame)) if r == robot_id => {
                let txt = serde_json::to_string(&ServerMsg::Sensor(frame)).unwrap();
                if sink.send(Message::Text(txt.into())).await.is_err() {
                    break;
                }
            }
            Ok(_) => continue,
            Err(broadcast::error::RecvError::Lagged(_)) => continue,
            Err(_) => break,
        }
    }
    read_task.abort();
    commands.clear(robot_id);
    commands.set_ws_connected(robot_id, false);
    println!("robot {} disconnected", robot_id);
}

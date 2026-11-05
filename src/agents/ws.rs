//! WebSocket agent protocol server (PLAN A4). Handles Hello/Welcome
//! handshakes with protocol-version enforcement (PLAN §0), streams each
//! agent its own `SensorFrame`s, and writes incoming `ActuatorFrame`s into
//! the shared command slots.
use crate::agents::faults::Faults;
use crate::agents::proto_log::{Dir, ProtoLog};
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
    proto: Option<Arc<ProtoLog>>,
    faults: Option<Arc<Faults>>,
) -> anyhow::Result<()> {
    loop {
        let (stream, addr) = listener.accept().await?;
        let commands = commands.clone();
        let sensor_rx = sensor_tx.subscribe();
        let proto = proto.clone();
        let faults = faults.clone();
        tokio::spawn(async move {
            handle_conn(stream, addr, num_robots, commands, sensor_rx, proto, faults).await;
        });
    }
}

async fn handle_conn(
    stream: TcpStream,
    addr: std::net::SocketAddr,
    num_robots: usize,
    commands: Arc<CommandSlots>,
    mut sensor_rx: broadcast::Receiver<(usize, SensorFrame)>,
    proto: Option<Arc<ProtoLog>>,
    faults: Option<Arc<Faults>>,
) {
    let Ok(ws) = tokio_tungstenite::accept_async(stream).await else {
        return;
    };
    let (mut sink, mut stream) = ws.split();

    // first message must be Hello
    let robot_id = match stream.next().await {
        Some(Ok(Message::Text(txt))) => match serde_json::from_str::<AgentMsg>(&txt) {
            Ok(hello @ AgentMsg::Hello {
                robot_id,
                protocol_version,
                ..
            }) => {
                if let Some(p) = &proto {
                    p.log(robot_id, Dir::In, &hello);
                }
                if protocol_version != PROTOCOL_VERSION {
                    let err = ServerMsg::Error {
                        message: format!(
                            "protocol version mismatch: agent={}, server={}",
                            protocol_version, PROTOCOL_VERSION
                        ),
                    };
                    eprintln!("agent {addr} rejected: {err:?}");
                    if let Some(p) = &proto {
                        p.log(robot_id, Dir::Out, &err);
                    }
                    let _ = sink
                        .send(Message::Text(serde_json::to_string(&err).unwrap().into()))
                        .await;
                    return;
                }
                if robot_id >= num_robots {
                    let err = ServerMsg::Error {
                        message: format!("robot_id {robot_id} out of range ({num_robots} robots)"),
                    };
                    if let Some(p) = &proto {
                        p.log(robot_id, Dir::Out, &err);
                    }
                    let _ = sink
                        .send(Message::Text(serde_json::to_string(&err).unwrap().into()))
                        .await;
                    return;
                }
                // duplicate robot_id takeover (issue #29): reject loudly
                // instead of silently stealing the command slot
                if !commands.try_claim_ws(robot_id) {
                    let err = ServerMsg::Error {
                        message: format!(
                            "robot_id {robot_id} already claimed by another agent"
                        ),
                    };
                    eprintln!("agent {addr} rejected: {err:?}");
                    if let Some(p) = &proto {
                        p.log(robot_id, Dir::Out, &err);
                    }
                    let _ = sink
                        .send(Message::Text(serde_json::to_string(&err).unwrap().into()))
                        .await;
                    return;
                }
                robot_id
            }
            // not a Hello (or unparsable): capture it so nothing on the
            // wire is lost, then reject
            other => {
                if let Some(p) = &proto {
                    match &other {
                        Ok(m) => p.log(usize::MAX, Dir::In, m),
                        Err(_) => p.log_raw(usize::MAX, Dir::In, &txt),
                    }
                }
                return;
            }
        },
        _ => return,
    };
    println!("agent {} connected -> robot {}", addr, robot_id);
    let welcome = ServerMsg::Welcome {
        robot_id,
        protocol_version: PROTOCOL_VERSION,
    };
    if let Some(p) = &proto {
        p.log(robot_id, Dir::Out, &welcome);
    }
    sink.send(Message::Text(
        serde_json::to_string(&welcome).unwrap().into(),
    ))
    .await
    .ok();

    // issue #32: per-link fault injector (None unless --faults covers this slot)
    let injector = faults.as_ref().and_then(|f| f.get(robot_id));

    let cmd_writer = commands.clone();
    let read_proto = proto.clone();
    let read_inj = injector.clone();
    let read_task = tokio::spawn(async move {
        while let Some(Ok(msg)) = stream.next().await {
            if let Message::Text(txt) = msg {
                match serde_json::from_str::<AgentMsg>(&txt) {
                    Ok(m @ AgentMsg::Actuator(_)) => {
                        if let Some(p) = &read_proto {
                            p.log(robot_id, Dir::In, &m);
                        }
                        if let AgentMsg::Actuator(a) = m {
                            // fault injection: loss/disconnect bursts drop
                            // inbound commands before they reach the slot
                            if let Some(inj) = &read_inj {
                                if !inj.lock().unwrap().allow_inbound() {
                                    continue;
                                }
                            }
                            cmd_writer.set(robot_id, a);
                        }
                    }
                    Ok(_) => {}
                    Err(_) => {
                        if let Some(p) = &read_proto {
                            p.log_raw(robot_id, Dir::In, &txt);
                        }
                    }
                }
            }
        }
    });

    'send: loop {
        match sensor_rx.recv().await {
            Ok((r, frame)) if r == robot_id => {
                let frames = match &injector {
                    Some(inj) => {
                        let mut g = inj.lock().unwrap();
                        let t = frame.tick;
                        g.outbound(t, frame);
                        g.drain_due(t)
                    }
                    None => vec![frame],
                };
                for frame in frames {
                let out = ServerMsg::Sensor(frame);
                if let Some(p) = &proto {
                    // stamp with the frame's own tick, not the loop's
                    // latest, so lagged receivers stay accurate
                    if let ServerMsg::Sensor(f) = &out {
                        p.log_at(f.tick, r, Dir::Out, &out);
                    }
                }
                let txt = serde_json::to_string(&out).unwrap();
                if sink.send(Message::Text(txt.into())).await.is_err() {
                    break 'send;
                }
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

//! Spectator/state-stream WebSocket broadcaster.
//!
//! Separate from the agent protocol: spectators get *all* robots' state,
//! serialized as JSON at ~30 Hz on ws://127.0.0.1:9771. Used by the
//! Babylon.js client in `web/`.

use anyhow::Result;
use frc_sim_m0::SensorFrame;
use futures_util::{SinkExt, StreamExt};
use serde_json::json;
use std::collections::BTreeMap;
use tokio::net::TcpListener;
use tokio::sync::broadcast;
use tokio::time::{interval, Duration};
use tokio_tungstenite::tungstenite::Message;

const VIZ_PORT: u16 = 9771;
const VIZ_HZ: f32 = 30.0;

/// Field geometry mirrors the constants in server.rs so the client can lay
/// out the floor/walls before the first state packet arrives.
fn config_msg() -> String {
    json!({
        "type": "config",
        "field": { "half_x": 8.25, "half_z": 4.0, "wall_height": 0.6, "wall_thick": 0.1 },
        "robot": { "half_extents": 0.4 },
    })
    .to_string()
}

/// Collects the latest SensorFrame per robot from the physics sensor bus and
/// republishes a combined world-state JSON blob at VIZ_HZ.
async fn state_collector(
    sensor_tx: broadcast::Sender<(usize, SensorFrame)>,
    viz_tx: broadcast::Sender<String>,
) {
    let mut rx = sensor_tx.subscribe();
    let mut latest: BTreeMap<usize, SensorFrame> = BTreeMap::new();
    let mut iv = interval(Duration::from_secs_f32(1.0 / VIZ_HZ));
    loop {
        tokio::select! {
            recv = rx.recv() => {
                match recv {
                    Ok((id, frame)) => { latest.insert(id, frame); }
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(_) => break,
                }
            }
            _ = iv.tick() => {
                if latest.is_empty() {
                    continue;
                }
                let tick = latest.values().map(|f| f.tick).max().unwrap_or(0);
                let robots: Vec<_> = latest
                    .iter()
                    .map(|(id, f)| {
                        json!({
                            "id": id,
                            "position": f.position,
                            "rotation": f.rotation,
                            "linvel": f.linvel,
                            "angvel": f.angvel,
                            "contact_impulse": f.contact_impulse,
                        })
                    })
                    .collect();
                let msg = json!({ "type": "state", "tick": tick, "robots": robots }).to_string();
                // ok if no spectators connected
                let _ = viz_tx.send(msg);
            }
        }
    }
}

/// Runs forever: binds the spectator socket and fans world state out to all
/// connected clients. Spawn once from main.
pub async fn run(sensor_tx: broadcast::Sender<(usize, SensorFrame)>) -> Result<()> {
    let listener = TcpListener::bind(("127.0.0.1", VIZ_PORT)).await?;
    println!("spectator stream listening on ws://127.0.0.1:{VIZ_PORT}");

    let (viz_tx, _) = broadcast::channel::<String>(64);
    tokio::spawn(state_collector(sensor_tx, viz_tx.clone()));

    loop {
        let (stream, addr) = listener.accept().await?;
        let mut rx = viz_tx.subscribe();
        let cfg = config_msg();
        tokio::spawn(async move {
            let Ok(ws) = tokio_tungstenite::accept_async(stream).await else {
                return;
            };
            println!("spectator connected: {addr}");
            let (mut sink, mut stream) = ws.split();
            if sink.send(Message::Text(cfg.into())).await.is_err() {
                return;
            }
            // drop inbound messages; spectators are read-only (but drain so
            // pings/closes are handled)
            tokio::spawn(async move { while let Some(Ok(_)) = stream.next().await {} });
            loop {
                match rx.recv().await {
                    Ok(msg) => {
                        if sink.send(Message::Text(msg.into())).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(_) => break,
                }
            }
            println!("spectator disconnected: {addr}");
        });
    }
}

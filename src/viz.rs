//! Spectator/state-stream WebSocket broadcaster.
//!
//! Separate from the agent protocol: spectators get *all* robots' state,
//! serialized as JSON at ~30 Hz on ws://127.0.0.1:9771. Used by the
//! Babylon.js client in `web/`.

use anyhow::Result;
use crate::world::field::FieldSpec;
use crate::world::RobotSpec;
use crate::SensorFrame;
use futures_util::{SinkExt, StreamExt};
use serde_json::json;
use std::collections::BTreeMap;
use tokio::net::TcpListener;
use tokio::sync::broadcast;
use tokio::time::{interval, Duration};
use tokio_tungstenite::tungstenite::Message;

const VIZ_PORT: u16 = 9771;
const VIZ_HZ: f32 = 30.0;

/// Non-agent world state for spectators (issue #3): all live piece poses +
/// per-alliance scores, published from the physics loop.
#[derive(Debug, Clone)]
pub struct VizFrame {
    pub tick: u64,
    /// world positions of free (not captured/scored) pieces
    pub pieces: Vec<[f32; 3]>,
    /// per-alliance score
    pub scores: [u32; 2],
    /// per-robot held inventory
    pub held: Vec<u32>,
    /// current match phase + seconds remaining in it (issue #10)
    pub match_state: crate::r#match::MatchPhase,
    pub match_time: f32,
    /// which alliance hub is scoring-active during teleop shifts
    /// (None = both active). Clients show "RED/BLUE HUB ACTIVE" (issue #10)
    pub hub_active: Option<usize>,
    /// per-robot mechanism arm joint angle (rad), None when unmechanized
    pub mech_angles: Vec<Option<f32>>,
    /// per-robot intake-run flag (drives roller/spinner animation)
    pub intake_active: Vec<bool>,
    /// robots in the current world — spectators prune stale sensor frames
    /// after a lineup change (home-screen robot select)
    pub robot_count: usize,
}

/// Emit the loaded field layout (issue #14) so web clients can render the
/// real geometry — dims, wall segments, and obstacles — before the first
/// state packet arrives. `wall_height`/`wall_thick` are kept for older
/// clients that only know the legacy four-wall layout.
pub fn config_msg(field: &FieldSpec, specs: &[RobotSpec]) -> String {
    let wall_height = field
        .walls
        .iter()
        .map(|w| w.half_extents[1] * 2.0)
        .fold(0.0f32, f32::max);
    // issue #42: spec paths like "assets/fields/x.glb" are served by vite's
    // publicDir ('../assets'), so the URL drops the leading "assets/".
    let glb_url = field.glb.as_ref().map(|p| {
        let rel = p.strip_prefix("assets/").unwrap_or(p);
        format!("/{rel}")
    });
    json!({
        "type": "config",
        "field": {
            "name": field.name,
            "half_x": field.half_x,
            "half_z": field.half_z,
            "wall_height": wall_height,
            "wall_thick": 0.1,
            "walls": field.walls,
            "obstacles": field.obstacles,
            "scoring_zones": field.scoring_zones,
            "climb_points": field.climb_points,
            // optional GLB visual model (issue #42); null when the spec
            // doesn't reference one — clients keep primitive geometry
            "glb_url": field.glb_url.clone().or(glb_url),
        },
        // legacy single-robot dims (pre-#43 clients)
        "robot": { "half_extents": 0.4 },
        // per-robot visual/mechanism refs (issue #43): model = GLB to
        // render instead of the fallback box (null = keep the box),
        // intake = chassis-relative capture region shown as a translucent
        // proxy while it's not a real articulated mechanism yet
        "robots": specs
            .iter()
            .enumerate()
            .map(|(id, s)| {
                json!({
                    "id": id,
                    "half_extents": s.half_extents,
                    "model": s.model.as_ref().cloned().or_else(|| {
                        // bare-minimum default: KitBot for every robot that
                        // doesn't declare its own CAD (issue #43 v2)
                        Some(crate::archetype::ModelSpec {
                            url: "/robot-assets/kitbot-2026.glb".into(),
                            scale: 0.72,
                            offset: [-0.30, 0.0, 0.05],
                            rotation_deg: [90.0, 0.0, 90.0],
                        })
                    }),
                    "intake": s.intake,
                    // articulated mech specs so the client can bind CAD
                    // subtrees to the same joint pivots physics uses
                    "mechs": s.mechs,
                    // kinematic mechanism joints (spin/position/steer/
                    // wheel): mv regex + motor params for CAD binding,
                    // state arrives per-tick in the robots[] "joints" field
                    "joints": s.joints,
                })
            })
            .collect::<Vec<_>>(),
    })
    .to_string()
}

/// Collects the latest SensorFrame per robot from the physics sensor bus and
/// republishes a combined world-state JSON blob at VIZ_HZ.
async fn state_collector(
    sensor_tx: broadcast::Sender<(usize, SensorFrame)>,
    world_tx: broadcast::Sender<VizFrame>,
    viz_tx: broadcast::Sender<String>,
) {
    let mut rx = sensor_tx.subscribe();
    let mut wrx = world_tx.subscribe();
    let mut latest: BTreeMap<usize, SensorFrame> = BTreeMap::new();
    let mut world_state = VizFrame {
        tick: 0,
        pieces: Vec::new(),
        scores: [0; 2],
        held: Vec::new(),
        match_state: crate::r#match::MatchPhase::Disabled,
        match_time: 0.0,
        hub_active: None,
        mech_angles: Vec::new(),
        intake_active: Vec::new(),
        robot_count: usize::MAX,
    };
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
            recv = wrx.recv() => {
                match recv {
                    Ok(f) => { world_state = f; }
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(_) => {}
                }
            }
            _ = iv.tick() => {
                if latest.is_empty() {
                    continue;
                }
                let tick = latest.values().map(|f| f.tick).max().unwrap_or(0);
                // lineup changes shrink/grow the world — drop frames for
                // robots that no longer exist so ghosts don't linger
                latest.retain(|id, _| *id < world_state.robot_count);
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
                            "joints": f.joints,
                        })
                    })
                    .collect();
                let msg = json!({
                    "type": "state",
                    "tick": tick,
                    "robots": robots,
                    "pieces": world_state.pieces,
                    "scores": world_state.scores,
                    "held": world_state.held,
                    "match_state": world_state.match_state.name(),
                    "match_time": world_state.match_time,
                    "hub_active": world_state.hub_active,
                    "mech_angles": world_state.mech_angles,
                    "intake_active": world_state.intake_active,
                })
                .to_string();
                // ok if no spectators connected
                let _ = viz_tx.send(msg);
            }
        }
    }
}

/// Runs forever: binds the spectator socket and fans world state out to all
/// connected clients. Spawn once from main.
pub async fn run(
    sensor_tx: broadcast::Sender<(usize, SensorFrame)>,
    world_tx: broadcast::Sender<VizFrame>,
    field: FieldSpec,
    specs: std::sync::Arc<std::sync::Mutex<Vec<RobotSpec>>>,
) -> Result<()> {
    let listener = TcpListener::bind(("127.0.0.1", VIZ_PORT)).await?;
    println!("spectator stream listening on ws://127.0.0.1:{VIZ_PORT}");

    let (viz_tx, _) = broadcast::channel::<String>(64);
    tokio::spawn(state_collector(sensor_tx, world_tx, viz_tx.clone()));

    loop {
        let (stream, addr) = listener.accept().await?;
        let mut rx = viz_tx.subscribe();
        // lineup can change at runtime (home-screen select) — read the
        // shared spec list per connection so config matches the world
        let cfg = config_msg(&field, &specs.lock().unwrap().clone());
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

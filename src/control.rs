//! Match control channel (issue #33): a small JSON-over-WebSocket API for
//! manually overriding match state — set phase, award score, query status.
//! For rehearsals, viz demos, and debugging before the full #10 follow-ups.
//!
//! Bind a TCP listener and connect any WS client; each text frame is one
//! command, answered with a JSON status snapshot.
use crate::archetype::Archetype;
use crate::r#match::MatchPhase;
use crate::world::{chassis, RobotSpec, World};
use futures_util::{SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};
use tokio::net::{TcpListener, TcpStream};
use tokio_tungstenite::tungstenite::Message;

use std::sync::atomic::{AtomicBool, Ordering};

/// Shared match-lifecycle state (issue #33 follow-up): the physics loop
/// consults `paused` each tick; `make_world` rebuilds for reset.
pub struct MatchCtl {
    pub paused: AtomicBool,
    pub make_world: std::sync::Mutex<Box<dyn Fn() -> World + Send>>,
    /// live robot specs — `SetLineup` rewrites this and rebuilds the world;
    /// `viz::run` reads it per spectator connection for the config message
    pub specs: std::sync::Arc<std::sync::Mutex<Vec<RobotSpec>>>,
    /// startup lineup — `set_lineup` with an empty list restores it
    pub defaults: Vec<RobotSpec>,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum ControlMsg {
    /// Gate physics stepping — true pauses, false resumes.
    Pause { paused: bool },
    /// Rebuild the world from the recorded config (fresh match).
    Reset,
    /// Arm the match: exit staging, start the clock at auto.
    StartMatch,
    /// Force a match phase. `secs` omitted/0 = hold until `clear_phase`.
    SetPhase { phase: MatchPhase, secs: Option<f32> },
    /// Drop the phase override; the clock resumes.
    ClearPhase,
    /// Award points to an alliance (alliance = robot_id % 2).
    AddScore { alliance: usize, points: u32 },
    /// No-op query — just returns the status snapshot.
    Status,
    /// Replace the robot lineup and rebuild the world (home-screen select).
    /// Names resolve like `--archetype` ("2910", "fast-light", paths).
    /// Empty list restores the startup lineup.
    SetLineup { archetypes: Vec<String> },
    /// List loadable archetype names (archetypes/*.toml stems) for the
    /// home-screen picker.
    ListArchetypes,
}

/// Resolve lineup names to spawn specs, using the same alliance/station
/// layout as `SimConfig::robot_specs`.
fn specs_for(names: &[String]) -> anyhow::Result<Vec<RobotSpec>> {
    let mut specs = Vec::with_capacity(names.len());
    for (i, name) in names.iter().enumerate() {
        let path = Archetype::resolve(std::path::Path::new(name));
        let a = Archetype::load(&path)?;
        let base = chassis::default_spec(i);
        specs.push(RobotSpec::from_archetype(&a, [base.spawn[0], base.spawn[2]]));
    }
    Ok(specs)
}

/// Top-level archetype TOML stems, sorted — what the home screen offers.
fn archetype_names() -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir("archetypes")
        .map(|rd| {
            rd.filter_map(|e| {
                let p = e.ok()?.path();
                if p.extension()?.to_string_lossy() != "toml" {
                    return None;
                }
                Some(p.file_stem()?.to_string_lossy().into_owned())
            })
            .collect()
        })
        .unwrap_or_default();
    names.sort();
    names
}

#[derive(Debug, Serialize)]
struct Status {
    tick: u64,
    phase: MatchPhase,
    time_remaining: f32,
    scores: [u32; 2],
    score_by_phase: [crate::r#match::PhaseScore; 2],
    zone_penalties: [u32; 2],
    tower_fouls: [u32; 2],
    override_active: bool,
}

fn status(world: &World) -> Status {
    Status {
        tick: world.tick(),
        phase: world.phase(),
        time_remaining: world.time_remaining(),
        scores: world.scores,
        score_by_phase: world.score_by_phase,
        zone_penalties: world.zone_penalty_instances(),
        tower_fouls: world.tower_foul_instances(),
        override_active: world.phase_override_active(),
    }
}

async fn handle_conn(
    stream: TcpStream,
    world: Arc<Mutex<World>>,
    ctl: Arc<MatchCtl>,
) -> anyhow::Result<()> {
    let Ok(ws) = tokio_tungstenite::accept_async(stream).await else {
        return Ok(());
    };
    let (mut sink, mut stream) = ws.split();
    while let Some(Ok(msg)) = stream.next().await {
        let Message::Text(txt) = msg else { continue };
        let reply = match serde_json::from_str::<ControlMsg>(&txt) {
            Ok(ControlMsg::Pause { paused }) => {
                ctl.paused.store(paused, Ordering::Relaxed);
                let w = world.lock().unwrap();
                serde_json::to_string(&status(&w)).unwrap()
            }
            Ok(ControlMsg::StartMatch) => {
                let mut w = world.lock().unwrap();
                w.match_clock.config.disabled_s = 0.0;
                w.start_match();
                serde_json::to_string(&status(&w)).unwrap()
            }
            Ok(ControlMsg::Reset) => {
                let mut w = world.lock().unwrap();
                *w = (ctl.make_world.lock().unwrap())();
                serde_json::to_string(&status(&w)).unwrap()
            }
            Ok(ControlMsg::SetPhase { phase, secs }) => {
                let mut w = world.lock().unwrap();
                w.set_phase(phase, secs.unwrap_or(0.0));
                serde_json::to_string(&status(&w)).unwrap()
            }
            Ok(ControlMsg::ClearPhase) => {
                let mut w = world.lock().unwrap();
                w.clear_phase();
                serde_json::to_string(&status(&w)).unwrap()
            }
            Ok(ControlMsg::AddScore { alliance, points }) => {
                let mut w = world.lock().unwrap();
                w.add_score(alliance, points);
                serde_json::to_string(&status(&w)).unwrap()
            }
            Ok(ControlMsg::Status) => {
                let w = world.lock().unwrap();
                serde_json::to_string(&status(&w)).unwrap()
            }
            Ok(ControlMsg::ListArchetypes) => {
                serde_json::to_string(&serde_json::json!({
                    "archetypes": archetype_names(),
                }))
                .unwrap()
            }
            Ok(ControlMsg::SetLineup { archetypes }) => {
                let new_specs = if archetypes.is_empty() {
                    ctl.defaults.clone()
                } else {
                    match specs_for(&archetypes) {
                        Ok(s) => s,
                        Err(e) => {
                            let reply = format!(
                                "{{\"error\":{}}}",
                                serde_json::to_string(&e.to_string()).unwrap()
                            );
                            if sink.send(Message::Text(reply.into())).await.is_err() {
                                break;
                            }
                            continue;
                        }
                    }
                };
                *ctl.specs.lock().unwrap() = new_specs;
                let mut w = world.lock().unwrap();
                *w = (ctl.make_world.lock().unwrap())();
                serde_json::to_string(&status(&w)).unwrap()
            }
            Err(e) => format!("{{\"error\":{}}}", serde_json::to_string(&e.to_string()).unwrap()),
        };
        if sink.send(Message::Text(reply.into())).await.is_err() {
            break;
        }
    }
    Ok(())
}

/// Accept loop: one task per control client. Runs forever.
pub async fn serve(
    listener: TcpListener,
    world: Arc<Mutex<World>>,
    ctl: Arc<MatchCtl>,
) -> anyhow::Result<()> {
    loop {
        let (stream, addr) = listener.accept().await?;
        let world = world.clone();
        let ctl = ctl.clone();
        tokio::spawn(async move {
            if let Err(e) = handle_conn(stream, world, ctl).await {
                eprintln!("control client {addr} error: {e}");
            }
        });
    }
}

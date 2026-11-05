//! AdvantageScope-compatible NT4 debug publisher (issue #22).
//!
//! The orchestrator already knows the full world state via the sensor
//! broadcast bus; this module republishes that state on a dedicated NT4
//! server (see `nt_server.rs`) so AdvantageScope (or any NT4 client) can
//! connect and introspect every robot exactly like it would a real robot's
//! NetworkTables.
//!
//! Topics (all under `/Sim`):
//!   /Sim/Tick                    int        — physics tick counter
//!   /Sim/Robot<id>/Pose          double[]   — Pose2d [x, y, yaw_rad] (meters)
//!   /Sim/Robot<id>/Pose3d        double[]   — [x,y,z, qw,qx,qy,qz]
//!   /Sim/Robot<id>/Vel           double[]   — world-frame linear velocity
//!   /Sim/Robot<id>/AngVel        double[]   — world-frame angular velocity
//!   /Sim/Robot<id>/Contact       double     — bumper contact impulse
//!
//! Field mapping: the sim world uses y-up ground plane (x,z flat); the NT
//! Pose topics convert to the FRC/AdvantageScope convention (x,y flat).

use crate::nt_server::NtServer;
use crate::SensorFrame;
use std::collections::BTreeMap;
use tokio::sync::broadcast;
use tokio::time::{interval, Duration};

/// Republish rate (matches the viz stream; physics already batches sensor
/// frames at AGENT_EVERY).
const PUB_HZ: f32 = 30.0;

// NT4 type ids (see nt_server::type_name)
const TY_INT: u32 = 0x01;
const TY_DOUBLE: u32 = 0x03;
const TY_DOUBLE_ARR: u32 = 0x13;

/// Extract yaw (rotation about the sim's up axis, +y) from a quaternion
/// stored as [x, y, z, w].
fn yaw_from_quat(q: [f32; 4]) -> f64 {
    let (x, y, z, w) = (q[0] as f64, q[1] as f64, q[2] as f64, q[3] as f64);
    (2.0 * (w * y + x * z)).atan2(w * w + x * x - y * y - z * z)
}

fn darr(xs: &[f64]) -> rmpv::Value {
    rmpv::Value::Array(xs.iter().map(|&v| rmpv::Value::from(v)).collect())
}

/// Start an NT4 server on `port` and republish world state from the sensor
/// broadcast bus. Runs forever; spawn once from the orchestrator.
pub async fn run(
    sensor_tx: broadcast::Sender<(usize, SensorFrame)>,
    port: u16,
) -> std::io::Result<()> {
    let server = NtServer::start(&format!("0.0.0.0:{port}")).await?;
    println!("debug NT4 server for AdvantageScope on :{port} (ws://localhost:{port}/nt/debug)");

    let mut rx = sensor_tx.subscribe();
    let mut latest: BTreeMap<usize, SensorFrame> = BTreeMap::new();
    let mut published: std::collections::BTreeSet<usize> = Default::default();
    let mut tick_published = false;
    let mut iv = interval(Duration::from_secs_f32(1.0 / PUB_HZ));

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
                if !tick_published {
                    server.publish("/Sim/Tick", TY_INT, rmpv::Value::from(0i64));
                    tick_published = true;
                }
                let mut tick: u64 = 0;
                for (&id, f) in &latest {
                    tick = tick.max(f.tick);
                    let base = format!("/Sim/Robot{id}");
                    if published.insert(id) {
                        // first sighting: announce all topics for this robot
                        server.publish(&format!("{base}/Pose"),   TY_DOUBLE_ARR, darr(&[0.0; 3]));
                        server.publish(&format!("{base}/Pose3d"), TY_DOUBLE_ARR, darr(&[0.0; 7]));
                        server.publish(&format!("{base}/Vel"),    TY_DOUBLE_ARR, darr(&[0.0; 3]));
                        server.publish(&format!("{base}/AngVel"), TY_DOUBLE_ARR, darr(&[0.0; 3]));
                        server.publish(&format!("{base}/Contact"), TY_DOUBLE, rmpv::Value::from(0.0f64));
                    }
                    // AdvantageScope Pose2d: [x, y, yaw_rad] on the ground plane.
                    // Sim ground plane is (x, z) with y up.
                    server.set(
                        &format!("{base}/Pose"),
                        darr(&[f.position[0] as f64, f.position[2] as f64,
                               yaw_from_quat(f.rotation)]),
                    );
                    // Pose3d: [x, y, z, qw, qx, qy, qz] — WPILib/AS layout.
                    // Remap sim (x, y-up, z) -> FRC (x, z, y-up).
                    server.set(
                        &format!("{base}/Pose3d"),
                        darr(&[
                            f.position[0] as f64,
                            f.position[2] as f64,
                            f.position[1] as f64,
                            f.rotation[3] as f64,
                            f.rotation[0] as f64,
                            f.rotation[1] as f64,
                            f.rotation[2] as f64,
                        ]),
                    );
                    server.set(
                        &format!("{base}/Vel"),
                        darr(&[
                            f.linvel[0] as f64,
                            f.linvel[2] as f64,
                            f.linvel[1] as f64,
                        ]),
                    );
                    server.set(
                        &format!("{base}/AngVel"),
                        darr(&[
                            f.angvel[0] as f64,
                            f.angvel[2] as f64,
                            f.angvel[1] as f64,
                        ]),
                    );
                    server.set(
                        &format!("{base}/Contact"),
                        rmpv::Value::from(f.contact_impulse as f64),
                    );
                }
                server.set("/Sim/Tick", rmpv::Value::from(tick as i64));
            }
        }
    }
    Ok(())
}

//! Human gamepad agent (issue #13, PLAN C5): drives a robot over the WS
//! protocol like `drive`, but commands come from a physical controller
//! via `gilrs` instead of a scripted controller.
//!
//! Mapping (standard FRC-style gamepad):
//!   left stick   -> drive force vector [x, z] (scaled by --max-force)
//!   right stick  -> yaw torque (right-stick x, scaled by --max-torque)
//!   right trigger / RB -> intake_active
//!   A (south)    -> one-shot shot while held pieces remain (fixed hood
//!                   params; --shot-* flags)
//!
//! Filtering done agent-side per PLAN C5:
//!   * radial deadzone on both sticks (--deadzone, default 0.12)
//!   * slew-rate limiting on force/torque (--slew, units/s of full-scale)
//!   * a disconnected/absent gamepad sends zeroed frames so the robot
//!     coasts to a stop rather than running open-loop.
//!
//! gilrs needs a real controller to produce input; without one the agent
//! still connects and streams zeroed ActuatorFrames (verify: run the
//! server, run this binary, watch the robot stay put).

use anyhow::Result;
use clap::Parser;
use frc_sim_m0::{ActuatorFrame, AgentMsg, ServerMsg, ShotCommand};
use futures_util::{SinkExt, StreamExt};
use gilrs::{Axis, Button, Gilrs};
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    robot_id: usize,
    /// max drive force at full stick deflection (N)
    #[arg(long, default_value_t = 1200.0)]
    max_force: f32,
    /// max yaw torque at full right-stick deflection (N*m)
    #[arg(long, default_value_t = 200.0)]
    max_torque: f32,
    /// radial stick deadzone [0,1)
    #[arg(long, default_value_t = 0.12)]
    deadzone: f32,
    /// slew-rate limit: fraction of full scale per second (force/torque
    /// can't change faster than this per agent tick)
    #[arg(long, default_value_t = 4.0)]
    slew: f32,
    /// orchestrator WS endpoint
    #[arg(long, default_value = "ws://127.0.0.1:9770")]
    server: String,
    /// fixed shot params for the A button
    #[arg(long, default_value_t = 10.0)]
    shot_velocity: f32,
    #[arg(long, default_value_t = 0.7)]
    shot_hood: f32,
}

/// Apply a radial deadzone and rescale so the remaining travel still
/// spans [0, 1] (avoids the "jump at deadzone edge" deadband artifact).
fn deadzone(x: f32, y: f32, dz: f32) -> (f32, f32) {
    let mag = (x * x + y * y).sqrt();
    if mag < dz || mag == 0.0 {
        return (0.0, 0.0);
    }
    let scale = ((mag - dz) / (1.0 - dz) / mag).min(1.0);
    (x * scale, y * scale)
}

/// Slew-rate limit `target` toward `current` by at most `step`.
fn slew(current: f32, target: f32, step: f32) -> f32 {
    current + (target - current).clamp(-step, step)
}

/// Latest gamepad inputs, refreshed by draining gilrs events each tick.
#[derive(Default)]
struct Pad {
    lx: f32,
    ly: f32,
    rx: f32,
    intake: bool,
    shoot: bool,
    connected: bool,
}

impl Pad {
    fn poll(&mut self, gilrs: Option<&mut Gilrs>) {
        let Some(gilrs) = gilrs else {
            *self = Pad::default();
            return;
        };
        // drain pending events
        while gilrs.next_event().is_some() {}
        self.connected = false;
        for (_id, gp) in gilrs.gamepads() {
            self.connected = true;
            self.lx = gp.value(Axis::LeftStickX);
            // gilrs stick y is +up; world +z "forward" matches pushing up
            self.ly = gp.value(Axis::LeftStickY);
            self.rx = gp.value(Axis::RightStickX);
            self.intake = gp.is_pressed(Button::RightTrigger)
                || gp.is_pressed(Button::RightTrigger2);
            self.shoot = gp.is_pressed(Button::South);
            break; // first connected pad wins
        }
        if !self.connected {
            *self = Pad::default();
        }
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();

    let mut gilrs = match Gilrs::new() {
        Ok(g) => Some(g),
        Err(e) => {
            eprintln!(
                "gilrs init failed ({e}); continuing with zeroed inputs \
                 (no gamepad control — robot will coast)"
            );
            None
        }
    };
    let mut pad = Pad::default();
    let mut announced = false;

    let (ws, _) = connect_async(&args.server).await?;
    let (mut sink, mut stream) = ws.split();

    sink.send(Message::Text(
        serde_json::to_string(&AgentMsg::Hello {
            name: format!("gamepad-{}", args.robot_id),
            robot_id: args.robot_id,
            protocol_version: frc_sim_m0::PROTOCOL_VERSION,
        })
        .unwrap()
        .into(),
    ))
    .await?;

    println!(
        "gamepad agent for robot {} (max_force={} deadzone={} slew={}/s)",
        args.robot_id, args.max_force, args.deadzone, args.slew
    );

    // commanded (post-slew) outputs, per agent tick (50 Hz)
    let dt = 1.0 / 50.0;
    let (mut cmd_fx, mut cmd_fz, mut cmd_t) = (0.0f32, 0.0f32, 0.0f32);

    while let Some(Ok(msg)) = stream.next().await {
        let Message::Text(txt) = msg else { continue };
        let Ok(ServerMsg::Sensor(s)) = serde_json::from_str::<ServerMsg>(&txt) else {
            continue;
        };

        pad.poll(gilrs.as_mut());
        if !pad.connected && !announced {
            announced = true;
            eprintln!("no gamepad connected — sending zeroed frames");
        } else if pad.connected && announced {
            announced = false;
            eprintln!("gamepad connected");
        }

        let (sx, sy) = deadzone(pad.lx, pad.ly, args.deadzone);
        let (rx, _) = deadzone(pad.rx, 0.0, args.deadzone);

        let step = args.slew * dt;
        cmd_fx = slew(cmd_fx, sx * args.max_force, args.max_force * step);
        cmd_fz = slew(cmd_fz, sy * args.max_force, args.max_force * step);
        cmd_t = slew(cmd_t, rx * args.max_torque, args.max_torque * step);

        let shoot = if pad.shoot && s.pieces_held > 0 {
            Some(ShotCommand {
                launch_velocity: args.shot_velocity,
                hood_angle: args.shot_hood,
                turret_yaw: 0.0,
                launch_height: 0.3,
            })
        } else {
            None
        };

        sink.send(Message::Text(
            serde_json::to_string(&AgentMsg::Actuator(ActuatorFrame {
                force: [cmd_fx, cmd_fz],
                torque: cmd_t,
                modules: None,
                intake_active: pad.intake,
                shoot,
            }))
            .unwrap()
            .into(),
        ))
        .await?;
    }
    Ok(())
}

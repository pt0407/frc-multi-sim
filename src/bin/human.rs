//! Human agent (issue #13): drive a robot with a gamepad (gilrs) or the
//! keyboard (crossterm) — both feed the same input state, so either works.
//!
//! Gamepad: left stick = translate (holonomic), right stick X = turn,
//! South/A = intake toggle, West/X = shoot, Start = quit.
//! Keyboard: WASD = translate, Q/E = turn, I = intake toggle, Space = shoot,
//! Ctrl-C = quit.
use anyhow::Result;
use clap::Parser;
use frc_sim_m0::{ActuatorFrame, AgentMsg, ServerMsg, PROTOCOL_VERSION, ShotCommand};
use futures_util::{SinkExt, StreamExt};
use std::sync::{Arc, Mutex};
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    robot_id: usize,
    /// max drive force (N)
    #[arg(long, default_value_t = 1200.0)]
    max_force: f32,
    /// max yaw torque (N·m)
    #[arg(long, default_value_t = 150.0)]
    max_torque: f32,
    /// shot parameters for the shoot button
    #[arg(long, default_value_t = 12.0)]
    launch_velocity: f32,
    #[arg(long, default_value_t = 0.6)]
    hood_angle: f32,
    /// archetype TOML; its max_force overrides --max-force
    #[arg(long)]
    archetype: Option<std::path::PathBuf>,
    /// disable the keyboard fallback (gamepad only)
    #[arg(long, default_value_t = false)]
    no_keyboard: bool,
}

/// Latest human input, written by whichever device produced it last.
#[derive(Default, Clone, Copy)]
struct Input {
    /// strafe right / forward, each in [-1, 1]
    x: f32,
    y: f32,
    /// yaw torque demand in [-1, 1]
    turn: f32,
    intake: bool,
    /// rising-edge shoot request; consumed by the agent loop
    shoot: bool,
}

fn deadzone(v: f32) -> f32 {
    // sticks rest around +-0.05; rescale the rest of the range
    let v = if v.abs() < 0.12 { 0.0 } else { v };
    v.clamp(-1.0, 1.0)
}

/// Poll a gilrs gamepad forever, merging events into `input`.
fn gamepad_loop(gilrs: gilrs::Gilrs, input: Arc<Mutex<Input>>, quit: Arc<Mutex<bool>>) {
    let mut gilrs = gilrs;
    loop {
        if *quit.lock().unwrap() {
            return;
        }
        let Some(ev) = gilrs.next_event() else {
            std::thread::sleep(std::time::Duration::from_millis(2));
            continue;
        };
        let mut inp = input.lock().unwrap();
        use gilrs::{Axis, Button, EventType};
        match ev.event {
            EventType::AxisChanged(Axis::LeftStickX, v, _) => inp.x = deadzone(v),
            EventType::AxisChanged(Axis::LeftStickY, v, _) => inp.y = -deadzone(v), // +Y is down on most pads
            EventType::AxisChanged(Axis::RightStickX, v, _) => inp.turn = deadzone(v),
            EventType::ButtonPressed(Button::South, _) => inp.intake = !inp.intake,
            EventType::ButtonPressed(Button::West, _) => inp.shoot = true,
            EventType::ButtonPressed(Button::Start, _) => *quit.lock().unwrap() = true,
            _ => {}
        }
    }
}

/// Poll the keyboard forever (crossterm raw mode).
fn keyboard_loop(input: Arc<Mutex<Input>>, quit: Arc<Mutex<bool>>) -> Result<()> {
    use crossterm::event::{read, Event, KeyCode, KeyEventKind, KeyModifiers};
    use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
    enable_raw_mode()?;
    let mut held = [false; 8]; // w a s d q e i space
    loop {
        if *quit.lock().unwrap() {
            disable_raw_mode().ok();
            return Ok(());
        }
        match read() {
            Ok(Event::Key(k)) if k.kind != KeyEventKind::Release => {
                let idx = match k.code {
                    KeyCode::Char('w') | KeyCode::Up => Some(0),
                    KeyCode::Char('s') | KeyCode::Down => Some(2),
                    KeyCode::Char('a') | KeyCode::Left => Some(1),
                    KeyCode::Char('d') | KeyCode::Right => Some(3),
                    KeyCode::Char('q') => Some(4),
                    KeyCode::Char('e') => Some(5),
                    KeyCode::Char('i') | KeyCode::Char('I') => Some(6),
                    KeyCode::Char(' ') => Some(7),
                    KeyCode::Char('c') if k.modifiers.contains(KeyModifiers::CONTROL) => {
                        *quit.lock().unwrap() = true;
                        None
                    }
                    _ => None,
                };
                if let Some(i) = idx {
                    // edge-trigger for toggles/shoot; level for motion
                    let was = held[i];
                    held[i] = !was;
                    let mut inp = input.lock().unwrap();
                    match i {
                        6 => {
                            if !was {
                                inp.intake = !inp.intake;
                            }
                        }
                        7 => {
                            if !was {
                                inp.shoot = true;
                            }
                        }
                        _ => {}
                    }
                    inp.y = deadzone(held[0] as i32 as f32 - held[2] as i32 as f32);
                    inp.x = deadzone(held[3] as i32 as f32 - held[1] as i32 as f32);
                    inp.turn = deadzone(held[5] as i32 as f32 - held[4] as i32 as f32);
                }
            }
            Ok(_) => {}
            Err(_) => std::thread::sleep(std::time::Duration::from_millis(5)),
        }
    }
}

/// Quaternion [x,y,z,w] -> yaw about +Y, and forward vector in the x/z plane.
fn yaw_and_forward(q: [f32; 4]) -> (f32, [f32; 2]) {
    let [x, y, z, w] = q;
    // forward = R * (1,0,0); keep x/z components (y is up)
    let fx = 1.0 - 2.0 * (y * y + z * z);
    let fz = 2.0 * (x * z - w * y);
    (fz.atan2(fx), [fx, fz])
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let arch = frc_sim_m0::archetype::Archetype::load_opt(&args.archetype)?;
    let max_force = arch.as_ref().map(|a| a.max_force).unwrap_or(args.max_force);

    let input = Arc::new(Mutex::new(Input::default()));
    let quit = Arc::new(Mutex::new(false));

    // gamepad thread — spawn even if no pad is attached; gilrs just idles
    {
        let input = input.clone();
        let quit = quit.clone();
        std::thread::spawn(move || {
            if let Ok(g) = gilrs::Gilrs::new() {
                gamepad_loop(g, input, quit);
            }
        });
    }
    if !args.no_keyboard {
        let input = input.clone();
        let quit = quit.clone();
        std::thread::spawn(move || {
            let _ = keyboard_loop(input, quit);
        });
    }

    let (ws, _) = connect_async("ws://127.0.0.1:9770").await?;
    let (mut sink, mut stream) = ws.split();
    sink.send(Message::Text(
        serde_json::to_string(&AgentMsg::Hello {
            name: format!("human-{}", args.robot_id),
            robot_id: args.robot_id,
            protocol_version: PROTOCOL_VERSION,
        })
        .unwrap()
        .into(),
    ))
    .await?;

    println!(
        "robot {} under human control — gamepad: LS move, RSx turn, A intake, X shoot, Start quit | keys: WASD/QE/I/Space/Ctrl-C",
        args.robot_id
    );

    while let Some(Ok(msg)) = stream.next().await {
        if *quit.lock().unwrap() {
            break;
        }
        let Message::Text(txt) = msg else { continue };
        let Ok(ServerMsg::Sensor(s)) = serde_json::from_str::<ServerMsg>(&txt) else {
            continue;
        };

        let (yaw, fwd) = yaw_and_forward(s.rotation);
        let (cos, sin) = (yaw.cos(), yaw.sin());
        let inp = *input.lock().unwrap();

        // stick (x=right, y=fwd) in robot frame -> world via yaw.
        // right vector for yaw θ is (sin θ, -cos θ) in (x, z).
        let fx = inp.y * fwd[0] + inp.x * sin;
        let fz = inp.y * fwd[1] - inp.x * cos;
        let mag = (fx * fx + fz * fz).sqrt();
        let (fx, fz) = if mag > 1.0 {
            (fx * max_force / mag, fz * max_force / mag)
        } else {
            (fx * max_force, fz * max_force)
        };

        let shoot = if inp.shoot {
            input.lock().unwrap().shoot = false; // consume the edge
            Some(ShotCommand {
                launch_velocity: args.launch_velocity,
                hood_angle: args.hood_angle,
                turret_yaw: 0.0,
                launch_height: 0.5,
            })
        } else {
            None
        };

        sink.send(Message::Text(
            serde_json::to_string(&AgentMsg::Actuator(ActuatorFrame {
                force: [fx, fz],
                torque: inp.turn * args.max_torque,
                modules: None,
                intake_active: inp.intake,
                shoot,
            }))
            .unwrap()
            .into(),
        ))
        .await?;
    }
    *quit.lock().unwrap() = true;
    Ok(())
}

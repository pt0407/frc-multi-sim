use anyhow::Result;
use clap::Parser;
use futures_util::{SinkExt, StreamExt};
use frc_sim_m0::drivetrain::{rot2, yaw_from_quat, Drivetrain};
use frc_sim_m0::{ActuatorFrame, AgentMsg, SensorFrame, ServerMsg};
use frc_sim_m0::nt_client_min::NtClient;
use rapier3d::prelude::*;
use std::sync::{Arc, Mutex};
use tokio::net::TcpListener;
use tokio::sync::mpsc;
use tokio::time::{interval, Duration};
use tokio_tungstenite::tungstenite::Message;

const PHYSICS_HZ: f32 = 400.0;
const AGENT_EVERY: u32 = 8; // 50 Hz agent tick
const NUM_ROBOTS: usize = 2;
const ROBOT_HALF: f32 = 0.4; // ~0.8m square chassis
const ROBOT_MASS: f32 = 60.0; // kg
const FIELD_HALF_X: f32 = 8.25;
const FIELD_HALF_Z: f32 = 4.0;

struct World {
    pipeline: PhysicsPipeline,
    gravity: Vector<f32>,
    integration: IntegrationParameters,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd: CCDSolver,
    query: QueryPipeline,
    chassis: Vec<RigidBodyHandle>,
    chassis_colliders: Vec<ColliderHandle>,
    drivetrains: Vec<Drivetrain>,
}

impl World {
    fn new() -> Self {
        let mut w = World {
            pipeline: PhysicsPipeline::new(),
            gravity: vector![0.0, -9.81, 0.0],
            integration: IntegrationParameters {
                dt: 1.0 / PHYSICS_HZ,
                ..Default::default()
            },
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd: CCDSolver::new(),
            query: QueryPipeline::new(),
            chassis: Vec::new(),
            chassis_colliders: Vec::new(),
            // default profile: 4-module swerve per robot (issue #2). The
            // legacy `Drivetrain::simple` sliding box remains available as a
            // fallback profile. Config-file selection is issue #8.
            drivetrains: (0..NUM_ROBOTS)
                .map(|_| Drivetrain::swerve(ROBOT_MASS, ROBOT_HALF))
                .collect(),
        };

        // ground
        let ground = w.bodies.insert(RigidBodyBuilder::fixed());
        w.colliders.insert_with_parent(
            ColliderBuilder::cuboid(20.0, 0.1, 20.0)
                .translation(vector![0.0, -0.1, 0.0])
                .friction(1.0)
                .build(),
            ground,
            &mut w.bodies,
        );
        // field walls
        for (x, z, hx, hz) in [
            (0.0, -FIELD_HALF_Z, FIELD_HALF_X, 0.05),
            (0.0, FIELD_HALF_Z, FIELD_HALF_X, 0.05),
            (-FIELD_HALF_X, 0.0, 0.05, FIELD_HALF_Z),
            (FIELD_HALF_X, 0.0, 0.05, FIELD_HALF_Z),
        ] {
            w.colliders.insert_with_parent(
                ColliderBuilder::cuboid(hx, 0.6, hz)
                    .translation(vector![x, 0.3, z])
                    .friction(0.6)
                    .build(),
                ground,
                &mut w.bodies,
            );
        }

        // chassis: robot 0 at (-4, 0), robot 1 at (4, 0)
        for (i, x) in [-4.0f32, 4.0].iter().enumerate() {
            let body = w.bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(vector![*x, ROBOT_HALF + 0.05, 0.0])
                    .additional_mass(ROBOT_MASS)
                    .linear_damping(1.0)
                    .angular_damping(4.0)
                    // lock X/Z rotation: chassis stays upright (2.5D drive, 3D contacts)
                    .enabled_rotations(false, true, false)
                    .ccd_enabled(true)
                    .build(),
            );
            let col = w.colliders.insert_with_parent(
                ColliderBuilder::cuboid(ROBOT_HALF, ROBOT_HALF, ROBOT_HALF)
                    .friction(0.9)
                    .restitution(0.1)
                    .density(1.0)
                    .build(),
                body,
                &mut w.bodies,
            );
            w.chassis.push(body);
            w.chassis_colliders.push(col);
            println!("robot {} spawned at x={}", i, x);
        }
        w
    }

    fn step(&mut self, commands: &[Option<ActuatorFrame>; NUM_ROBOTS]) {
        for (i, handle) in self.chassis.iter().enumerate() {
            if let Some(cmd) = &commands[i] {
                // per-wheel traction model: resolve the actuator command into
                // module forces (grip-limited), then apply them off-center so
                // yaw torque emerges from the force placement.
                let (yaw, pos) = {
                    let body = &self.bodies[*handle];
                    let r = body.rotation();
                    (yaw_from_quat([r.i, r.j, r.k, r.w]), *body.translation())
                };
                let resolved = self.drivetrains[i].resolve(cmd, yaw);
                let body = &mut self.bodies[*handle];
                body.reset_forces(true);
                body.reset_torques(true);
                for f in &resolved {
                    // apply at the wheel's ground contact point
                    let p = rot2(f.local_point, yaw);
                    body.add_force_at_point(
                        vector![f.force[0], 0.0, f.force[1]],
                        point![pos.x + p[0], 0.05, pos.z + p[1]],
                        true,
                    );
                    if f.torque != 0.0 {
                        body.add_torque(vector![0.0, f.torque, 0.0], true);
                    }
                }
            } else {
                let body = &mut self.bodies[*handle];
                body.reset_forces(true);
                body.reset_torques(true);
            }
        }
        self.pipeline.step(
            &self.gravity,
            &self.integration,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd,
            Some(&mut self.query),
            &(),
            &(),
        );
    }

    fn sensor_frame(&self, robot: usize, tick: u64) -> SensorFrame {
        let body = &self.bodies[self.chassis[robot]];
        let t = body.translation();
        let r = body.rotation();
        let lv = body.linvel();
        let av = body.angvel();
        // only robot-vs-robot contacts (filter out ground/walls)
        let other = self.chassis_colliders[1 - robot];
        let contact_impulse: f32 = self
            .narrow_phase
            .contact_pairs_with(self.chassis_colliders[robot])
            .filter(|p| p.collider1 == other || p.collider2 == other)
            .map(|p| p.manifolds.iter().map(|m| m.points.len()).sum::<usize>())
            .sum::<usize>() as f32;
        SensorFrame {
            tick,
            position: [t.x, t.y, t.z],
            rotation: [r.i, r.j, r.k, r.w],
            linvel: [lv.x, lv.y, lv.z],
            angvel: [av.x, av.y, av.z],
            contact_impulse,
        }
    }
}

type Commands = Arc<Mutex<[Option<ActuatorFrame>; NUM_ROBOTS]>>;

#[derive(Parser)]
struct Args {
    /// NT4 server ports to bridge, indexed by robot id (e.g. --nt 5810 --nt 5820)
    #[arg(long)]
    nt: Vec<u16>,
}

/// NT4 bridge: orchestrator acts as an NT client to a robot's own NT server —
/// the same topology as a real WPILib sim. Subscribes to actuator topics,
/// publishes sensor topics. Uses our minimal client (`nt_client` 0.5.1's
/// GenericPublisher has a pubuid-latching bug, see issue #1).
async fn nt_bridge(
    robot_id: usize,
    port: u16,
    commands: Commands,
    sensor_tx: tokio::sync::broadcast::Sender<(usize, SensorFrame)>,
) {
    loop {
        let Ok(client) = NtClient::connect(port).await else {
            tokio::time::sleep(Duration::from_millis(500)).await;
            continue;
        };
        println!("robot {} nt bridge connected -> :{}", robot_id, port);
        for n in ["pose_x", "pose_z", "yaw", "vel_x", "vel_z", "contact"] {
            client.publish(&format!("/sim/{n}"), 3);
        }
        // poll actuator topics into the command slot
        {
            let client = client.clone();
            let commands = commands.clone();
            tokio::spawn(async move {
                loop {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                    let fx = client.get_f64("/sim/cmd_fx").unwrap_or(0.0) as f32;
                    let fz = client.get_f64("/sim/cmd_fz").unwrap_or(0.0) as f32;
                    let tq = client.get_f64("/sim/cmd_torque").unwrap_or(0.0) as f32;
                    commands.lock().unwrap()[robot_id] = Some(ActuatorFrame {
                        force: [fx, fz],
                        torque: tq,
                        modules: None,
                    });
                }
            });
        }
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
                s.position[0], s.position[2], yaw,
                s.linvel[0], s.linvel[2], s.contact_impulse,
            ];
            for (i, n) in ["pose_x", "pose_z", "yaw", "vel_x", "vel_z", "contact"]
                .iter()
                .enumerate()
            {
                client.set_f64(&format!("/sim/{n}"), vals[i] as f64);
            }
        }
        eprintln!("robot {} nt bridge dropped, reconnecting", robot_id);
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let listener = TcpListener::bind("127.0.0.1:9770").await?;
    println!("orchestrator listening on ws://127.0.0.1:9770/robot/<id>");

    let commands: Commands = Arc::new(Mutex::new([None, None]));
    let (sensor_tx, _) = tokio::sync::broadcast::channel::<(usize, SensorFrame)>(256);
    let (done_tx, mut done_rx) = mpsc::channel::<usize>(8);

    for (robot_id, port) in args.nt.iter().enumerate() {
        if robot_id >= NUM_ROBOTS {
            break;
        }
        tokio::spawn(nt_bridge(
            robot_id,
            *port,
            commands.clone(),
            sensor_tx.clone(),
        ));
        println!("nt bridge for robot {} -> :{}", robot_id, port);
    }

    // physics loop
    {
        let commands = commands.clone();
        let sensor_tx = sensor_tx.clone();
        tokio::spawn(async move {
            let mut world = World::new();
            let mut tick: u64 = 0;
            let mut iv = interval(Duration::from_micros(2500)); // 400 Hz wall-clock
            loop {
                iv.tick().await;
                let cmds = commands.lock().unwrap().clone();
                world.step(&cmds);
                if tick % AGENT_EVERY as u64 == 0 {
                    for r in 0..NUM_ROBOTS {
                        let _ = sensor_tx.send((r, world.sensor_frame(r, tick)));
                    }
                }
                tick += 1;
            }
        });
    }

    loop {
        tokio::select! {
            accepted = listener.accept() => {
                let (stream, addr) = accepted?;
                let commands = commands.clone();
                let mut sensor_rx = sensor_tx.subscribe();
                let done_tx = done_tx.clone();
                tokio::spawn(async move {
                    let Ok(ws) = tokio_tungstenite::accept_async(stream).await else {
                        return;
                    };
                    let (mut sink, mut stream) = ws.split();

                    // first message must be Hello{ robot_id }
                    let robot_id = match stream.next().await {
                        Some(Ok(Message::Text(txt))) => {
                            match serde_json::from_str::<AgentMsg>(&txt) {
                                Ok(AgentMsg::Hello { robot_id, .. }) if robot_id < NUM_ROBOTS => {
                                    robot_id
                                }
                                _ => return,
                            }
                        }
                        _ => return,
                    };
                    println!("agent {} connected -> robot {}", addr, robot_id);
                    sink.send(Message::Text(
                        serde_json::to_string(&ServerMsg::Welcome { robot_id }).unwrap().into(),
                    ))
                    .await
                    .ok();

                    let cmd_writer = commands.clone();
                    let mut read_task = tokio::spawn(async move {
                        while let Some(Ok(msg)) = stream.next().await {
                            if let Message::Text(txt) = msg {
                                if let Ok(AgentMsg::Actuator(a)) =
                                    serde_json::from_str::<AgentMsg>(&txt)
                                {
                                    cmd_writer.lock().unwrap()[robot_id] = Some(a);
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
                            Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                            Err(_) => break,
                        }
                    }
                    read_task.abort();
                    commands.lock().unwrap()[robot_id] = None;
                    let _ = done_tx.send(robot_id).await;
                    println!("robot {} disconnected", robot_id);
                });
            }
            Some(id) = done_rx.recv() => {
                eprintln!("agent {} gone", id);
            }
        }
    }
}

# FRC Multi-Agent Simulation Platform

A deterministic 3D physics orchestrator for FRC match simulation: **6 robots, each
controlled by an independent process** — real WPILib robot code (via its own NT4
server), scripted agents, RL policies, or humans — sharing one authoritative
Rapier physics world.

Status: **M0 (proof of concept)** — two robots colliding/pushing in a shared
Rapier world, each believing it's the only robot.

See [../ARCHITECTURE.md](../ARCHITECTURE.md) for the full design.

## Components

| Binary | Role |
|---|---|
| `server` | Physics orchestrator: Rapier3D world, field + walls, N chassis at 400 Hz physics / 50 Hz agent tick. Two agent backends: raw WebSocket protocol, and **NT4 client bridge** (connects to each robot's own NT server, like a real WPILib sim). |
| `drive` | Standalone agent process (WebSocket protocol): joystick-style force commands via a P-controller. |
| `robot_process` | Mock WPILib robot: hosts its own **NT4 server** on a port, publishes actuator topics, polls sensor topics — the same topology a real WPILib sim exposes. |
| `nt_probe` | NT4 debug probe: subscribes to all topics and prints updates. |

`src/nt_server.rs` is a minimal spec-conformant NT4 server (WS + msgpack +
announce/subscribe/publish), just enough for `nt_client`/`ntcore` clients.

## NT4 topic contract

Orchestrator → robot: `/sim/pose_x`, `/sim/pose_z`, `/sim/yaw`,
`/sim/vel_x`, `/sim/vel_z`, `/sim/contact` (double)
Robot → orchestrator: `/sim/cmd_fx`, `/sim/cmd_fz`, `/sim/cmd_torque` (double)

## Run (NT4 head-on collision demo)

```bash
cargo run --release --bin server -- --nt 5810 --nt 5820
cargo run --release --bin robot_process -- --port 5810 --target 4 0 --max-force 2500
cargo run --release --bin robot_process -- --port 5820 --target -4 0 --max-force 600
```

Both robots drive toward each other; the weaker one gets pushed back to its wall.

## Notes / known issues

- `nt_client` 0.5.1's `GenericPublisher` latches the first announce carrying a
  pubuid regardless of topic name — publishers must be created sequentially.
- Chassis rotation is yaw-locked (2.5D contacts on 3D bodies); full 6-DOF later.
- WPILib Java sim can't run on this machine yet (needs JDK + Gradle).

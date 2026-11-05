# FRC Multi-Agent Simulation Platform

A deterministic 3D physics orchestrator for FRC match simulation: **6 robots, each
controlled by an independent process** — real WPILib robot code (via its own NT4
server), scripted agents, RL policies, or humans — sharing one authoritative
Rapier physics world.

Status: **M1** — the default lineup is **6 robots in two alliance rows of 3**
(even robot ids on -x, odd on +x; alliance = `robot_id % 2`), each believing
it's the only robot. `--archetype` overrides any slot; extra `--archetype`
flags extend the lineup past 6 with the same row layout.

See [../ARCHITECTURE.md](../ARCHITECTURE.md) for the full design.

## Components

| Binary | Role |
|---|---|
| `server` | Physics orchestrator: Rapier3D world, field + walls, N chassis at 400 Hz physics / 50 Hz agent tick. Two agent backends: raw WebSocket protocol, and **NT4 client bridge** (connects to each robot's own NT server, like a real WPILib sim). |
| `drive` | Standalone agent process (WebSocket protocol): joystick-style force commands via a P-controller. |
| `gamepad` | Human-in-the-loop agent (WebSocket protocol): physical gamepad via `gilrs` — left stick = drive force, right stick = yaw torque, right trigger/RB = intake, A = shoot. Deadzone + slew-rate limiting agent-side; sends zeroed frames when no pad is connected. |
| `robot_process` | Mock WPILib robot: hosts its own **NT4 server** on a port, publishes actuator topics, polls sensor topics — the same topology a real WPILib sim exposes. |
| `nt_probe` | NT4 debug probe: subscribes to all topics and prints updates. |
| `web/` | Babylon.js browser viz: connects to the spectator stream and renders the field, walls, and color-coded robots with an orbit camera. |

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

## Browser viz

The server also exposes a read-only **spectator stream** on
`ws://127.0.0.1:9771`: on connect it sends a `config` message (field + robot
dimensions), then broadcasts `{type:"state", tick, robots:[...]}` JSON at ~30 Hz
with every robot's pose, velocities, and contact impulse.

```bash
cargo run --bin server
cd web && npm install && npm run dev   # open http://localhost:5173
# optional: download the real 2026 REBUILT field model (~18 MB, see
# web/ASSETS.md for provenance). Without it the client falls back to
# primitive walls/obstacles — physics is identical either way.
npm run fetch-assets
# in another shell, drive a robot and watch it move in the browser:
cargo run --bin drive -- --robot-id 0 --target 4 0
```

## Determinism / replay

The physics loop is step-count driven (issue #5). `--time-mode` selects
pacing: `realtime` (400 Hz wall-clock, default), `lockstep` (agent tick
barrier — waits for each connected WS agent's fresh command), `fast`
(no sleeps, for RL).

```bash
# record inputs + state samples, then re-run them headless
cargo run --release --bin server -- --seed 7 --record run.fmsl
cargo run --release --bin server -- --replay run.fmsl --diff run.fmsl
```

Log files carry a header (seed, protocol version, per-robot specs, config
hash) so a replay rebuilds the exact world. `--diff` exits non-zero with
the first divergent tick + magnitude. Agents must send
`protocol_version` in `Hello` (currently 1) or they're rejected.

## Game configs

`games/*.toml` is the single per-game surface (issue #23): one file names
the field layout, the game-piece spawn/scoring table, and the `[match]`
timing/endgame rules, so

```bash
cargo run --bin server -- --game games/rebuilt.toml
```

is equivalent to passing `--field fields/rebuilt.toml --pieces
games/fuel.toml --game games/rebuilt.toml` (the explicit flags still win
when both are given; `games/rebuilt.toml` is auto-picked when present).
Archetype TOMLs may declare `compatible_games = ["rebuilt"]`; the server
warns when an archetype is paired with a game not on its list.

Drive a robot by hand with a USB/Bluetooth gamepad:

```bash
cargo run --bin gamepad -- --robot-id 0 --max-force 1500
```

## Notes / known issues

- `nt_client` 0.5.1's `GenericPublisher` latches the first announce carrying a
  pubuid regardless of topic name — publishers must be created sequentially.
- Chassis rotation is yaw-locked (2.5D contacts on 3D bodies); full 6-DOF later.
- WPILib Java sim can't run on this machine yet (needs JDK + Gradle).

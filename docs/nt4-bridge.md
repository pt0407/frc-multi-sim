# NT4 robot bridge — verified end-to-end

The orchestrator (`server --nt <port>`) runs an NT4 *client* against each
robot process's own NT4 *server* — the same topology as a WPILib robot on
a real field (robot hosts NT4, driver station / sim connects as client).

## Wire contract

Robot process (NT4 server on `--port`) publishes actuator topics:

- `/sim/cmd_fx`, `/sim/cmd_fz` — world-frame drive force, newtons
- `/sim/cmd_torque` — yaw torque

Orchestrator publishes sensor topics (client-publish, doubles):

- `/sim/pose_x`, `/sim/pose_z`, `/sim/yaw`, `/sim/vel_x`, `/sim/vel_z`,
  `/sim/contact`

The bridge polls actuator topics every 5 ms into `CommandSlots` and writes
sensor frames back each agent tick. On disconnect the command slot is
cleared and the bridge reconnects with exponential backoff (500 ms → 5 s).

## Verified recipe (mock robot process)

```sh
cargo build --release --bins

# 1. authoritative world — NOTE: games/rebuilt.toml sets
#    disabled_s = 99999 (staging mode); the match must be armed or
#    actuator commands are ignored, exactly like a real field.
./target/release/server --seed 7 --num-robots 1 --archetype 2910 \
    --pieces games/fuel.toml --nt 5811 &

# 2. robot process hosting its own NT4 server
./target/release/robot_process --port 5811 --target 0 0 \
    --archetype archetypes/2910.toml &

# 3. arm the match (control WS on :9772)
python3 - <<'EOF'
import websocket, json, time
w = websocket.create_connection("ws://127.0.0.1:9772")
w.send(json.dumps({"op": "start_match"}))
w.close()
EOF

# 4. observe — robot drives from spawn (x=-5.1, z=-1.517) to (0,0)
./target/release/nt_probe 5811   # /sim/pose_x goes -5.1 -> ~0.0
```

Observed: `/sim/cmd_fx` saturated at 1200 N, `/sim/pose_x` travelled
~-5.1 -> ~-0.03 and settled on target. Verified 2025-11.

## Gotchas found while proving this

- **Port collisions**: any process already bound to the NT port wins the
  connection silently (a leftover WPILib jar on :5810 hijacked the bridge
  for a day). Check `lsof -nP -iTCP:<port>` before debugging data flow.
- **`disabled_s = 99999`** in `games/rebuilt.toml` is intentional staging
  mode — send `{"op":"start_match"}` (or `set_phase`) on `:9772` or every
  command is dropped by the phase gate in `world::step`.
- **Spawn clearance**: stations are z=±1.517 and 0 at x=±5.1. The center
  station is wider than the 1.18 m bump corridor, so center robots must
  navigate around the ramps — same as a real field.
- `sim --script` headless runs need an enabled `[match]` config
  (`disabled_s = 0`) or commands are ignored there too.

## Status vs. real WPILib

This proves the *transport and data path*: real NT4 WS + msgpack, sensor
feedback loop, command polling, physics response. `robot_process` is a
mock PD controller, **not** WPILib. Running actual robot code (e.g. the
2910 jar) additionally requires the robot program to subscribe to
`/sim/pose_*` and publish `/sim/cmd_*` — i.e. a sim adapter in the robot
code, or an NT relay that maps WPILib topics (`/SmartDashboard/*`,
`DriverStation/*`) onto the `/sim/` contract.

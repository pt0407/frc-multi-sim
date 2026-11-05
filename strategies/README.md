# Strategy format (issue #59)

Machine-readable per-robot strategy — the ingest seam for external tools
(video scouting, match analysis). `strategy_agent` executes it over the
standard WS agent protocol.

## Schema

```json
{
  "name": "my-strategy",          // optional label
  "loop_actions": false,          // optional: repeat queue forever
  "actions": [ ... ]
}
```

### Actions (executed in order)

| type | fields | behavior |
|---|---|---|
| `goto` | `x`, `z` | P-drive to point on the ground plane; advances when within `--arrive` (default 0.3 m) |
| `intake` | `active` | set intake on/off (capture only when arm deployed, if mech equipped) |
| `wait` | `seconds` | hold position for N seconds |
| `shoot` | `launch_velocity`, `hood_angle`, `turret_yaw`, `launch_height` | one-shot ballistic launch |
| `hold` | — | hold position forever (terminal) |

## Run

```
cargo run --bin strategy_agent -- --robot-id 0 --strategy strategies/example.json [--max-force 1200] [--server ws://...]
```

One process per robot — run six for a full scripted match.

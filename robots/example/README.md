# Robot package convention

A robot package is a self-contained folder the sim loads as one unit:

```
robots/<name>/
  archetype.toml      # physics + mechanism spec (required)
  model.glb           # visual/collision mesh (optional; KitBot default)
  autos/
    auto.json         # strategy file(s) — chosen at match time
  agent/
    <code>            # custom agent (python/node/rust binary)
```

`server --robot robots/<name>` per slot; `match.sh` picks `autos/<slot>.json`
as the auto routine, then the agent (if any) drives teleop.

## Writing a custom agent (any language)

Connect to `ws://127.0.0.1:9770/robot/<id>` and exchange JSON:

- Server sends `SensorFrame` at 50Hz: `tick, position, rotation,
  linvel, held, mech_angle, match_state, match_time, hub_active,
  vision_pose (noisy), contact_impulse, score`
- Agent replies `ActuatorFrame` per tick: `force:[x,z] newtons,
  torque, modules?, intake_active, shoot:{launch_velocity,
  hood_angle,turret_yaw,launch_height}, mech:[{joint,target}], climb`

See `agents/example_client.py` for a working 50-line Python client.

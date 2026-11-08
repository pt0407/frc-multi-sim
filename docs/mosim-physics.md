# MoSim (MoSimulator) extracted physics constants

Source: decompiled managed assemblies from the installed MoSim app
(`/tmp/mosim-src/`, RobotFramework/GameSystems/Rebuilt assemblies).
These are MoSim's hand-tuned values — useful realism targets, not FRC rules.

## Roller motors (`InteractionRoller.Motor` presets)

| Preset | Stall torque (N·m) | Max RPM |
|---|---|---|
| FullSize | 7.0 | 6000 |
| Medium | 4.5 | 5500 |
| Small | 1.5 | 11000 |
| Tiny | 0.1 | 15000 |
| Shooter | 7.5 | 11000 |
| Unstable | 25.0 | ∞ |

## FlywheelBehaviour (shooter spin model)

- `spinUpTime = 1.2 s`, `spinDownTime = 0.6 s` — SmoothDamp profiles,
  **asymmetric**: spools down faster than it spools up
- `ballDipFraction = 0.08` — each ball contact costs 8% of current RPM
- `recoveryTime = 0.25 s` — dip recovers over this time
- `atSpeedTolerance = 0.02` — IsAtSpeed = within 2% of target RPM
- RPM floor for at-speed check: 10

## Game pieces

- Fuel radius = 0.075 m (matches our `FUEL_RADIUS`)
- Piece mass/friction live in Unity prefabs (not in managed code);
  roller contact uses `coreFriction = 1.0`, `coreBounce = 0`

## Architecture notes (already implemented)

- Robots = Unity prefabs + mechanism MonoBehaviours (roller/flywheel/
  intake/joint/turret/elevator) — our `[[joint]]` TOML specs are the
  data-driven equivalent
- Intake = trigger volume → piece rides a node graph → ballistic release
  (our piece-carry path system, deprioritized)
- `Motor` = trapezoidal vel/accel profile; `GenericJoint` = PID position

## CAD extraction

`payload.dat` unpacks to the full Unity asset set (level0-4,
sharedassets0-4, resources). Field visuals extract cleanly via UnityPy
(`/tmp/mosim-unity/export_field.py` → `/tmp/mosim-field/mosim-field.glb`,
real FIRST field CAD). Robot models are NOT in the scene assets — they
ship as AES-256 encrypted AssetBundles under
`StreamingAssets/ProtectedRobots/macOS/*.enc`, decrypted at runtime by
`libipc_plugin.dylib` using a session key handed over IPC by MoSim's
launcher (`MOSIM_IPC_PORT`). Mechanism *constants* survive in the
decompiled robot scripts — see `docs/mosim-robots.md`.

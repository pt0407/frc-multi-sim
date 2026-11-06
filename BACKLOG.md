# frc-multi-sim backlog

## P0 — broken/janky now
- [ ] robot spawn y = half+0.05 drops them mid-air → visible bounce/shake
- [ ] chassis jitter at rest (traction/contact noise) — bots visibly shake
- [ ] match starts instantly on connect — needs an explicit ARMED→START flow
- [ ] HUD text is debug numbers, not a match UI (phase/clock/score unclear)
- [ ] viz frame timing: 50Hz updates but no interpolation → stutter
- [ ] camera controls: no zoom limits/reset/focus keys
- [ ] fuel spawn pile looks wrong (grid, not mound)
- [ ] mechanism arm renders as floating bar, not attached to robot model

## P1 — field/match fidelity
- [ ] bump obstacles aren't the real raised platforms (drive-over shape)
- [ ] trench entries real geometry (currently plain boxes)
- [ ] depot/outpost chutes as intake zones + fuel dispensing
- [ ] hub internal collider so balls rest inside, not bounce off
- [ ] tower/climb mechanism — attach + hang physics, not proximity score
- [ ] fouls/rules: crossing, contact penalties, starting line violation
- [ ] human player fuel feed at outpost chute
- [ ] real auto timing (hub shift winner order) + disabled→armed→auto

## P2 — robot/mechanism realism
- [ ] mechanisms as CAD-part rigid bodies w/ joints (not one bar)
- [ ] physical shooter: flywheel contact → launch, not params
- [ ] physical intake: roller contact, capture timing, jams
- [ ] weight shift affects driving (elevator up = tip risk)
- [ ] bumpers separate collision vs chassis body
- [ ] drivetrain: real swerve kinematics (module positions, not traction circle)

## P3 — sim-as-a-tool (what people actually use)
- [ ] lineup editor UI: pick archetype+auto per slot in browser
- [ ] robot package import: drop robot dir → appears in picker
- [ ] replay scrub UI (timeline over FMSL logs)
- [ ] match stats: fuel scored per robot per phase, cycle time
- [ ] scouting input: load strategy JSON per alliance
- [ ] WPILib adapter verified end-to-end on a real robot project
- [ ] deterministic replays from UI (record/download button)
- [ ] headless `sim` + tune already work — wire to a UI

## P4 — presentation
- [ ] proper skybox/environment lighting, field materials
- [ ] alliance-colored robot overlays/indicators
- [ ] fuel pile visual states (held count bubble per robot)
- [ ] score popups on hub goals
- [ ] match timeline bar (phase progress + shift windows)
- [ ] audio: match start/end chimes
- [ ] mobile/touch controls for gamepad

## P5 — engineering
- [ ] piece render: instanced already — next: LOD spheres at distance
- [ ] viz interpolation between 50Hz updates (lerp positions)
- [ ] physics substep budget guard (504 pieces at 400Hz)
- [ ] WASM core (blocked #24) → in-browser no-server mode
- [ ] AdvantageKit log ingestion → auto-tune (#25)
- [ ] real team CADs: 254/4414 gated on Onshape export

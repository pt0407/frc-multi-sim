# frc-multi-sim — Implementation Plan

Ordered work plan for the open GitHub issues plus identified gaps. Phase order
matters: each phase unblocks or de-risks the next. See `../ARCHITECTURE.md` for
the design rationale and `PLAN.md`'s dependency graph for ordering constraints.

**Legend:** `[issue #N]` = tracked on GitHub · `[new]` = needs an issue filed ·
`[blocked]` = external dependency · `[done]` = merged.

---

## Status snapshot (2026-11-05, updated)

**Landed:** A1 determinism/replay + A4 refactor (#34) · traction model (#18) ·
game pieces + scoring (#37) · web viz + real 2026 field GLB (#19, #55) ·
sensor noise/vision (#39) · archetypes + per-game configs (#20, #51) ·
CI + headless goldens (#17) · match state machine (#38) · NT debug pub (#35) ·
trajectory (#41), strategy (#59) + gamepad (#51) agents · 6-robot scale (#45) ·
fault injection (#54) · headless sim bin + proto replay (#49) ·
RL obs/policy/experience scaffold (#50) · rolling-resistance fix +
constants audit (#48) · agent bug batch (#46) · mech articulation spike +
v2 gaps (#58, mech_angle/NT/viz/intake-gating) · robot GLB pipeline (#56) ·
M4 replay-tuning scaffold (`tune` binary, #25 partial).

**Open, plan-relevant:** #6 WPILib adapter `[blocked: JDK+Gradle]` ·
#33 match-state override API `[other agent in-flight]` ·
#24 WASM build `[deprioritized]` · #25 M4 AdvantageKit tuning
`[scaffold done; needs real logs]` · #43 v2 mechanism articulation `[spike
landed; full scope open]`.

**Duplicates still open:** none (#15/#16 closed).

---

## Work split (2026-11-05): two parallel sessions

**Session A — fast implementer** (mechanical, well-scoped, plumbing-heavy):
bug batch #26–29 · #13 gamepad agent · #30 headless sim bin · #31 AgentMsg
replay · #33 match-state API · #23 per-game configs · #42 field import ·
#43 robot CAD v1/v2 · #15/#16 dup cleanup · PR #41 trajectory agent.

**Session B — physics/math** (modeling judgment, tuning, validation):
#40 chassis friction (correct rolling-resistance model, not just a constant)
· #44 constants alignment (friction/restitution/damping provenance) ·
#12 RL obs vector + actor · #25 M4 AdvantageKit log-replay auto-tuning ·
#6 WPILib adapter when JDK lands · drivetrain/traction refinements.

Rule of thumb: if the PR's correctness depends on *physics judgment*
(friction values, tuned constants, sim-vs-real divergence), it's Session B.
If it's protocol plumbing, CLI surfaces, asset pipelines, or UI, it's Session A.
Overlap files to watch: `world/` (B) vs `agents/`, `viz`, `web/`, `bin/` (A).

---

## 0. Conventions (apply to every issue)

These rules exist so later phases stay verifiable. Every PR should satisfy the
relevant ones:

- **Definition of done — physics changes:** a replay-diff golden test (from #5)
  or a headless pose-bound assertion (from #9). No physics PR merges on
  "looks right in the viz" alone.
- **Definition of done — agent-facing changes:** smoke test via `nt_probe`
  (NT4 path) or a scripted WS client (protocol path). Sensor topics must be
  confirmed publishing before merge.
- **Determinism contract:** all randomness flows through a single seeded RNG
  owned by the orchestrator (§3 below). No `rand::thread_rng()` in world code.
- **Protocol versioning:** `AgentMsg`/`ServerMsg` carry a `protocol_version`
  field from day one. Breaking changes bump it; agents with mismatched versions
  are rejected at `Hello` with a clear error rather than silently misbehaving.
- **Performance budget:** orchestrator must hold **≥400 Hz physics** with 6
  chassis + ~200 game pieces on a single core, and agent-tick → command-applied
  latency **<5 ms** p99. `fast` mode targets **10–100×** real time for RL
  rollouts. Any PR that regresses these gets flagged in review.

---

## 1. Housekeeping (do first)

- Close duplicate issues **#15** and **#16** (duplicates of #13 and #14).
- **#6** (real WPILib sim adapter) `[blocked]` — needs JDK + Gradle on the
  target machine. Independent of everything else; revisit when the toolchain
  exists. The NT4 bridge already proves the seam with mock robots.
- File the missing issues identified in §6–§9 below.

---

## Phase A — Foundation

Everything after this phase should be verifiable via replay-diff rather than
manual testing.

### A1. [#5] Determinism: fixed-step lockstep + replay/diff log
**Why first:** `src/bin/server.rs` uses wall-clock `interval()` for the physics
loop (~line 266) — non-deterministic. Until the loop is step-count-driven and
inputs are event-sourced, nothing downstream can be regression-tested.

Scope:
- Replace wall-clock pacing with a step counter; add the three time modes from
  arch doc §2.1: `realtime` (wall-clock locked), `lockstep` (agents tick on
  physics schedule — required for WPILib's 20 ms loop), `fast` (no sleeps,
  async agents, for RL).
- **Event-sourced input log:** record `(tick, robot_id, ActuatorFrame)` tuples;
  write to a compact binary replay file (length-prefixed frames, header with
  seed + protocol version + archetype config hash).
- **Replay binary:** re-runs a log deterministically; `--diff` mode compares
  per-tick chassis poses/velocities against a golden run and reports first
  divergence tick + magnitude.
- **Seeded RNG contract (§3):** introduce `SimRng` (e.g. `rand_pcg`/`ChaCha`
  with explicit seed) owned by `World`; all future stochastic features
  (piece spawns, sensor noise, RL domain randomization) draw from it.
- Contract test: same seed + same log → bit-identical trajectory.

### A2. [#9] CI: build + headless collision regression test
- GitHub Actions workflow: `cargo build --release --locked`, `cargo test`,
  `cargo clippy` (warnings currently exist — `dead_code` on `NtServer.state`,
  `unused_mut` in server.rs:314 — clean these up in this PR).
- Headless regression test: run the 2-robot collision scenario in `fast`
  mode, assert the weaker robot is displaced toward its wall (pose bounds).
- After A1 lands: upgrade to replay-diff assertion against a checked-in golden
  log.

### A3. [#8] Robot archetype config files
`server.rs` hardcodes `NUM_ROBOTS`, `ROBOT_HALF`, `ROBOT_MASS`, spawn positions
(~lines 15–18, 88–112). Replace with a config layer:

- TOML (or JSON) archetype spec per robot: chassis dims + mass + CG height,
  spawn pose, wheel/module layout (feeds A4's traction model), NT port or
  agent type (ws/nt4/scripted/human/rl), mechanism params (intake region,
  shooter params, climber attach point).
- Match-level config: game identifier, field asset reference, piece counts,
  auto/teleop durations — this is the seam where per-game configs (§7) plug in.
- Server loads N archetypes at startup; `NUM_ROBOTS` becomes `config.robots.len()`.
- Validation: schema-checked at load; unknown fields error loudly.

### A4. [new] Server module refactor — do alongside A3
`src/bin/server.rs` is a 350-line monolith: physics world, NT4 bridge, WS
protocol, and accept loop all inline. Every later issue (#11, #14, #3) touches
this file. Refactor into:

```
src/
  world/        # Rapier world, chassis, field, stepping, sensor extraction
    mod.rs, field.rs, chassis.rs
  agents/       # protocol server, nt4 bridge, per-agent command slots
    mod.rs, ws.rs, nt_bridge.rs
  protocol.rs   # AgentMsg/ServerMsg + versioning (extends lib.rs)
  config.rs     # archetype + match config loading (from A3)
  replay.rs     # event log write/read/diff (from A1)
  rng.rs        # SimRng wrapper (from A1)
```

Binaries (`server`, `drive`, `robot_process`, `nt_probe`) stay thin. Do this
**before** Phase B so #2/#3/#11 don't each re-litigate file structure.

---

## Phase B — Single-agent realism (M1)

### B1. [#2] Per-wheel drivetrain traction model
**Realism-critical** (arch doc risk #2). Port maple-sim's friction-circle
model: per-module normal load → grip limit → longitudinal/lateral slip → force,
applied as custom Rapier forces at wheel contact points. NOT Rapier's built-in
vehicle controller.

- **Changes the `ActuatorFrame` contract:** wheel/module voltage or force
  commands per module (with module count/orientation from A3 archetype)
  instead of raw chassis `force`/`torque`. *This is the contract #3, #10,
  #12, #13 all consume — get it right once.*
- Swerve first (maple-sim's model); tank/mecanum variants as config, not code.
- Validation target: reproduce maple-sim or AdvantageKit drive-log
  acceleration/traction curves within tolerance on a straight-line and
  rotation test.

### B2. [#7] Sensor noise + latency model
Orchestrator-side filtering per arch doc §2.2 — applied by the orchestrator so
all agents see identically-filtered truth:

- Seeded noise (§3 RNG) on gyro, encoders, IMU accel; configurable bias
  (per-boot random offset) and Gaussian noise per sensor.
- Vision pose estimate: latency queue + error model (PhotonVision-style:
  distance-dependent noise, occasional outliers/dropouts).
- `contact` and game-piece contacts get realistic update rates, not per-tick.
- Config per archetype: "clean sim" vs "competition realistic" presets.

### B3. [#3] Game piece physics + scoring layer (FuelSim port)
- Port FuelSim's tuned constants: `FIELD_COR`, `FUEL_COR`, `NET_COR`,
  `ROBOT_COR`, plus the drag model.
- Game pieces as Rapier rigid bodies (spheres w/ restitution + drag); counts
  and spawn zones from match config (A3).
- **Mechanism proxies** (arch doc §2.1) — scope explicitly:
  - *Intakes:* active-region collision sensors (FuelSim `registerIntake`
    pattern) → piece attaches/follows robot while in region.
  - *Shooters:* parameterized ballistics spawners (flywheel velocity, hood
    angle → launch vector + spin).
  - *Climbers:* rigid-body joint/constraint attachment to field climb points.
- Scoring: hub/goal detection zones → points per arch doc game rules; feeds
  match state machine (§9).
- Performance check: ~200 pieces must hold 400 Hz (see §0 budget).

### B4. [#4] Browser visualization client
- Babylon.js scene consuming a broadcast state stream. Extend the WS protocol
  with a `StateFrame` (all robot poses + piece poses at 50 Hz, or delta
  encoding if bandwidth demands).
- Match control UI (auto/teleop/endgame, start/stop), robot inspector
  (per-robot sensor views), camera follow.
- After A1: replay scrubber — load a replay log, scrub timeline, re-sim or
  interpolate recorded states.
- Lowest coupling in Phase B — can proceed in parallel with B3.

### B5. [new] AdvantageScope / NT debug publishing
- Orchestrator republishes per-robot internals (pose, vel, commands, contacts)
  on NT4 topics in AdvantageScope-compatible layout.
- Cheap observability win; makes every later debugging session faster.
  Implement alongside B4 since both consume the state stream.

---

## Phase C — Multi-agent (M2)

### C1. [#14] Field geometry from config
- Replace hardcoded ground/walls (server.rs ~71–85) with field colliders
  loaded from match config (A3): field dims, wall heights/restitution,
  scoring zones, climb points, intake/shooter region definitions.
- Asset pipeline decision: collider primitives from config now; GLB import
  (à la fieldcore/Synthesis) later if needed — keep the collider-generation
  seam clean either way.

### C2. [#11] Scale to 6 robots
Mostly mechanical once A3/A4 exist, but audit these known hardcodings:
- `NUM_ROBOTS` const, `[Option<ActuatorFrame>; N]` command array (~lines 15,
  168, 242).
- **Contact filter bug:** `sensor_frame` uses `1 - robot` (~line 150) —
  assumes exactly 2 robots. Must become per-pair contact aggregation.
- Spawn layout: config-driven starting zones per game (alliance rows etc.).
- NT port allocation: per-archetype config, no collisions.
- Performance re-check at 6 robots + pieces (budget §0).

### C3. [#10] Match state machine + scoring
- States: `disabled → auto → teleop → endgame → complete`, with per-game
  durations from match config.
- `match_state` field (arch doc §2.2 contract) wired into every SensorFrame.
- Scoring accumulation, endgame/climb bonus logic per game rules.
- Autonomous period: scripted/planned routines per robot or full WPILib auto.

### C4. [new] Per-game match configs
Distinct from A3's *archetype* configs — this is the *game* layer:

- `games/rebuilt.toml`, `games/<next-game>.toml`: field asset ref, piece
  types/counts/spawn tables, scoring table, match durations, endgame rules.
- Archetype configs reference compatible games (a 2026 bot can't play a
  2027 game).
- This is what makes "one real-code robot playing REBUILT vs scripted
  opponents" a config exercise rather than a code change.

### C5. [#13] Human gamepad agent
- Gamepad → `ActuatorFrame` (post-B1: per-wheel commands). Deadzone/rate
  limiting in the agent process, not the orchestrator.
- Runs as a first-class agent alongside NT4/WS/scripted/RL agents.
- Close duplicate **#15** when done.

---

## Phase C+ — Realism & identity (new, from fieldcore research)

Researched 2026-11-05 (full writeup on #43). Core insight: an FRC robot in
match sim is ONE rigid body + mechanism proxies — CAD integration needs only
visual mesh + collision proxy + mass properties, not articulated URDF.

### C+1. [#42] Import real 2026 REBUILT field
- fieldcore ships `field.glb` (17 MB, official AdvantageScope 2026 package)
  + `gamepiece.glb` (Fuel). Babylon imports GLB natively.
- Colliders: Rapier `trimesh` is correct for STATIC field geometry (hollow
  is fine — field never moves); fieldcore's recipe = mesh colliders on GLB
  submeshes filtered by name/vertex-count.
- Correct dims: field is **16.541 × 8.069 m** (ours: 16.5 × 8.0 — fix half_z).
- `fields/rebuilt.toml` gains a `mesh` source alongside primitives.

### C+2. [#44] Align physics constants with reference values
- fieldcore constants extracted: fuel 0.215 kg / 0.15 m / friction 0.6 /
  restitution 0.18 / damping 0.16+0.22; chassis friction **0.04** (confirms
  #40); field friction 0.82 / restitution 0.08.
- Audit `world/pieces.rs` + `world/chassis.rs`; record provenance per constant.

### C+3. [#43] Robot CAD pipeline — "team code + CAD → launchable robot"
- **v1:** `robots/<name>/robot.toml` (archetype) + `mesh.glb` (visual only) +
  code endpoint. Babylon renders the GLB; physics keeps primitives. Ships
  immediately — teams see their actual robot.
- **v2:** auto collision fit — gltf → `MeshConverter::ConvexDecomposition`
  (VHACD, target <~20 convex pieces for the 400 Hz budget); import mass
  properties (mass/CoM/**inertia tensor** — governs contact rotation).
- **v3 (maybe never):** articulated mechanisms via `onshape-to-robot` URDF —
  only if mechanism proxies prove insufficient.
- Convention: floor-center pivot, +X forward, Y-up, meters. Manual GLB export
  (Onshape/Fusion both export glTF) beats API-key onboarding for v1.
- Verify VHACD determinism for replay-diff (#5 contract).

---

## Phase D — RL (M3)

### D1. [#12] RL observation vector + actor scaffold
Depends on: A1 (`fast` mode), B2 (noise in obs), C3 (`match_state` in obs).

- Normalized obs vector à la StrategyEngine's `GlobalState`: own pose/vel,
  alliance-relative piece positions, opponent/alliance proximity, contact
  states, score, match clock, last actions.
- ONNX inference: sidecar process first (keeps orchestrator lean), in-proc
  later if latency demands.
- Experience logging → PPO learner (StrategyEngine actor/learner split as
  reference); domain randomization draws from §3 RNG.
- Fast-mode requirement: batch N parallel worlds in-process for rollouts —
  needs the world refactor (A4) to be instantiable, not a singleton.

### D2. [new] WASM physics core build
Arch doc §2.5: compile the Rapier world to WASM for offline single-robot dev
— same determinism, no server. Requires A4's clean `world/` module boundary
(world code must not depend on tokio/native-only deps). Deprioritize until
M1–M2 are stable; it's a dev-experience feature, not a blocker.

---

## Phase E — Validation (M4) — file the issue now, schedule last

### E1. [new] AdvantageKit log replay → physics auto-tuning
The arch doc's "most realistic" claim rests entirely on this. No issue exists
yet — file it.

- Import real-match drive logs (AdvantageKit): commanded vs actual velocity,
  accel, current draw.
- Replay the same command sequence through the sim; compute divergence
  metrics (velocity error, stopping distance, traction limits).
- Auto-tune restitution, friction, drag, and B1's traction parameters to
  minimize divergence — gradient-free search (CMA-ES or Bayesian opt) over
  the parameter set, replay-diff as the objective function (A1 makes this
  cheap).
- Output: tuned defaults committed as archetype/game config presets + a
  validation report per season.

---

## Dependency graph

```
A1(#5) ──► A2(#9) ──► all later verification
A1(#5) ──► B3(#3), C3(#10), D1(#12)   [fast mode / replay]
A3(#8) ──► A4(refactor) ──► C1(#14), C2(#11), D1(#12 batch worlds)
A3(#8) ──► C4(per-game configs)
B1(#2) ──► B3(#3), C3(#10), C5(#13), D1(#12)   [actuator contract]
B2(#7) ──► D1(#12)                   [noise in obs]
B3(#3) ──► C3(#10)                   [scoring feeds match state]
C3(#10) ──► D1(#12)                  [match_state in obs]
C1(#14) ──► C2(#11)                  [field config before 6-robot spawn]
#6  — blocked on JDK+Gradle (independent)
D2(WASM) — after A4, deprioritized
E1(M4)  — last; consumes everything
```

## Critical path

`#40 → #11 → #42 → #43-v1 → C3(#10 follow-ups) → D1 → E1`

- **#40 (chassis friction) is the new keystone-adjacent bug** — every agent
  silently stalls under ~530 N; fix before scaling agents or tuning RL.
- **#11 (6 robots) is next structural work** — Phase C core.
- **#42/#43 are the "looks like a real FRC sim" payoff** — fieldcore proved
  the assets exist; viz + physics integration is mostly plumbing.
- **E1 is the credibility payoff** — schedule last but keep the issue warm.

## Suggested next PRs (post-#34 world)

1. **#40 friction fix** — one-line-ish collider change + golden re-record;
   unblocks honest agent development.
2. **#11 six-robot scale** — audit `NUM_ROBOTS`, command slots, the
   `1 - robot` contact filter (now per-pair in world/), spawn zones.
3. **#42 field import spike** — GLB → Babylon + trimesh colliders; biggest
   visual jump per effort.

//! The authoritative Rapier world: arena, chassis, stepping, and sensor
//! extraction (PLAN A4). No networking, no wall-clock — the world is
//! advanced one fixed step at a time by the caller, which is what makes the
//! simulation deterministic and replayable (issue #5).
pub mod chassis;
pub mod field;
pub mod joint;
pub mod mech;
pub mod noise;
pub mod pieces;
pub mod vision;

pub use crate::archetype::IntakeSpec;
pub use chassis::{RobotSpec, NUM_ROBOTS, ROBOT_HALF, ROBOT_MASS};
pub use field::{FieldSpec, FIELD_HALF_X, FIELD_HALF_Z};
pub use pieces::{HubZone, PieceTable, SpawnZone};

use crate::drivetrain::{rot2, yaw_from_quat, Drivetrain};
use self::vision::TagView;
use crate::r#match::{MatchClock, MatchPhase, PhaseScore};
use crate::rng::SimRng;
use crate::{ActuatorFrame, SensorFrame};
use noise::SensorNoise;
use rapier3d::prelude::*;

pub const PHYSICS_HZ: f32 = 400.0;
/// Default carried-piece feed time (carry anchor -> muzzle) when the
/// shooter joint doesn't set `feed_time`. ~0.15 s matches a real
/// intake-to-flywheel transit.
pub const FEED_TIME_DEFAULT: f32 = 0.15;
/// Seconds an out-of-bounds piece sits flagged before teleporting back
/// to its return point (MoSim ReturnCounters = 3 s).
pub const RETURN_DELAY_S: f32 = 3.0;
pub const AGENT_EVERY: u32 = 8; // 50 Hz agent tick

/// Feed state for a carried piece traveling carry-anchor -> muzzle
/// (MoSim GamePieceGraph SendTo/Release: the piece physically traverses
/// the mechanism path instead of teleporting on shot).
#[derive(Debug, Clone, Copy)]
struct FeedState {
    /// elapsed feed time, s
    t: f32,
    /// total feed duration, s
    dur: f32,
    /// shot params captured at feed start; launch vel/pos are resolved
    /// at release against live joint + chassis state
    shot: crate::ShotCommand,
}

/// A game piece kinematically carried by a robot (intake capture ->
/// mechanism ride -> feed -> ballistic release).
#[derive(Debug, Clone, Copy)]
struct CarriedPiece {
    handle: RigidBodyHandle,
    robot: usize,
    /// `Some` while the piece is traveling to the muzzle on a shot
    feed: Option<FeedState>,
}

/// A scored piece held inside a hub (MoSim occupancy model: scored fuel
/// stays in the scoring mechanism, visible in the world, and cannot
/// score again — the 1.5 s per-piece debounce is moot while held).
#[derive(Debug, Clone, Copy)]
struct HubHeldPiece {
    handle: RigidBodyHandle,
    /// index into `World::hubs`
    hub: usize,
}

/// Out-of-bounds piece awaiting return to play (MoSim
/// RebuiltGamePieceController: flag on exit, 3 s counter, then
/// MovePosition back to the alliance return point).
#[derive(Debug, Clone, Copy)]
struct ReturnState {
    handle: RigidBodyHandle,
    /// seconds remaining before teleport (MoSim ReturnCounters = 3 s)
    t: f32,
    /// world return point
    point: [f32; 3],
}

pub struct World {
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
    pub chassis: Vec<RigidBodyHandle>,
    pub chassis_colliders: Vec<ColliderHandle>,
    drivetrains: Vec<Drivetrain>,
    /// per-robot chassis half-extents (for wheel-contact spring points)
    chassis_dims: Vec<[f32; 3]>,
    /// Seeded RNG — the only legal source of randomness in world code
    /// (PLAN §0). Piece spawns / sensor noise draw here.
    pub rng: SimRng,
    /// The field layout this world was built from (issue #14); viz reads
    /// it to emit the config message, and it feeds the replay config hash.
    pub field: FieldSpec,
    /// live game-piece bodies (issue #3); scored pieces are removed,
    /// captured pieces switch to kinematic carry (see `carried`)
    pub pieces: Vec<RigidBodyHandle>,
    /// pieces physically captured by intake: kinematic bodies glued to a
    /// carry anchor on their robot (MoSim GamePieceIntake model — the piece
    /// visibly rides the mechanism instead of vanishing into a counter).
    /// Released ballistically on `shoot`; preload inventory (held without
    /// a body) still spawns fresh on shot.
    carried: Vec<CarriedPiece>,
    /// scored pieces held kinematically inside their hub
    hub_held: Vec<HubHeldPiece>,
    /// out-of-bounds pieces on their 3 s return timer (MoSim return
    /// points — fuel that leaves the field comes back into play)
    returning: Vec<ReturnState>,
    /// last robot to capture each piece (MoSim HasOwner/OwnedBy — piece
    /// possession persists through carry and release)
    piece_owner: Vec<(RigidBodyHandle, usize)>,
    /// zone-shot penalty (MoSim ZoneShotsPenalty): instances per
    /// alliance, consecutive-event counter, last event tick
    zone_penalty_instances: [u32; 2],
    zone_penalty_consec: u32,
    zone_penalty_last_tick: Option<u64>,
    /// per-robot held-piece inventory, fed by intake captures and consumed
    /// by shots
    pub held: Vec<u32>,
    /// per-alliance score (alliance index = robot_id % 2)
    pub scores: [u32; 2],
    hubs: Vec<HubZone>,
    intakes: Vec<Option<IntakeSpec>>,
    /// sticky intake-run flag per robot, updated from actuator frames
    intake_active: Vec<bool>,
    /// edge-trigger for `cmd.shoot` — a latched frame consumes one piece
    /// once, not every physics tick until the next command arrives
    shot_latched: Vec<bool>,
    /// physics ticks elapsed — the world's only clock (issue #10); the
    /// match phase is a pure function of this via `match_clock`
    tick: u64,
    /// match state machine config (issue #10); default = start enabled
    /// (disabled_s = 0) so bare worlds behave exactly like before
    pub match_clock: MatchClock,
    /// physics tick when the match was started (start_match); the match
    /// clock is a function of `tick - match_epoch` so staging time doesn't
    /// count. Set via `start_match()`.
    pub match_epoch: u64,
    /// per-alliance score attributed to the phase it was scored in
    /// (auto / teleop / endgame+bonus); `scores` stays the running total
    pub score_by_phase: [PhaseScore; 2],
    /// set once the climb bonus has been awarded at match completion
    match_finished: bool,
    /// manual phase override (issue #33): (phase, ticks remaining). While
    /// set, `phase()`/`time_remaining()` report the override and the normal
    /// clock resumes when it expires.
    phase_override: Option<(MatchPhase, u64)>,
    /// per-robot sensor noise/latency models (issue #7); drawn from `rng`
    sensors: Vec<SensorNoise>,
    /// per-robot vision cameras (`[camera]` archetype spec); `None` = no
    /// camera, so `vision_pose` stays on the legacy ungated path
    cameras: Vec<Option<crate::archetype::CameraSpec>>,
    /// per-robot articulated mechanism (issue #57 spike): `Some` only for
    /// specs that opt in via `[mech]`; `None` robots stay single-body
    /// articulated mechanism parts per robot (issue #61)
    pub mechs: Vec<Vec<mech::Mech>>,
    /// per-robot kinematic mechanism joints (`[[joint]]` specs): spin,
    /// position, swerve steer and wheel channels simulated server-side
    /// and streamed to the viz for CAD binding
    pub joints: Vec<Vec<joint::JointSim>>,
}

impl World {
    /// Default six-robot world (alliance rows at x=∓4, z=-2,0,+2 —
    /// alliance = robot_id % 2), unseeded (seed 0).
    pub fn new() -> Self {
        Self::with_specs(&chassis::default_specs(), 0)
    }

    /// Build the arena and spawn one chassis per spec. `seed` initializes
    /// the world's `SimRng`; the same seed + same inputs always produce the
    /// same trajectory.
    pub fn with_specs(specs: &[RobotSpec], seed: u64) -> Self {
        Self::with_config(specs, seed, FieldSpec::default(), &PieceTable::default())
    }

    /// Full constructor: robot specs + field layout + game-piece table.
    pub fn with_config(
        specs: &[RobotSpec],
        seed: u64,
        field: FieldSpec,
        table: &PieceTable,
    ) -> Self {
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
            // fallback profile. Config-file selection is issue #8 follow-up.
            drivetrains: specs
                .iter()
                .map(|s| Drivetrain::swerve(s.mass, s.half_extents[0]))
                .collect(),
            chassis_dims: specs.iter().map(|s| s.half_extents).collect(),
            rng: SimRng::seeded(seed),
            field,
            pieces: Vec::new(),
            carried: Vec::new(),
            hub_held: Vec::new(),
            returning: Vec::new(),
            piece_owner: Vec::new(),
            zone_penalty_instances: [0; 2],
            zone_penalty_consec: 0,
            zone_penalty_last_tick: None,
            held: vec![0; specs.len()],
            scores: [0; 2],
            hubs: table.hubs.clone(),
            intakes: specs.iter().map(|s| s.intake).collect(),
            intake_active: vec![false; specs.len()],
            shot_latched: vec![false; specs.len()],
            tick: 0,
            match_clock: MatchClock::default(),
            match_epoch: 0,
            score_by_phase: [PhaseScore::default(); 2],
            match_finished: false,
            phase_override: None,
            sensors: Vec::new(),
            cameras: Vec::new(),
            mechs: Vec::new(),
            joints: Vec::new(),
        };

        // issue #42: a GLB field mesh CAN replace the placeholder
        // `obstacles` with trimesh colliders, but it's opt-in
        // (`glb_colliders`) — 1.8 M tris is ~50 Hz in release. Walls +
        // ground always stay (fieldcore does the same). GLB load failure
        // warns and falls back to primitives.
        let mut arena_spec = w.field.clone();
        let want_glb = w
            .field
            .glb_colliders
            .unwrap_or(false)
            .then(|| w.field.glb.clone())
            .flatten();
        let glb_meshes = want_glb.as_ref().and_then(|p| {
            match crate::glb::load_field_meshes(std::path::Path::new(p)) {
                Ok(m) => {
                    let tris: usize = m.iter().map(|x| x.indices.len()).sum();
                    println!(
                        "field GLB {}: {} meshes, {} triangles",
                        p,
                        m.len(),
                        tris
                    );
                    Some(m)
                }
                Err(e) => {
                    eprintln!("warning: field GLB {p} failed to load ({e}); using primitive obstacles");
                    None
                }
            }
        });
        if glb_meshes.is_some() {
            arena_spec.obstacles.clear();
        }
        let arena_body = field::build_arena(&arena_spec, &mut w.bodies, &mut w.colliders);
        if let Some(meshes) = glb_meshes {
            // merge into ONE trimesh: 1000+ separate BVHs made startup take
            // ~60 s; a single BVH over the same triangles builds far faster
            // and broadphase has less to do.
            let total_v: usize = meshes.iter().map(|m| m.positions.len()).sum();
            let total_i: usize = meshes.iter().map(|m| m.indices.len()).sum();
            let mut positions = Vec::with_capacity(total_v);
            let mut indices = Vec::with_capacity(total_i);
            for m in &meshes {
                let base = positions.len() as u32;
                positions.extend(m.positions.iter().map(|p| point![p[0], p[1], p[2]]));
                indices.extend(
                    m.indices
                        .iter()
                        .map(|t| [t[0] + base, t[1] + base, t[2] + base]),
                );
            }
            w.colliders.insert_with_parent(
                ColliderBuilder::trimesh(positions, indices)
                    .friction(0.82)
                    .restitution(0.08)
                    .restitution_combine_rule(CoefficientCombineRule::Min)
                    .build(),
                arena_body,
                &mut w.bodies,
            );
        }
        w.pieces = pieces::spawn_table(
            &mut w.bodies,
            &mut w.colliders,
            table,
            &mut w.rng,
        );

        for (i, spec) in specs.iter().enumerate() {
            let (body, col) =
                chassis::spawn_chassis(&mut w.bodies, &mut w.colliders, spec);
            w.chassis.push(body);
            w.chassis_colliders.push(col);
            let mut mlist = Vec::new();
            for mspec in &spec.mechs {
                mlist.push(mech::spawn_arm(
                    &mut w.bodies,
                    &mut w.colliders,
                    &mut w.impulse_joints,
                    body,
                    mspec,
                ));
            }
            w.mechs.push(mlist);
            w.joints.push(
                spec.joints
                    .iter()
                    .map(|js| {
                        joint::JointSim::new(js.clone(), w.drivetrains[i].module_count())
                    })
                    .collect(),
            );
            println!(
                "robot {} spawned at x={} mass={} half={:?} mech={}",
                i,
                spec.spawn[0],
                spec.mass,
                spec.half_extents,
                spec.mechs.len()
            );
        }
        // per-boot sensor biases draw from the world RNG in robot order
        // (disabled configs draw nothing → identical stream to before)
        w.sensors = specs
            .iter()
            .map(|s| SensorNoise::new(s.sensors, &mut w.rng))
            .collect();
        w.cameras = specs.iter().map(|s| s.camera).collect();
        w
    }

    pub fn num_robots(&self) -> usize {
        self.chassis.len()
    }

    /// Current match phase. A manual override (issue #33) wins over the
    /// clock until it expires.
    pub fn phase(&self) -> MatchPhase {
        self.phase_at_tick(self.tick)
    }

    /// Phase at an arbitrary tick (override-aware; used by `sensor_frame`,
    /// which receives the tick it is reporting for).
    pub fn phase_at_tick(&self, tick: u64) -> MatchPhase {
        self.phase_override
            .map(|(p, _)| p)
            .unwrap_or_else(|| self.match_clock.phase_at(tick.saturating_sub(self.match_epoch)))
    }

    /// Seconds remaining in the current phase (override-aware).
    pub fn time_remaining(&self) -> f32 {
        self.time_remaining_at_tick(self.tick)
    }

    /// Phase-clock remaining at an arbitrary tick (override-aware).
    pub fn time_remaining_at_tick(&self, tick: u64) -> f32 {
        match self.phase_override {
            Some((_, ticks)) => ticks as f32 / PHYSICS_HZ,
            None => self.match_clock.time_remaining(tick.saturating_sub(self.match_epoch)),
        }
    }

    /// Force `phase` for `secs` seconds, then resume the normal clock.
    /// `secs <= 0` holds the phase until `clear_phase` (issue #33).
    pub fn set_phase(&mut self, phase: MatchPhase, secs: f32) {
        let ticks = if secs > 0.0 {
            (secs * PHYSICS_HZ) as u64
        } else {
            u64::MAX
        };
        self.phase_override = Some((phase, ticks));
    }

    /// Whether a manual phase override is currently active (issue #33).
    pub fn phase_override_active(&self) -> bool {
        self.phase_override.is_some()
    }

    /// Drop any phase override; the clock resumes from `self.tick`.
    pub fn clear_phase(&mut self) {
        self.phase_override = None;
    }

    /// Award `points` to `alliance` (robot_id % 2), attributed to the
    /// current phase bucket — manual-score hook for rehearsals (issue #33).
    pub fn add_score(&mut self, alliance: usize, points: u32) {
        let a = alliance.min(self.scores.len() - 1);
        self.scores[a] += points;
        self.score_by_phase[a].add(self.phase(), points);
    }

    /// Ground-truth translation of one chassis (unfiltered — bypasses the
    /// sensor noise model).
    pub fn position(&self, robot: usize) -> [f32; 3] {
        let t = self.bodies[self.chassis[robot]].translation();
        [t.x, t.y, t.z]
    }

    /// Apply per-robot actuator commands, then advance the physics one
    /// fixed step (1/PHYSICS_HZ). Step-count driven, never wall-clock.
    /// While the match is `disabled` or `complete` actuator commands are
    /// ignored — as on a real field (issue #10).
    pub fn step(&mut self, commands: &[Option<ActuatorFrame>]) {
        let phase = self.phase();
        let enabled = phase.enabled();
        for (i, handle) in self.chassis.iter().enumerate() {
            let cmd = if enabled {
                commands.get(i).and_then(|c| c.as_ref())
            } else {
                None
            };
            // per-wheel traction model runs every tick — even with no
            // command — so the wheels' lateral friction resists pushes and
            // scrubs drift (issue #40; an uncommanded real swerve still
            // grips the carpet sideways). A zeroed frame keeps steer
            // headings and applies only the lateral-slip force.
            let zero = ActuatorFrame {
                force: [0.0, 0.0],
                torque: 0.0,
                modules: None,
                intake_active: false,
                shoot: None,
                mech: None,
                joints: None,
                shooter_on: false,
            };
            let cmd = cmd.unwrap_or(&zero);
            {
                // per-wheel traction model: resolve the actuator command into
                // module forces (grip-limited), then apply them off-center so
                // yaw torque emerges from the force placement.
                let (yaw, pos, lv, av) = {
                    let body = &self.bodies[*handle];
                    let r = body.rotation();
                    (
                        yaw_from_quat([r.i, r.j, r.k, r.w]),
                        *body.translation(),
                        *body.linvel(),
                        *body.angvel(),
                    )
                };
                let kin = crate::drivetrain::ChassisState {
                    yaw,
                    linvel: [lv.x, lv.z],
                    yaw_rate: av.y,
                    dt: self.integration.dt,
                };
                self.intake_active[i] = cmd.intake_active;
                // kinematic mechanism joints: advance spin/position/steer/
                // wheel channels with the motor model. Wheel channels need
                // chassis speed in the robot frame for the rolling omega.
                let lv_robot = {
                    let (s, c) = yaw.sin_cos();
                    [lv.x * c - lv.z * s, lv.x * s + lv.z * c]
                };
                let n_joints = self.joints.get(i).map(|j| j.len()).unwrap_or(0);
                if n_joints > 0 {
                    // split borrows: joints mutate, drivetrain reads
                    let (js, dt_) = {
                        let js = self.joints.get_mut(i).unwrap();
                        let dt_ = &self.drivetrains[i];
                        (js, dt_)
                    };
                    for (jidx, jsim) in js.iter_mut().enumerate() {
                        jsim.step(
                            jidx,
                            cmd,
                            self.integration.dt,
                            self.intake_active[i],
                            cmd.shoot.is_some(),
                            cmd.shooter_on,
                            dt_,
                            lv_robot,
                        );
                    }
                }
                // shooter gating: with `driver = "shooter"` joints the flywheel
                // must be at speed before a shot releases (FlywheelBehaviour
                // IsAtSpeed). A `shoot` request also spins the wheels up, so
                // agents can just request shots; the shot fires once fast.
                let shooter_ready = self
                    .joints
                    .get(i)
                    .map(|js| js.iter().all(|j| j.shooter_at_speed()))
                    .unwrap_or(true);
                // mechanism servo (issue #57 spike): retarget the arm's
                // revolute motor when the agent commands it; target is
                // clamped to the spec's joint limits
                if let Some(mcmd) = cmd.mech {
                    if let Some(m) = self
                        .mechs
                        .get_mut(i)
                        .and_then(|ms| ms.get_mut(mcmd.joint))
                    {
                    let target =
                        mcmd.target.clamp(m.spec.limits[0], m.spec.limits[1]);
                    if let Some(j) = self.impulse_joints.get_mut(m.joint) {
                        j.data.set_motor_position(
                            m.axis,
                            target,
                            m.spec.motor_stiffness,
                            m.spec.motor_damping,
                        );
                    }
                    }
                }
                // shooter: consume a held piece and spawn a ballistic fuel
                // (FuelSim launchFuel: hood angle, turret yaw, launch height,
                // plus chassis velocity)
                if cmd.shoot.is_none() {
                    self.shot_latched[i] = false;
                }
                if let Some(shot) = cmd.shoot {
                    if self.held[i] > 0 && !self.shot_latched[i] && shooter_ready {
                        self.shot_latched[i] = true;
                        self.held[i] -= 1;
                        // physically-carried piece: feed it carry-anchor ->
                        // muzzle over `feed_time` (MoSim SendTo path — the
                        // piece visibly travels the mechanism); launch
                        // vel/pos resolve at release against live flywheel,
                        // turret and chassis state. Preload inventory
                        // (held with no body) still launches instantly.
                        if let Some(ci) = self
                            .carried
                            .iter()
                            .position(|c| c.robot == i && c.feed.is_none())
                        {
                            let dur = self
                                .joints
                                .get(i)
                                .into_iter()
                                .flatten()
                                .find(|j| {
                                    j.spec.driver
                                        == crate::archetype::JointDriver::Shooter
                                })
                                .and_then(|j| j.spec.feed_time)
                                .unwrap_or(FEED_TIME_DEFAULT);
                            self.carried[ci].feed =
                                Some(FeedState { t: 0.0, dur, shot });
                        } else {
                            let launch_pos = Self::shot_launch_pos(
                                &self.bodies,
                                self.chassis[i],
                                self.joints.get(i),
                                &shot,
                            );
                            let vel = Self::shot_launch_vel(
                                &self.bodies,
                                self.chassis[i],
                                self.joints.get(i),
                                &shot,
                            );
                            self.pieces.push(pieces::spawn_piece(
                                &mut self.bodies,
                                &mut self.colliders,
                                launch_pos,
                                vel,
                            ));
                        }
                        // flywheel ball-contact dip
                        if let Some(js) = self.joints.get_mut(i) {
                            for j in js.iter_mut() {
                                j.ball_dip();
                            }
                        }
                    }
                }
                let resolved = self.drivetrains[i].resolve(cmd, &kin);
                let body = &mut self.bodies[*handle];
                body.reset_forces(true);
                body.reset_torques(true);
                for f in &resolved {
                    // apply slightly below COM: enough to pitch the nose up
                    // over bumps (force-at-COM gives zero torque and the
                    // chassis can never climb a wedge), not so low that the
                    // leading edge digs into flat ground.
                    let p = rot2(f.local_point, yaw);
                    body.add_force_at_point(
                        vector![f.force[0], 0.0, f.force[1]],
                        point![pos.x + p[0], pos.y - 0.26, pos.z + p[1]],
                        true,
                    );
                    if f.torque != 0.0 {
                        body.add_torque(vector![0.0, f.torque, 0.0], true);
                    }
                }
                // virtual-wheel suspension: 4 corner contacts sample
                // field.ground_height and push up with spring-damper force.
                // The rigid chassis conforms to bump terrain per-corner
                // (real robots climb via wheels) instead of seesawing.
                {
                    let [hx, hy, hz] = self.chassis_dims[i];
                    let rq = body.rotation();
                    // compute contacts first (immutable field borrow), then
                    // apply — avoids borrowing self.field while body is mut
                    let mut contacts = [(0f32,0f32,0f32,0f32); 4];
                    let mut nc = 0;
                    for &(lx, lz) in &[
                        (hx * 0.8, hz * 0.8),
                        (hx * 0.8, -hz * 0.8),
                        (-hx * 0.8, hz * 0.8),
                        (-hx * 0.8, -hz * 0.8),
                    ] {
                        let w = rq * vector![lx, -hy + 0.01, lz];
                        let (wx, wy, wz) = (pos.x + w.x, pos.y + w.y, pos.z + w.z);
                        let gh = self.field.ground_height(wx, wz);
                        let pen = gh + 0.01 - wy;
                        if pen > 0.0 {
                            contacts[nc] = (wx, wy, wz, pen);
                            nc += 1;
                        }
                    }
                    for &(wx, wy, wz, pen) in contacts.iter().take(nc) {
                        let f = (9000.0 * pen - 500.0 * lv.y).max(0.0);
                        body.add_force_at_point(
                            vector![0.0, f, 0.0],
                            point![wx, wy, wz],
                            true,
                        );
                    }
                }
            }
        }
        // carried pieces: kinematic-follow their robot's carry anchor
        // (chassis-frame offset rotated by yaw, riding at hopper height).
        // Feeding pieces lerp anchor -> muzzle over `feed_time` and
        // release ballistically at the end (MoSim SendTo/Release path).
        // Set before pipeline.step so Rapier moves them with the robot.
        let mut feed_releases: Vec<(RigidBodyHandle, usize, crate::ShotCommand)> =
            Vec::new();
        for c in &self.carried {
            // anchor rides the mechanism arm when the robot has one —
            // the piece visibly travels with the intake arm from scoop
            // to stowed carry position (MoSim path-graph equivalent)
            let (cpos, cyaw) = {
                let b = &self.bodies[self.chassis[c.robot]];
                let r = b.rotation();
                let t = b.translation();
                ([t.x, t.y, t.z], yaw_from_quat([r.i, r.j, r.k, r.w]))
            };
            let anchor = if let Some(ap) = self.mech_arm_position(c.robot) {
                [ap[0], ap[1] + 0.12, ap[2]]
            } else {
                let (s, co) = cyaw.sin_cos();
                // anchor: front of the chassis at hopper height
                [
                    cpos[0] + 0.30 * co,
                    cpos[1] + 0.45,
                    cpos[2] - 0.30 * s,
                ]
            };
            // feeding pieces travel anchor -> live muzzle position so a
            // moving robot / slewing turret carries the piece with it
            let target = if let Some(f) = &c.feed {
                let k = (f.t / f.dur.max(1e-4)).min(1.0);
                // smoothstep: the piece accelerates out of the mechanism
                // like a real feed, not a constant-speed slide
                let e = k * k * (3.0 - 2.0 * k);
                let muzzle = Self::shot_launch_pos(
                    &self.bodies,
                    self.chassis[c.robot],
                    self.joints.get(c.robot),
                    &f.shot,
                );
                // authored feed_path waypoints (chassis frame) route the
                // piece through the mechanism — hopper -> shooter throat —
                // instead of a straight anchor->muzzle line
                let wps = self
                    .joints
                    .get(c.robot)
                    .into_iter()
                    .flatten()
                    .find(|j| {
                        j.spec.driver == crate::archetype::JointDriver::Shooter
                    })
                    .and_then(|j| j.spec.feed_path.as_ref())
                    .filter(|p| !p.is_empty());
                match wps {
                    Some(wps) => {
                        let (s, co) = cyaw.sin_cos();
                        let mut path = Vec::with_capacity(wps.len() + 2);
                        path.push(anchor);
                        for w in wps {
                            path.push([
                                cpos[0] + w[0] * co + w[2] * s,
                                cpos[1] + w[1],
                                cpos[2] - w[0] * s + w[2] * co,
                            ]);
                        }
                        path.push(muzzle);
                        let segs = path.len() - 1;
                        let u = e * segs as f32;
                        let i = (u.floor() as usize).min(segs - 1);
                        let lt = (u - i as f32).clamp(0.0, 1.0);
                        let (a, b) = (path[i], path[i + 1]);
                        [
                            a[0] + (b[0] - a[0]) * lt,
                            a[1] + (b[1] - a[1]) * lt,
                            a[2] + (b[2] - a[2]) * lt,
                        ]
                    }
                    None => [
                        anchor[0] + (muzzle[0] - anchor[0]) * e,
                        anchor[1] + (muzzle[1] - anchor[1]) * e,
                        anchor[2] + (muzzle[2] - anchor[2]) * e,
                    ],
                }
            } else {
                anchor
            };
            let body = &mut self.bodies[c.handle];
            body.set_next_kinematic_translation(vector![
                target[0], target[1], target[2]
            ]);
        }
        // advance feed clocks and release finished pieces ballistically
        for c in &mut self.carried {
            let Some(f) = &mut c.feed else { continue };
            f.t += self.integration.dt;
            if f.t >= f.dur {
                feed_releases.push((c.handle, c.robot, f.shot));
            }
        }
        for (handle, robot, shot) in feed_releases {
            if let Some(ci) = self.carried.iter().position(|c| c.handle == handle) {
                self.carried.remove(ci);
            }
            let vel = Self::shot_launch_vel(
                &self.bodies,
                self.chassis[robot],
                self.joints.get(robot),
                &shot,
            );
            let lp = Self::shot_launch_pos(
                &self.bodies,
                self.chassis[robot],
                self.joints.get(robot),
                &shot,
            );
            let body = &mut self.bodies[handle];
            body.set_body_type(RigidBodyType::Dynamic, true);
            let cols: Vec<ColliderHandle> = body.colliders().to_vec();
            for ch in cols {
                if let Some(col) = self.colliders.get_mut(ch) {
                    col.set_sensor(false);
                }
            }
            body.set_translation(vector![lp[0], lp[1], lp[2]], true);
            body.set_linvel(vector![vel[0], vel[1], vel[2]], true);
            body.set_angvel(vector![0.0, 4.0, 0.0], true);
        }
        // quadratic air drag on free pieces (FuelSim DRAG_FORCE_FACTOR).
        // Runs BEFORE roller forces: apply_drag resets accumulated forces,
        // so anything added earlier in the tick would be wiped.
        pieces::apply_drag(&mut self.bodies, &self.pieces);
        // mechanism rollers: spinning joint surfaces push free pieces
        // along their surface velocity (MoSim InteractionRoller — rollers
        // reach beyond the capture AABB and actively gather or eject fuel)
        self.apply_roller_forces();
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
        self.update_game_pieces();
        self.tick += 1;
        // phase override expiry (issue #33)
        if let Some((_, remaining)) = &mut self.phase_override {
            *remaining = remaining.saturating_sub(1);
            if *remaining == 0 {
                self.phase_override = None;
            }
        }
        // endgame/climb placeholder (issue #10): on the transition into
        // `complete`, any robot parked within `climb_radius_m` of a field
        // climb point banks the bonus for its alliance
        if !self.match_finished && self.phase() == MatchPhase::Complete {
            self.award_climb_bonus();
            // zone-shot penalties are assessed at match end (FRC): each
            // instance subtracts one point, saturating at zero
            for a in 0..self.scores.len() {
                self.scores[a] = self
                    .scores[a]
                    .saturating_sub(self.zone_penalty_instances[a]);
            }
            self.match_finished = true;
        }
    }

    /// Physics ticks elapsed since world construction.
    pub fn tick(&self) -> u64 {
        self.tick
    }

    /// One-shot endgame bonus: each robot within horizontal
    /// `climb_radius_m` of any field climb point scores
    /// Is robot `i` inside its own alliance zone — own half of the field
    /// past ~45% out (MoSim InAllianceZone: Blue x>4.25, Red x<-4.25 on
    /// their ~9.4 m field; scaled to our 8.27 m tile).
    fn robot_in_alliance_zone(&self, robot: usize) -> bool {
        let t = self.bodies[self.chassis[robot]].translation();
        let thresh = FIELD_HALF_X * 0.45;
        if robot % 2 == 0 {
            t.x < -thresh
        } else {
            t.x > thresh
        }
    }

    /// Zone-shot penalty instances per alliance (MoSim
    /// ZoneShotsPenalty.Red/BlueInstances) — subtracted from the final
    /// score by whatever consumes match results.
    pub fn zone_penalty_instances(&self) -> [u32; 2] {
        self.zone_penalty_instances
    }

    /// `climb_bonus_points` for its alliance (counted in the endgame
    /// bucket). Real rigid-body climb joints are B3 follow-up; this ties
    /// the placeholder to the field spec's climb-point anchors.
    fn award_climb_bonus(&mut self) {
        let cfg = &self.match_clock.config;
        if cfg.climb_bonus_points == 0 || self.field.climb_points.is_empty() {
            return;
        }
        let r2 = cfg.climb_radius_m * cfg.climb_radius_m;
        for (i, handle) in self.chassis.iter().enumerate() {
            let t = self.bodies[*handle].translation();
            let climbed = self.field.climb_points.iter().any(|c| {
                let dx = t.x - c.pos[0];
                let dz = t.z - c.pos[2];
                dx * dx + dz * dz <= r2
            });
            if climbed {
                let a = (i % self.scores.len()).min(self.scores.len() - 1);
                self.scores[a] += cfg.climb_bonus_points;
                self.score_by_phase[a].endgame += cfg.climb_bonus_points;
            }
        }
    }

    /// Post-step piece layer: intake captures (FuelSim registerIntake —
    /// robot-relative AABB, piece is removed and counted as held) and hub
    /// scoring (piece falls through a hub's entry plane -> alliance score).
    fn update_game_pieces(&mut self) {
        let phase = self.phase();
        // robot poses for intake tests (only robots with an active intake;
        // intake is a mechanism command, so it's inert while disabled)
        let robots: Vec<Option<([f32; 3], f32)>> = (0..self.chassis.len())
            .map(|i| {
                // if the robot has a mechanism arm, intake only works while
                // the arm is deployed (issue #57 v2: intake<->arm integration)
                let deployed = self
                    .mech_angle(i)
                    .map_or(true, |a| a < 0.5);
                if phase.enabled()
                    && self.intake_active[i]
                    && self.intakes[i].is_some()
                    && deployed
                {
                    let body = &self.bodies[self.chassis[i]];
                    let t = body.translation();
                    let r = body.rotation();
                    // #61: if the robot has a mechanism arm, the intake
                    // capture region rides the arm tip rather than the
                    // chassis center — pieces are scooped by the arm.
                    let origin = self
                        .mech_arm_position(i)
                        .unwrap_or([t.x, t.y, t.z]);
                    Some((
                        origin,
                        yaw_from_quat([r.i, r.j, r.k, r.w]),
                    ))
                } else {
                    None
                }
            })
            .collect();

        for h in self.pieces.iter() {
            // carried pieces are kinematically glued to their robot —
            // skip intake re-capture and hub scoring for them; hub-held
            // scored pieces are already counted
            if self.carried.iter().any(|c| c.handle == *h)
                || self.hub_held.iter().any(|c| c.handle == *h)
            {
                continue;
            }
            let body = &self.bodies[*h];
            let t = *body.translation();
            let v = *body.linvel();
            let pos = [t.x, t.y, t.z];

            // out-of-bounds: flag for return to play (MoSim return
            // points — 3 s counter then teleport back inside)
            if (pos[0].abs() > FIELD_HALF_X + 0.3
                || pos[2].abs() > FIELD_HALF_Z + 0.3
                || pos[1] < -0.1)
                && !self.returning.iter().any(|r| r.handle == *h)
            {
                self.returning.push(ReturnState {
                    handle: *h,
                    t: RETURN_DELAY_S,
                    point: Self::piece_return_point(pos),
                });
            }

            // intake capture
            let mut captured = false;
            if std::env::var("DEBUG_INTAKE").is_ok() && !robots.iter().all(|r| r.is_none()) {
                static mut N: u32 = 0;
                unsafe { N += 1; if N % 400 == 0 { eprintln!("intake check: robots={:?} piece={:?}", robots.iter().map(|r| r.is_some()).collect::<Vec<_>>(), pos); } }
            }
            for (i, robot) in robots.iter().enumerate() {
                if let Some((rpos, ryaw)) = robot {
                    // hopper capacity: the mechanism physically can't hold
                    // more than `capacity` pieces (carried + preload)
                    let capacity = self.intakes[i]
                        .and_then(|s| s.capacity)
                        .unwrap_or(crate::archetype::HOPPER_DEFAULT);
                    // feeding pieces already left `held` but are still
                    // aboard until release — count them for capacity
                    let feeding = self
                        .carried
                        .iter()
                        .filter(|c| c.robot == i && c.feed.is_some())
                        .count() as u32;
                    if self.held[i] + feeding < capacity
                        && pieces::intake_contains(
                            &self.intakes[i].unwrap(),
                            rpos,
                            *ryaw,
                            &pos,
                        )
                    {
                        self.held[i] += 1;
                        // carry, don't despawn: kinematic body glued to the
                        // robot's carry anchor until shot or match end.
                        // Sensor collider — a ball riding inside the
                        // mechanism must not shove the chassis or field.
                        {
                            let body = &mut self.bodies[*h];
                            body.set_body_type(
                                RigidBodyType::KinematicPositionBased,
                                true,
                            );
                            body.set_linvel(vector![0.0, 0.0, 0.0], true);
                            body.set_angvel(vector![0.0, 0.0, 0.0], true);
                            let cols: Vec<ColliderHandle> =
                                body.colliders().to_vec();
                            for ch in cols {
                                if let Some(c) = self.colliders.get_mut(ch) {
                                    c.set_sensor(true);
                                }
                            }
                        }
                        self.carried.push(CarriedPiece {
                            handle: *h,
                            robot: i,
                            feed: None,
                        });
                        // an intaked piece can't also be returning
                        self.returning.retain(|r| r.handle != *h);
                        // MoSim HasOwner: possession persists through
                        // carry and release for zone-shot penalties
                        self.piece_owner.retain(|(oh, _)| *oh != *h);
                        self.piece_owner.push((*h, i));
                        captured = true;
                        break;
                    }
                }
            }
            if captured {
                continue;
            }

            // hub scoring: first matching hub wins; the point is also
            // attributed to the match phase it was scored in (issue #10).
            // MoSim occupancy model: the piece is CAPTURED kinematically
            // inside the hub — it stays in the world (fuel visibly piles
            // up in the scorer) and can't score again.
            if let Some((hi, hub)) = self
                .hubs
                .iter()
                .enumerate()
                .find(|(_, hub)| pieces::hub_scores(hub, &pos, &[v.x, v.y, v.z]))
            {
                let a = (hub.alliance as usize).min(self.scores.len() - 1);
                // REBUILT hub shifts: a hub only scores while its alliance's
                // window is active (always active when shift_s == 0)
                if !self.hub_active_at(a, self.tick) {
                    continue;
                }
                if std::env::var("FRS_SCORE_DEBUG").is_ok() {
                    eprintln!("[score] hub@({:.2},{:.2}) a={} pos=({:.2},{:.2},{:.2})", hub.center[0], hub.center[1], a, pos[0], pos[1], pos[2]);
                }
                self.scores[a] += 1;
                self.score_by_phase[a].add(phase, 1);
                // zone-shot penalty (MoSim "404"): a scored piece still
                // owned by a robot that is OUTSIDE its alliance zone
                // counts toward ZoneShotsPenalty — 3rd consecutive event
                // within 5 s = 3 instances, each further = 1
                if let Some((_, owner)) = self
                    .piece_owner
                    .iter()
                    .find(|(oh, _)| *oh == *h)
                {
                    let owner = *owner;
                    if !self.robot_in_alliance_zone(owner) {
                        const WINDOW_TICKS: u64 =
                            (5.0 * PHYSICS_HZ) as u64;
                        self.zone_penalty_consec = match self.zone_penalty_last_tick
                        {
                            Some(t) if self.tick - t < WINDOW_TICKS => {
                                self.zone_penalty_consec + 1
                            }
                            _ => 1,
                        };
                        self.zone_penalty_last_tick = Some(self.tick);
                        let num = if self.zone_penalty_consec == 3 {
                            3
                        } else if self.zone_penalty_consec > 3 {
                            1
                        } else {
                            0
                        };
                        let oa = owner % 2;
                        self.zone_penalty_instances[oa] += num;
                    }
                }
                // capture into the hub: kinematic, zero velocity, sensor
                // collider — it rests inside the scorer, out of play
                {
                    let body = &mut self.bodies[*h];
                    body.set_body_type(
                        RigidBodyType::KinematicPositionBased,
                        true,
                    );
                    body.set_linvel(vector![0.0, 0.0, 0.0], true);
                    body.set_angvel(vector![0.0, 0.0, 0.0], true);
                    body.set_next_kinematic_translation(
                        vector![pos[0], pos[1], pos[2]],
                    );
                    let cols: Vec<ColliderHandle> =
                        body.colliders().to_vec();
                    for ch in cols {
                        if let Some(c) = self.colliders.get_mut(ch) {
                            c.set_sensor(true);
                        }
                    }
                }
                self.hub_held.push(HubHeldPiece {
                    handle: *h,
                    hub: hi,
                });
                // stays in `pieces` (kinematic, skipped below) — scored
                // fuel remains visible inside the hub like MoSim
                continue;
            }
        }

        // return timers: pieces flagged out-of-bounds teleport back to
        // their return point after RETURN_DELAY_S (MoSim MovePosition +
        // inward drift). Pieces captured/scored mid-timer drop out.
        let mut returned: Vec<ReturnState> = Vec::new();
        for r in &mut self.returning {
            if self.carried.iter().any(|c| c.handle == r.handle)
                || self.hub_held.iter().any(|c| c.handle == r.handle)
            {
                r.t = -1.0;
            } else {
                r.t -= self.integration.dt;
            }
            if r.t <= 0.0 {
                returned.push(*r);
            }
        }
        for r in returned {
            self.returning.retain(|x| x.handle != r.handle);
            let body = &mut self.bodies[r.handle];
            body.set_translation(
                vector![r.point[0], r.point[1], r.point[2]],
                true,
            );
            body.set_linvel(vector![0.0, 0.0, 0.0], true);
            body.set_angvel(vector![0.0, 0.0, 0.0], true);
            // drift back toward midfield so it re-enters play
            let (dx, dz) = (-r.point[0], -r.point[2]);
            let d = (dx * dx + dz * dz).sqrt().max(1e-3);
            body.set_linvel(vector![dx / d * 0.8, 0.0, dz / d * 0.8], true);
        }
    }

    /// Roller contact model: for every spinning joint with a `roller_pos`
    /// zone, free pieces inside the radius interact with the rollers.
    /// With `roller_axis` set, pieces are pushed along the roller's surface
    /// velocity ω × r — spin direction decides whether fuel is dragged in
    /// or ejected (MoSim InteractionRoller). Without it, pieces are pulled
    /// toward the intake center, scaled by spin fraction. Runs before
    /// `pipeline.step`; deterministic (no RNG).
    fn apply_roller_forces(&mut self) {
        // zones: (anchor, pull-target, radius, strength, axis, signed vel)
        let mut zones: Vec<([f32; 3], [f32; 3], f32, f32, Option<[f32; 3]>, f32)> =
            Vec::new();
        for i in 0..self.chassis.len() {
            let Some(intake) = self.intakes[i] else { continue };
            let (cpos, cyaw) = {
                let b = &self.bodies[self.chassis[i]];
                let r = b.rotation();
                let t = b.translation();
                ([t.x, t.y, t.z], yaw_from_quat([r.i, r.j, r.k, r.w]))
            };
            let (s, c) = cyaw.sin_cos();
            // intake center in world (x/z only — pull is horizontal)
            let icx = (intake.x_min + intake.x_max) * 0.5;
            let icz = (intake.z_min + intake.z_max) * 0.5;
            let target = [
                cpos[0] + icx * c + icz * s,
                cpos[1],
                cpos[2] - icx * s + icz * c,
            ];
            // arm-carried roller zones: (pivot, arm angle rel. chassis yaw)
            let arm = self.mechs.get(i).and_then(|ms| ms.first()).map(|m| {
                (m.spec.pivot, mech::arm_angle(&self.bodies, m.arm) - cyaw)
            });
            for js in self.joints.get(i).into_iter().flatten() {
                let Some(rp) = js.spec.roller_pos else { continue };
                // rollers must actually be turning to grab anything
                let svel = js.vel[0];
                if svel.abs() < 2.0 {
                    continue;
                }
                let anchor = Self::roller_anchor(rp, js.spec.on_arm, cpos, cyaw, arm);
                let radius = js.spec.roller_radius.unwrap_or(0.3);
                // strength scales with spin fraction of max_vel
                let strength =
                    (svel.abs() / js.spec.max_vel.max(1.0)).clamp(0.0, 1.0);
                let axis = js
                    .spec
                    .roller_axis
                    .map(|a| Self::roller_axis_world(a, js.spec.on_arm, cyaw, arm));
                zones.push((anchor, target, radius, strength, axis, svel));
            }
        }
        if zones.is_empty() {
            return;
        }
        for &h in &self.pieces {
            // kinematically-held pieces (carried / hub-scored) are glued —
            // rollers can't drag them
            if self.carried.iter().any(|cp| cp.handle == h)
                || self.hub_held.iter().any(|cp| cp.handle == h)
            {
                continue;
            }
            let (pos, zone) = {
                let body = &self.bodies[h];
                let t = body.translation();
                let p = [t.x, t.y, t.z];
                // nearest zone wins
                let mut best: Option<usize> = None;
                let mut best_d = f32::INFINITY;
                for (zi, (a, _, r, _, _, _)) in zones.iter().enumerate() {
                    let dx = p[0] - a[0];
                    let dz = p[2] - a[2];
                    let dy = p[1] - a[1];
                    let d2 = dx * dx + dz * dz;
                    if d2 < r * r && dy.abs() < r * 1.5 && d2 < best_d {
                        best_d = d2;
                        best = Some(zi);
                    }
                }
                (p, best)
            };
            let Some(zi) = zone else { continue };
            let (anchor, tgt, _, strength, axis, svel) = zones[zi];
            // push direction: roller surface velocity ω × r when an axis
            // is authored, else pull toward the intake center
            let (dx, dz) = match axis {
                Some(ax) => {
                    let rx = pos[0] - anchor[0];
                    let ry = pos[1] - anchor[1];
                    let rz = pos[2] - anchor[2];
                    // tangential surface velocity = ω_vec × r, ω_vec =
                    // svel·axis; only the horizontal part pushes fuel
                    let vx = svel * (ax[1] * rz - ax[2] * ry);
                    let vz = svel * (ax[0] * ry - ax[1] * rx);
                    let l = (vx * vx + vz * vz).sqrt();
                    if l < 1e-4 {
                        continue;
                    }
                    (vx / l, vz / l)
                }
                None => {
                    let dx = tgt[0] - pos[0];
                    let dz = tgt[2] - pos[2];
                    let d = (dx * dx + dz * dz).sqrt();
                    if d < 1e-4 {
                        continue;
                    }
                    (dx / d, dz / d)
                }
            };
            // ~20 m/s² peak — rollers move fuel without launching it
            let f = 4.0 * strength; // N — ~20 m/s² on a 0.2 kg piece
            let body = &mut self.bodies[h];
            body.add_force(vector![f * dx, 0.0, f * dz], true);
        }
    }

    /// Arm joint angle for mechanism telemetry/tests (issue #57 spike).
    /// `None` when the robot has no `[mech]`.
    /// REBUILT shift order: the alliance that scored more in AUTO owns the
    /// first teleop window (ties -> alliance 0). Captured once at the
    /// auto->teleop boundary.
    pub fn auto_winner(&self) -> usize {
        let sa = self.score_by_phase[0].auto;
        let sb = self.score_by_phase[1].auto;
        if sb > sa { 1 } else { 0 }
    }

    fn hub_active_at(&self, alliance: usize, tick: u64) -> bool {
        if self.match_clock.config.shift_s <= 0.0 {
            return true;
        }
        let mt = tick.saturating_sub(self.match_epoch);
        if self.match_clock.phase_at(mt) != crate::r#match::MatchPhase::Teleop {
            return true;
        }
        let b = self.match_clock.config.bounds();
        let teleop_t = mt as f32 / crate::r#match::TICK_HZ - b[1];
        let window = (teleop_t / self.match_clock.config.shift_s) as usize;
        let winner = self.auto_winner();
        (window % 2 == 0) == (alliance == winner)
    }

    /// Start the match: staging ends, the clock begins at auto (issue:
    /// match-ops lifecycle). disabled_s stays whatever config says — the
    /// epoch makes clock time start now.
    pub fn start_match(&mut self) {
        self.match_epoch = self.tick;
    }

    /// public wrapper for viz/control
    pub fn hub_active_for_viz(&self, alliance: usize, tick: u64) -> bool {
        self.hub_active_at(alliance, tick)
    }

    /// Stage preloaded fuel (game manual 6.3.4): each robot starts
    /// holding `preload_fuel` pieces.
    pub fn apply_preload(&mut self) {
        let n = self.match_clock.config.preload_fuel;
        for h in &mut self.held {
            *h = n;
        }
    }

    /// Set drivetrain wheel-carpet mu for every robot (M4 tuning knob —
    /// the parameter that actually moves trajectories post-#48).
    pub fn set_drivetrain_mu(&mut self, mu: f32) {
        for d in &mut self.drivetrains {
            d.mu = mu;
        }
    }

    /// Set the chassis collider friction for every robot (M4 tuning knob).
    pub fn set_chassis_friction(&mut self, friction: f32) {
        for &h in &self.chassis_colliders {
            if let Some(c) = self.colliders.get_mut(h) {
                c.set_friction(friction);
            }
        }
    }

    pub fn intake_active(&self, robot: usize) -> bool {
        self.intake_active.get(robot).copied().unwrap_or(false)
    }

    pub fn mech_angle(&self, robot: usize) -> Option<f32> {
        self.mechs[robot]
            .first()
            .map(|m| mech::arm_angle(&self.bodies, m.arm))
    }

    /// World position of the mechanism arm's center of mass (viz/tests).
    pub fn mech_arm_position(&self, robot: usize) -> Option<[f32; 3]> {
        self.mechs[robot].first().map(|m| {
            let t = self.bodies[m.arm].translation();
            [t.x, t.y, t.z]
        })
    }

    /// World position of one joint's roller contact anchor. `on_arm`
    /// joints resolve `roller_pos` in the mech arm's local frame (origin
    /// at the pivot, rotating with the arm angle); others treat it as a
    /// chassis-frame offset. `None` when the joint has no roller zone.
    pub fn joint_roller_anchor(&self, robot: usize, joint: usize) -> Option<[f32; 3]> {
        let js = self.joints.get(robot)?.get(joint)?;
        let rp = js.spec.roller_pos?;
        let b = &self.bodies[self.chassis[robot]];
        let t = b.translation();
        let r = b.rotation();
        let cpos = [t.x, t.y, t.z];
        let cyaw = yaw_from_quat([r.i, r.j, r.k, r.w]);
        let arm = self.mechs.get(robot).and_then(|ms| ms.first()).map(|m| {
            (m.spec.pivot, mech::arm_angle(&self.bodies, m.arm) - cyaw)
        });
        Some(Self::roller_anchor(rp, js.spec.on_arm, cpos, cyaw, arm))
    }

    /// World launch position for robot `i`'s shot: the shooter joint's
    /// `muzzle_pos` (chassis frame, yaw-rotated) or chassis center +
    /// `launch_height`. Associated fn (not `&self`) so feed code can call
    /// it while `carried` is mutably borrowed.
    fn shot_launch_pos(
        bodies: &RigidBodySet,
        chassis: RigidBodyHandle,
        joints: Option<&Vec<joint::JointSim>>,
        shot: &crate::ShotCommand,
    ) -> [f32; 3] {
        let b = &bodies[chassis];
        let t = b.translation();
        let r = b.rotation();
        let yaw = yaw_from_quat([r.i, r.j, r.k, r.w]);
        if let Some(m) = joints
            .into_iter()
            .flatten()
            .find(|j| j.spec.driver == crate::archetype::JointDriver::Shooter)
            .and_then(|j| j.spec.muzzle_pos)
        {
            let (s, c) = yaw.sin_cos();
            [
                t.x + m[0] * c + m[2] * s,
                t.y + m[1],
                t.z - m[0] * s + m[2] * c,
            ]
        } else {
            [t.x, t.y + shot.launch_height, t.z]
        }
    }

    /// Launch velocity for robot `i`'s shot at current joint/chassis
    /// state: turret joint angle (slew lag included) or commanded
    /// `turret_yaw`, hood angle, chassis velocity, and flywheel-derived
    /// exit speed when the shooter joint sets `launch_radius`.
    fn shot_launch_vel(
        bodies: &RigidBodySet,
        chassis: RigidBodyHandle,
        joints: Option<&Vec<joint::JointSim>>,
        shot: &crate::ShotCommand,
    ) -> [f32; 3] {
        let b = &bodies[chassis];
        let r = b.rotation();
        let lv = b.linvel();
        let yaw = yaw_from_quat([r.i, r.j, r.k, r.w]);
        // aim: a physical turret joint tracks the commanded yaw — launch
        // uses its actual angle, not the raw command
        let aim = yaw
            + joints
                .into_iter()
                .flatten()
                .find(|j| j.spec.driver == crate::archetype::JointDriver::Turret)
                .map(|j| j.pos[0])
                .unwrap_or(shot.turret_yaw);
        // hood: a physical position joint tracks the commanded angle —
        // launch elevation uses its actual position (slew lag is real)
        let hood = joints
            .into_iter()
            .flatten()
            .find(|j| j.spec.driver == crate::archetype::JointDriver::Hood)
            .map(|j| j.pos[0])
            .unwrap_or(shot.hood_angle);
        let hv = shot.launch_velocity * hood.cos();
        let mut vel = [
            hv * aim.cos() + lv.x,
            shot.launch_velocity * hood.sin(),
            hv * aim.sin() + lv.z,
        ];
        // physics-derived exit speed: `launch_radius` sets the magnitude
        // from actual flywheel ω (MoSim FlywheelBehaviour)
        if let Some(js) = joints
            .into_iter()
            .flatten()
            .find(|j| j.spec.driver == crate::archetype::JointDriver::Shooter)
        {
            if let Some(lr) = js.spec.launch_radius {
                let speed = js.vel[0] * lr * 1.8;
                let cur =
                    (vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]).sqrt();
                if cur > 1e-4 {
                    let k = speed / cur;
                    vel = [vel[0] * k, vel[1] * k, vel[2] * k];
                }
            }
        }
        vel
    }

    /// Return point for an out-of-bounds piece: backfield corner on the
    /// same alliance side it exited, inside the walls (MoSim
    /// ReturnPoints — elevated spawn, then drifts back to midfield).
    fn piece_return_point(pos: [f32; 3]) -> [f32; 3] {
        let sx = if pos[0] < 0.0 { -1.0 } else { 1.0 };
        [sx * 7.0, 0.4, -sx * 3.5]
    }

    /// Resolve a roller anchor to world coords. `arm` is the robot's
    /// (pivot, arm angle relative to chassis yaw) when it has a mech.
    fn roller_anchor(
        rp: [f32; 3],
        on_arm: bool,
        cpos: [f32; 3],
        cyaw: f32,
        arm: Option<([f32; 3], f32)>,
    ) -> [f32; 3] {
        let (s, c) = cyaw.sin_cos();
        // chassis-frame anchor: `on_arm` joints ride the mech arm —
        // roller_pos is arm-local (origin at pivot, +x along the arm)
        // and rotates with the arm angle about the pivot's lateral axis
        let (ax, ay, az) = if on_arm {
            match arm {
                Some(([px, py, pz], th)) => {
                    let (st, ct) = th.sin_cos();
                    (
                        px + rp[0] * ct - rp[1] * st,
                        py + rp[0] * st + rp[1] * ct,
                        pz + rp[2],
                    )
                }
                None => (rp[0], rp[1], rp[2]),
            }
        } else {
            (rp[0], rp[1], rp[2])
        };
        [
            cpos[0] + ax * c + az * s,
            cpos[1] + ay,
            cpos[2] - ax * s + az * c,
        ]
    }

    /// Resolve a `roller_axis` direction to world coords using the same
    /// frame chain as `roller_anchor` (arm-local pitch, then chassis yaw).
    /// Directions don't translate, so only rotations apply.
    fn roller_axis_world(
        axis: [f32; 3],
        on_arm: bool,
        cyaw: f32,
        arm: Option<([f32; 3], f32)>,
    ) -> [f32; 3] {
        let local = if on_arm {
            match arm {
                Some((_, th)) => {
                    let (st, ct) = th.sin_cos();
                    [
                        axis[0] * ct - axis[1] * st,
                        axis[0] * st + axis[1] * ct,
                        axis[2],
                    ]
                }
                None => axis,
            }
        } else {
            axis
        };
        let (x, y, z) = (local[0], local[1], local[2]);
        let (s, c) = cyaw.sin_cos();
        [x * c + z * s, y, -x * s + z * c]
    }

    /// Ground-truth extraction + orchestrator-side sensor filtering (issue
    /// #7). `&mut self` because the noise model draws from the world `SimRng`
    /// and advances its vision latency queue — exactly one call per robot
    /// per agent tick keeps server, replay, and tests in lockstep.
    pub fn sensor_frame(&mut self, robot: usize, tick: u64) -> SensorFrame {
        let body = &self.bodies[self.chassis[robot]];
        let t = body.translation();
        let r = body.rotation();
        let lv = body.linvel();
        let av = body.angvel();
        // robot-vs-robot contacts only (filter out ground/walls);
        // aggregated across every other chassis
        // real solver impulse (N·s), not a manifold point count (issue #27)
        let contact_impulse: f32 = self
            .chassis_colliders
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != robot)
            .map(|(_, other)| {
                self.narrow_phase
                    .contact_pairs_with(self.chassis_colliders[robot])
                    .filter(|p| p.collider1 == *other || p.collider2 == *other)
                    .map(|p| p.total_impulse_magnitude())
                    .sum::<f32>()
            })
            .sum::<f32>();
        let mut frame = SensorFrame {
            tick,
            position: [t.x, t.y, t.z],
            rotation: [r.i, r.j, r.k, r.w],
            linvel: [lv.x, lv.y, lv.z],
            angvel: [av.x, av.y, av.z],
            contact_impulse,
            pieces_held: self.held[robot],
            score: self.scores[robot % self.scores.len()],
            match_state: self.phase_at_tick(tick),
            match_time: self.time_remaining_at_tick(tick),
            vision_pose: None,
            vision_tags: Vec::new(),
            mech_angle: self.mech_angle(robot),
            hub_active: self.hub_active_at(robot % self.scores.len(), tick),
            joints: self.joint_states(robot),
        };
        // AprilTag detection runs on ground truth before the noise layer
        // (a real camera resolves tags regardless of estimator noise).
        // Robots without `[camera]` get `None` → legacy ungated vision.
        let tag_view = self
            .cameras
            .get(robot)
            .and_then(|c| c.as_ref())
            .map(|cam| vision::detect(cam, &self.field.apriltags, frame.position, frame.rotation));
        frame.vision_tags = tag_view
            .as_ref()
            .map(|v| v.ids.clone())
            .unwrap_or_default();
        // noise layer: no-op (and zero RNG draws) when disabled
        let true_pos = frame.position;
        let vision_gate = tag_view.map(|v| v.nearest_m);
        self.sensors[robot].apply(&mut self.rng, &mut frame, true_pos, vision_gate);
        frame
    }

    /// Entity snapshot for RL observation building (issue #12): every
    /// chassis pose/vel and every live piece position — the global-state
    /// view the viz also broadcasts. Own-robot scalars in the obs still
    /// come from the filtered `SensorFrame`; this feeds the
    /// nearest-K/rays entity features only.
    pub fn obs_world(&self) -> crate::obs::ObsWorld {
        let robots = self
            .chassis
            .iter()
            .map(|h| {
                let b = &self.bodies[*h];
                let t = b.translation();
                let v = b.linvel();
                ([t.x, t.y, t.z], [v.x, v.y, v.z])
            })
            .collect();
        crate::obs::ObsWorld {
            robots,
            pieces: self.piece_positions(),
            field_half: [self.field.half_x, self.field.half_z],
            scores: self.scores,
        }
    }

    /// Telemetry for one robot's `[[joint]]` mechanism joints (viz +
    /// SensorFrame encoder feedback).
    pub fn joint_states(&self, robot: usize) -> Vec<crate::protocol::JointState> {
        self.joints
            .get(robot)
            .map(|js| js.iter().map(|j| j.state()).collect())
            .unwrap_or_default()
    }

    /// Scored pieces currently held inside hub `hub` — telemetry/viz can
    /// show fuel accumulating in the scorer (MoSim occupancy model).
    pub fn hub_held_count(&self, hub: usize) -> usize {
        self.hub_held.iter().filter(|c| c.hub == hub).count()
    }

    /// World positions of all live (not captured/scored) pieces, for the
    /// spectator state stream.
    pub fn piece_positions(&self) -> Vec<[f32; 3]> {
        self.pieces
            .iter()
            .map(|h| {
                let t = self.bodies[*h].translation();
                [t.x, t.y, t.z]
            })
            .collect()
    }
}


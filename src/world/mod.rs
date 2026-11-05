//! The authoritative Rapier world: arena, chassis, stepping, and sensor
//! extraction (PLAN A4). No networking, no wall-clock — the world is
//! advanced one fixed step at a time by the caller, which is what makes the
//! simulation deterministic and replayable (issue #5).
pub mod chassis;
pub mod field;
pub mod noise;
pub mod pieces;

pub use crate::archetype::IntakeSpec;
pub use chassis::{RobotSpec, NUM_ROBOTS, ROBOT_HALF, ROBOT_MASS};
pub use field::{FieldSpec, FIELD_HALF_X, FIELD_HALF_Z};
pub use pieces::{HubZone, PieceTable, SpawnZone};

use crate::drivetrain::{rot2, yaw_from_quat, Drivetrain};
use crate::r#match::{MatchClock, MatchPhase, PhaseScore};
use crate::rng::SimRng;
use crate::{ActuatorFrame, SensorFrame};
use noise::SensorNoise;
use rapier3d::prelude::*;

pub const PHYSICS_HZ: f32 = 400.0;
pub const AGENT_EVERY: u32 = 8; // 50 Hz agent tick

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
    /// Seeded RNG — the only legal source of randomness in world code
    /// (PLAN §0). Piece spawns / sensor noise draw here.
    pub rng: SimRng,
    /// The field layout this world was built from (issue #14); viz reads
    /// it to emit the config message, and it feeds the replay config hash.
    pub field: FieldSpec,
    /// live game-piece bodies (issue #3); scored/captured pieces are removed
    pub pieces: Vec<RigidBodyHandle>,
    /// per-robot held-piece inventory, fed by intake captures and consumed
    /// by shots
    pub held: Vec<u32>,
    /// per-alliance score (alliance index = robot_id % 2)
    pub scores: [u32; 2],
    hubs: Vec<HubZone>,
    intakes: Vec<Option<IntakeSpec>>,
    /// sticky intake-run flag per robot, updated from actuator frames
    intake_active: Vec<bool>,
    /// physics ticks elapsed — the world's only clock (issue #10); the
    /// match phase is a pure function of this via `match_clock`
    tick: u64,
    /// match state machine config (issue #10); default = start enabled
    /// (disabled_s = 0) so bare worlds behave exactly like before
    pub match_clock: MatchClock,
    /// per-alliance score attributed to the phase it was scored in
    /// (auto / teleop / endgame+bonus); `scores` stays the running total
    pub score_by_phase: [PhaseScore; 2],
    /// set once the climb bonus has been awarded at match completion
    match_finished: bool,
    /// per-robot sensor noise/latency models (issue #7); drawn from `rng`
    sensors: Vec<SensorNoise>,
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
            rng: SimRng::seeded(seed),
            field,
            pieces: Vec::new(),
            held: vec![0; specs.len()],
            scores: [0; 2],
            hubs: table.hubs.clone(),
            intakes: specs.iter().map(|s| s.intake).collect(),
            intake_active: vec![false; specs.len()],
            tick: 0,
            match_clock: MatchClock::default(),
            score_by_phase: [PhaseScore::default(); 2],
            match_finished: false,
            sensors: Vec::new(),
        };

        field::build_arena(&w.field, &mut w.bodies, &mut w.colliders);
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
            println!(
                "robot {} spawned at x={} mass={} half={:?}",
                i, spec.spawn[0], spec.mass, spec.half_extents
            );
        }
        // per-boot sensor biases draw from the world RNG in robot order
        // (disabled configs draw nothing → identical stream to before)
        w.sensors = specs
            .iter()
            .map(|s| SensorNoise::new(s.sensors, &mut w.rng))
            .collect();
        w
    }

    pub fn num_robots(&self) -> usize {
        self.chassis.len()
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
        let phase = self.match_clock.phase_at(self.tick);
        let enabled = phase.enabled();
        for (i, handle) in self.chassis.iter().enumerate() {
            let cmd = if enabled {
                commands.get(i).and_then(|c| c.as_ref())
            } else {
                None
            };
            if let Some(cmd) = cmd {
                // per-wheel traction model: resolve the actuator command into
                // module forces (grip-limited), then apply them off-center so
                // yaw torque emerges from the force placement.
                let (yaw, pos, lv) = {
                    let body = &self.bodies[*handle];
                    let r = body.rotation();
                    (
                        yaw_from_quat([r.i, r.j, r.k, r.w]),
                        *body.translation(),
                        *body.linvel(),
                    )
                };
                self.intake_active[i] = cmd.intake_active;
                // shooter: consume a held piece and spawn a ballistic fuel
                // (FuelSim launchFuel: hood angle, turret yaw, launch height,
                // plus chassis velocity)
                if let Some(shot) = cmd.shoot {
                    if self.held[i] > 0 {
                        self.held[i] -= 1;
                        let aim = yaw + shot.turret_yaw;
                        let hv = shot.launch_velocity * shot.hood_angle.cos();
                        let vel = [
                            hv * aim.cos() + lv.x,
                            shot.launch_velocity * shot.hood_angle.sin(),
                            hv * aim.sin() + lv.z,
                        ];
                        self.pieces.push(pieces::spawn_piece(
                            &mut self.bodies,
                            &mut self.colliders,
                            [pos.x, pos.y + shot.launch_height, pos.z],
                            vel,
                        ));
                    }
                }
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
        // quadratic air drag on free pieces (FuelSim DRAG_FORCE_FACTOR)
        pieces::apply_drag(&mut self.bodies, &self.pieces);
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
        // endgame/climb placeholder (issue #10): on the transition into
        // `complete`, any robot parked within `climb_radius_m` of a field
        // climb point banks the bonus for its alliance
        if !self.match_finished
            && self.match_clock.phase_at(self.tick) == MatchPhase::Complete
        {
            self.award_climb_bonus();
            self.match_finished = true;
        }
    }

    /// Physics ticks elapsed since world construction.
    pub fn tick(&self) -> u64 {
        self.tick
    }

    /// One-shot endgame bonus: each robot within horizontal
    /// `climb_radius_m` of any field climb point scores
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
        let phase = self.match_clock.phase_at(self.tick);
        // robot poses for intake tests (only robots with an active intake;
        // intake is a mechanism command, so it's inert while disabled)
        let robots: Vec<Option<([f32; 3], f32)>> = (0..self.chassis.len())
            .map(|i| {
                if phase.enabled()
                    && self.intake_active[i]
                    && self.intakes[i].is_some()
                {
                    let body = &self.bodies[self.chassis[i]];
                    let t = body.translation();
                    let r = body.rotation();
                    Some((
                        [t.x, t.y, t.z],
                        yaw_from_quat([r.i, r.j, r.k, r.w]),
                    ))
                } else {
                    None
                }
            })
            .collect();

        let mut remove: Vec<usize> = Vec::new();
        for (pi, h) in self.pieces.iter().enumerate() {
            let body = &self.bodies[*h];
            let t = body.translation();
            let v = body.linvel();
            let pos = [t.x, t.y, t.z];

            // intake capture
            let mut captured = false;
            for (i, robot) in robots.iter().enumerate() {
                if let Some((rpos, ryaw)) = robot {
                    if pieces::intake_contains(
                        &self.intakes[i].unwrap(),
                        rpos,
                        *ryaw,
                        &pos,
                    ) {
                        self.held[i] += 1;
                        remove.push(pi);
                        captured = true;
                        break;
                    }
                }
            }
            if captured {
                continue;
            }

            // hub scoring: first matching hub wins; the point is also
            // attributed to the match phase it was scored in (issue #10)
            if let Some(hub) = self
                .hubs
                .iter()
                .find(|hub| pieces::hub_scores(hub, &pos, &[v.x, v.y, v.z]))
            {
                let a = (hub.alliance as usize).min(self.scores.len() - 1);
                self.scores[a] += 1;
                self.score_by_phase[a].add(phase, 1);
                remove.push(pi);
            }
        }

        // remove captured/scored pieces (descending order keeps indices valid)
        remove.sort_unstable_by(|a, b| b.cmp(a));
        for pi in remove {
            let h = self.pieces.swap_remove(pi);
            self.bodies.remove(
                h,
                &mut self.islands,
                &mut self.colliders,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                true,
            );
        }
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
        let contact_impulse: f32 = self
            .chassis_colliders
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != robot)
            .map(|(_, other)| {
                self.narrow_phase
                    .contact_pairs_with(self.chassis_colliders[robot])
                    .filter(|p| p.collider1 == *other || p.collider2 == *other)
                    .map(|p| p.manifolds.iter().map(|m| m.points.len()).sum::<usize>())
                    .sum::<usize>()
            })
            .sum::<usize>() as f32;
        let mut frame = SensorFrame {
            tick,
            position: [t.x, t.y, t.z],
            rotation: [r.i, r.j, r.k, r.w],
            linvel: [lv.x, lv.y, lv.z],
            angvel: [av.x, av.y, av.z],
            contact_impulse,
            pieces_held: self.held[robot],
            score: self.scores[robot % self.scores.len()],
            match_state: self.match_clock.phase_at(tick),
            match_time: self.match_clock.time_remaining(tick),
            vision_pose: None,
        };
        // noise layer: no-op (and zero RNG draws) when disabled
        let true_pos = frame.position;
        self.sensors[robot].apply(&mut self.rng, &mut frame, true_pos);
        frame
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


//! Headless physics world, extracted so tests and tools can run the same
//! simulation the orchestrator runs — no networking, no wall-clock.
//! Kept independent of `bin/server.rs` so other agents can refactor the
//! networked shell without breaking regression tests.
use crate::ActuatorFrame;
use rapier3d::prelude::*;

pub const PHYSICS_HZ: f32 = 400.0;
pub const NUM_ROBOTS: usize = 2;
pub const ROBOT_HALF: f32 = 0.4;
pub const ROBOT_MASS: f32 = 60.0;
pub const FIELD_HALF_X: f32 = 8.2705; // fieldcore: 16.541 m field (issue #44)
pub const FIELD_HALF_Z: f32 = 4.0345; // fieldcore: 8.069 m field (issue #44)

/// One robot's spawn-time physical parameters (archetype). Defaults match
/// the historical hard-coded chassis so existing behavior is unchanged.
#[derive(Debug, Clone, Copy)]
pub struct RobotSpec {
    pub mass: f32,
    /// cuboid half-extents [x, y, z]
    pub half_extents: [f32; 3],
    pub spawn: [f32; 3],
}

impl Default for RobotSpec {
    fn default() -> Self {
        RobotSpec {
            mass: ROBOT_MASS,
            half_extents: [ROBOT_HALF; 3],
            spawn: [0.0, ROBOT_HALF + 0.05, 0.0],
        }
    }
}

impl RobotSpec {
    /// Spawn `x` on the field, chassis resting on the ground.
    pub fn at_x(x: f32) -> Self {
        let mut s = Self::default();
        s.spawn = [x, s.half_extents[1] + 0.05, 0.0];
        s
    }
    pub fn with_mass(mut self, mass: f32) -> Self {
        self.mass = mass;
        self
    }
    pub fn with_half_extents(mut self, h: [f32; 3]) -> Self {
        self.half_extents = h;
        self.spawn[1] = h[1] + 0.05;
        self
    }
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
}

impl World {
    /// Default two-robot field: robot 0 at x=-4, robot 1 at x=+4.
    pub fn new() -> Self {
        Self::with_specs(&[RobotSpec::at_x(-4.0), RobotSpec::at_x(4.0)])
    }

    pub fn with_specs(specs: &[RobotSpec]) -> Self {
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
        };

        // ground
        let ground = w.bodies.insert(RigidBodyBuilder::fixed());
        w.colliders.insert_with_parent(
            ColliderBuilder::cuboid(20.0, 0.1, 20.0)
                .translation(vector![0.0, -0.1, 0.0])
                .friction(1.0)
                .build(),
            ground,
            &mut w.bodies,
        );
        // field walls
        for (x, z, hx, hz) in [
            (0.0, -FIELD_HALF_Z, FIELD_HALF_X, 0.05),
            (0.0, FIELD_HALF_Z, FIELD_HALF_X, 0.05),
            (-FIELD_HALF_X, 0.0, 0.05, FIELD_HALF_Z),
            (FIELD_HALF_X, 0.0, 0.05, FIELD_HALF_Z),
        ] {
            w.colliders.insert_with_parent(
                ColliderBuilder::cuboid(hx, 0.6, hz)
                    .translation(vector![x, 0.3, z])
                    .friction(0.6)
                    .build(),
                ground,
                &mut w.bodies,
            );
        }

        for spec in specs {
            let body = w.bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(vector![spec.spawn[0], spec.spawn[1], spec.spawn[2]])
                    .additional_mass(spec.mass)
                    .linear_damping(0.02) // fieldcore chassis damping (issue #44)
                    .angular_damping(0.08)
                    // chassis stays upright (2.5D drive, 3D contacts)
                    .enabled_rotations(false, true, false)
                    .ccd_enabled(true)
                    .build(),
            );
            let [hx, hy, hz] = spec.half_extents;
            let col = w.colliders.insert_with_parent(
                ColliderBuilder::cuboid(hx, hy, hz)
                    // fieldcore chassis collider (issue #40/#44): low
                    // rolling-resistance friction, Min combine so the
                    // pair vs the grippy ground stays ~0.04
                    .friction(0.04)
                    .friction_combine_rule(CoefficientCombineRule::Min)
                    .restitution(0.01)
                    .density(1.0)
                    .build(),
                body,
                &mut w.bodies,
            );
            w.chassis.push(body);
            w.chassis_colliders.push(col);
        }
        w
    }

    pub fn num_robots(&self) -> usize {
        self.chassis.len()
    }

    /// Apply per-robot actuator commands (index = robot), then advance one tick.
    pub fn step(&mut self, commands: &[Option<ActuatorFrame>]) {
        for (i, handle) in self.chassis.iter().enumerate() {
            if let Some(Some(cmd)) = commands.get(i) {
                let body = &mut self.bodies[*handle];
                body.reset_forces(true);
                body.reset_torques(true);
                body.add_force(vector![cmd.force[0], 0.0, cmd.force[1]], true);
                body.add_torque(vector![0.0, cmd.torque, 0.0], true);
            }
        }
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
    }

    pub fn position(&self, robot: usize) -> [f32; 3] {
        let t = self.bodies[self.chassis[robot]].translation();
        [t.x, t.y, t.z]
    }

    /// Number of robot-vs-robot contact manifold points felt by `robot`
    /// (filters out ground/wall contacts).
    pub fn robot_contact_points(&self, robot: usize, other: usize) -> usize {
        let other_col = self.chassis_colliders[other];
        self.narrow_phase
            .contact_pairs_with(self.chassis_colliders[robot])
            .filter(|p| p.collider1 == other_col || p.collider2 == other_col)
            .map(|p| p.manifolds.iter().map(|m| m.points.len()).sum::<usize>())
            .sum()
    }
}

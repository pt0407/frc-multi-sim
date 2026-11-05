//! Chassis spawn specs and rigid-body construction (PLAN A4).
//!
//! `RobotSpec` is the spawn-time physical description of one robot — what an
//! archetype config resolves to. Spawn pose is stored per-spec so replay
//! headers can carry the exact initial conditions.
use crate::archetype::{Archetype, IntakeSpec};
use rapier3d::prelude::*;

pub const ROBOT_HALF: f32 = 0.4; // ~0.8m square chassis
pub const ROBOT_MASS: f32 = 60.0; // kg
pub const NUM_ROBOTS: usize = 2;

/// Spawn-time physical params for one robot (archetype or default).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RobotSpec {
    pub mass: f32,
    /// cuboid half-extents [x, y, z]
    pub half_extents: [f32; 3],
    /// spawn translation [x, y, z]
    pub spawn: [f32; 3],
    /// robot-relative intake capture region (issue #3); `None` = no intake
    pub intake: Option<IntakeSpec>,
}

impl RobotSpec {
    /// Default chassis resting on the ground at world x = `x`.
    pub fn at_x(x: f32) -> Self {
        RobotSpec {
            mass: ROBOT_MASS,
            half_extents: [ROBOT_HALF; 3],
            spawn: [x, ROBOT_HALF + 0.05, 0.0],
            intake: None,
        }
    }

    /// Build a spec at spawn x = `x` from an archetype config.
    pub fn from_archetype(a: &Archetype, x: f32) -> Self {
        RobotSpec {
            mass: a.mass,
            half_extents: a.half_extents,
            spawn: [x, a.half_extents[1] + 0.05, 0.0],
            intake: a.intake,
        }
    }
}

/// Default two-robot lineup: robot 0 at x=-4, robot 1 at x=+4.
pub fn default_specs() -> Vec<RobotSpec> {
    vec![RobotSpec::at_x(-4.0), RobotSpec::at_x(4.0)]
}

/// Insert the chassis rigid body + collider for `spec`.
pub fn spawn_chassis(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    spec: &RobotSpec,
) -> (RigidBodyHandle, ColliderHandle) {
    let [hx, hy, hz] = spec.half_extents;
    let body = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(vector![spec.spawn[0], spec.spawn[1], spec.spawn[2]])
            .additional_mass(spec.mass)
            .linear_damping(1.0)
            .angular_damping(4.0)
            // lock X/Z rotation: chassis stays upright (2.5D drive, 3D contacts)
            .enabled_rotations(false, true, false)
            .ccd_enabled(true)
            .build(),
    );
    let col = colliders.insert_with_parent(
        ColliderBuilder::cuboid(hx, hy, hz)
            .friction(0.9)
            .restitution(0.1)
            .density(1.0)
            .build(),
        body,
        bodies,
    );
    (body, col)
}

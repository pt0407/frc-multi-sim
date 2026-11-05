//! Chassis spawn specs and rigid-body construction (PLAN A4).
//!
//! `RobotSpec` is the spawn-time physical description of one robot — what an
//! archetype config resolves to. Spawn pose is stored per-spec so replay
//! headers can carry the exact initial conditions.
use crate::archetype::{Archetype, IntakeSpec};
use crate::world::noise::SensorConfig;
use rapier3d::prelude::*;

pub const ROBOT_HALF: f32 = 0.4; // ~0.8m square chassis
pub const ROBOT_MASS: f32 = 60.0; // kg
/// Default lineup size (issue #11): 3 per alliance, alliance = robot_id % 2.
pub const NUM_ROBOTS: usize = 6;
/// Distance of each alliance's station row from midfield (m).
pub const ALLIANCE_X: f32 = 4.0;
/// Spacing between stations along z within an alliance row (m).
pub const STATION_SPACING: f32 = 2.0;

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
    /// resolved sensor-noise config (issue #7); disabled = pre-noise output
    pub sensors: SensorConfig,
}

impl RobotSpec {
    /// Default chassis resting on the ground at world x = `x`.
    pub fn at_x(x: f32) -> Self {
        RobotSpec {
            mass: ROBOT_MASS,
            half_extents: [ROBOT_HALF; 3],
            spawn: [x, ROBOT_HALF + 0.05, 0.0],
            intake: None,
            sensors: SensorConfig::default(),
        }
    }

    /// Build a spec at spawn `[x, z]` from an archetype config.
    pub fn from_archetype(a: &Archetype, xz: [f32; 2]) -> Self {
        RobotSpec {
            mass: a.mass,
            half_extents: a.half_extents,
            spawn: [xz[0], a.half_extents[1] + 0.05, xz[1]],
            intake: a.intake,
            sensors: a.sensors.resolved(),
        }
    }
}

/// Station z offsets along an alliance row (m). Station 0 sits at
/// midfield so robots 0/1 keep their historical spawns (x=∓4, z=0);
/// stations 1 and 2 fill in on either side.
pub const STATION_Z: [f32; 3] = [0.0, -STATION_SPACING, STATION_SPACING];

/// Default spawn spec for one robot slot (issue #11): alliance rows.
/// Alliance = `robot_id % 2` (even ids on -x, odd on +x); the station
/// index `robot_id / 2` picks the slot's z position per `STATION_Z`,
/// so ids 0..6 spawn at x=∓4 with z=0, -2, +2. Slots beyond 6 wrap
/// through the stations.
pub fn default_spec(robot_id: usize) -> RobotSpec {
    let alliance = robot_id % 2;
    let station = robot_id / 2;
    let x = if alliance == 0 {
        -ALLIANCE_X
    } else {
        ALLIANCE_X
    };
    let mut spec = RobotSpec::at_x(x);
    spec.spawn[2] = STATION_Z[station % STATION_Z.len()];
    spec
}

/// Default `NUM_ROBOTS` lineup (6 = 3 per alliance).
pub fn default_specs() -> Vec<RobotSpec> {
    (0..NUM_ROBOTS).map(default_spec).collect()
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

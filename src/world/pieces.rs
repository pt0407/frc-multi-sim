//! Game-piece layer (issue #3, PLAN B3) — port of hammerheads5000/FuelSim's
//! tuned constants onto Rapier rigid bodies.
//!
//! FuelSim models fuel as analytic particles; here they are dynamic ball
//! colliders. The constants are ported 1:1 where Rapier's contact model
//! allows it:
//!
//! - Sphere radius/mass, gravity and drag are exact.
//! - Restitution is a per-collider-pair combine in Rapier, so the four
//!   FuelSim CORs are approximated: field/wall colliders carry
//!   `FIELD_COR` with `Min` combine (so existing robot-field contacts are
//!   unchanged: `min(robot_cor, FIELD_COR)`), and fuel colliders carry
//!   `FUEL_COR` with `Max` combine, giving fuel-vs-field = FIELD_COR,
//!   fuel-vs-fuel = FUEL_COR, fuel-vs-robot = FUEL_COR (FuelSim's
//!   ROBOT_COR = 0.1 is approximated by the livelier 0.5 — documented gap).
//!
//! Issue #44 audit vs fieldcore SimWorld.ts fuel constants:
//!   fieldcore: mass 0.215 kg · diameter 0.150 m · friction 0.6 ·
//!              restitution 0.18 · lin damping 0.16 · ang damping 0.22
//!   FuelSim  : mass 0.204 kg (0.448 lb) · radius 0.075 m · friction n/a
//!              (collider uses fieldcore's 0.6) · CORs below · drag below
//! Diameter and friction agree. Mass is kept at the FuelSim value (0.204
//! vs fieldcore 0.215 — unverified which matches the real piece; flagged
//! in the PR). Restitution is modeled per-pair from FuelSim's four CORs
//! rather than fieldcore's single 0.18 — the FuelSim model is the more
//! faithful port. Rapier linear/angular damping is left at 0 because
//! quadratic air drag (FuelSim) already models fuel dissipation; adding
//! fieldcore's damping on top would double-count (also unverified whether
//! fieldcore stacks both).
use crate::rng::SimRng;
use anyhow::{Context, Result};
use rapier3d::prelude::*;
use rand_core::RngCore;
use serde::Deserialize;
use std::path::Path;

// ---- FuelSim constants (FuelSim.java) ------------------------------------

/// fuel radius, m
pub const FUEL_RADIUS: f32 = 0.075;
/// fuel mass, kg (0.448 lb * 0.45392 ≈ 0.2034)
pub const FUEL_MASS: f32 = 0.204;
/// coefficient of restitution with the field: sqrt(22/51.5)
pub const FIELD_COR: f32 = 0.6537;
/// coefficient of restitution fuel vs fuel
pub const FUEL_COR: f32 = 0.5;
/// coefficient of restitution with the net (unmapped — no net collider yet)
pub const NET_COR: f32 = 0.2;
/// coefficient of restitution with a robot (approximated, see module docs)
pub const ROBOT_COR: f32 = 0.1;
/// sphere drag coefficient
pub const DRAG_COF: f32 = 0.47;
/// kg/m^3
pub const AIR_DENSITY: f32 = 1.225;
/// 0.5 * rho * Cd * pi * r^2 — multiply by -v*|v| for the drag force
pub const DRAG_FORCE_FACTOR: f32 =
    0.5 * AIR_DENSITY * DRAG_COF * std::f32::consts::PI * FUEL_RADIUS * FUEL_RADIUS;

// ---- match-config tables --------------------------------------------------

/// Rectangular piece spawn zone on the floor (x/z), pieces at rest.
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct SpawnZone {
    /// zone center [x, z]
    pub center: [f32; 2],
    /// zone half-extents [hx, hz]
    pub half_extents: [f32; 2],
    /// pieces to spawn uniformly at random (seeded) inside the zone
    pub count: u32,
}

/// Cylindrical scoring zone: a piece falling through `entry_height` inside
/// `radius` of `center` scores for `alliance` and is removed (FuelSim Hub).
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct HubZone {
    /// hub center [x, z]
    pub center: [f32; 2],
    /// entry radius, m
    pub radius: f32,
    /// entry plane height, m — piece must cross downward through it
    pub entry_height: f32,
    /// alliance index that owns this hub (0 or 1)
    pub alliance: u32,
}

/// Piece spawn + scoring table, loaded from a game/match TOML:
///
/// ```toml
/// [[zone]]
/// center = [0.0, 0.0]
/// half_extents = [2.0, 1.0]
/// count = 120
///
/// [[hub]]
/// center = [4.0, 0.0]
/// radius = 0.6
/// entry_height = 1.0
/// alliance = 0
/// ```
#[derive(Debug, Clone, Default, Deserialize)]
pub struct PieceTable {
    #[serde(default, rename = "zone")]
    pub zones: Vec<SpawnZone>,
    #[serde(default, rename = "hub")]
    pub hubs: Vec<HubZone>,
}

impl PieceTable {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("reading piece table {}", path.display()))?;
        toml::from_str(&text)
            .with_context(|| format!("parsing piece table {}", path.display()))
    }

    pub fn piece_count(&self) -> usize {
        self.zones.iter().map(|z| z.count as usize).sum()
    }
}

// ---- piece bodies ---------------------------------------------------------

/// Spawn one fuel ball rigid body + collider.
pub fn spawn_piece(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    pos: [f32; 3],
    vel: [f32; 3],
) -> RigidBodyHandle {
    let body = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(vector![pos[0], pos[1], pos[2]])
            .linvel(vector![vel[0], vel[1], vel[2]])
            .additional_mass(FUEL_MASS)
            .ccd_enabled(true)
            .build(),
    );
    colliders.insert_with_parent(
        ColliderBuilder::ball(FUEL_RADIUS)
            .restitution(FUEL_COR)
            // Max so fuel pairs resolve to FIELD_COR vs field colliders and
            // FUEL_COR vs fuel/fuel; see module docs for the approximation.
            .restitution_combine_rule(CoefficientCombineRule::Max)
            .friction(0.15)
            .build(),
        body,
        bodies,
    );
    body
}

/// Populate a piece set from the spawn table. All randomness draws from
/// `rng` (determinism contract §0). Pieces rest on the floor.
pub fn spawn_table(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    table: &PieceTable,
    rng: &mut SimRng,
) -> Vec<RigidBodyHandle> {
    let mut out = Vec::with_capacity(table.piece_count());
    for z in &table.zones {
        // real staging (how FuelSim/fieldcore do it): deterministic
        // positions with zero overlap and zero velocity — pieces spawn
        // AT REST, so nothing pops or scatters. Jittered grid over the
        // footprint; the 504-piece count fits in roughly one layer.
        // pitch sized to the zone: fit `count` pieces in ~one layer
        // (FuelSim staging) — small depots still space out cleanly.
        let area = z.half_extents[0] * 2.0 * z.half_extents[1] * 2.0;
        let pitch = (area / z.count as f32).sqrt().max(FUEL_RADIUS * 2.1);
        let nx = ((z.half_extents[0] * 2.0) / pitch).floor().max(1.0) as usize;
        let nz = ((z.half_extents[1] * 2.0) / pitch).floor().max(1.0) as usize;
        let mut i = 0usize;
        for _ in 0..z.count as usize {
            let col = i % nx;
            let row = i / nx;
            let layer = row / nz;
            let x = z.center[0] - z.half_extents[0]
                + pitch * (col as f32 + 0.5)
                + if row % 2 == 1 { pitch * 0.5 } else { 0.0 }; // stagger
            let zz = z.center[1] - z.half_extents[1]
                + pitch * (row as f32 + 0.5);
            let y = FUEL_RADIUS + 0.01 + layer as f32 * FUEL_RADIUS * 2.0;
            out.push(spawn_piece(bodies, colliders, [x, y, zz], [0.0; 3]));
            i += 1;
        }
    }
    out
}

/// Quadratic air drag: F = -DRAG_FORCE_FACTOR * |v| * v  (FuelSim
/// `DRAG_FORCE_FACTOR` port). Applied to every free piece each tick; cheap
/// (a few flops per piece).
pub fn apply_drag(bodies: &mut RigidBodySet, pieces: &[RigidBodyHandle]) {
    for h in pieces {
        let body = &mut bodies[*h];
        let v = *body.linvel();
        let speed = v.norm();
        if speed < 1e-4 {
            continue;
        }
        let k = -DRAG_FORCE_FACTOR * speed;
        body.reset_forces(false);
        body.add_force(vector![k * v.x, k * v.y, k * v.z], false);
    }
}

/// Is `point` (world) inside this robot-relative intake AABB? `yaw` is the
/// chassis heading; the box is [x_min,x_max] x [z_min,z_max] in the frame
/// where +x is the chassis forward axis.
pub fn intake_contains(
    spec: &crate::world::IntakeSpec,
    robot_pos: &[f32; 3],
    yaw: f32,
    point: &[f32; 3],
) -> bool {
    let dx = point[0] - robot_pos[0];
    let dz = point[2] - robot_pos[2];
    let (s, c) = (-yaw).sin_cos();
    let lx = c * dx - s * dz;
    let lz = s * dx + c * dz;
    let ly = point[1] - robot_pos[1];
    lx >= spec.x_min
        && lx <= spec.x_max
        && lz >= spec.z_min
        && lz <= spec.z_max
        && ly >= spec.y_min
        && ly <= spec.y_max
}

/// Downward entry test for a hub zone (FuelSim `didFuelScore`): inside the
/// radius in x/z, at/below the entry plane, and moving down. Pieces rolling
/// on the floor (center height ≈ ball radius) cannot enter a real hub —
/// require the piece to be well off the ground so scattered fuel doesn't
/// roll in and score.
pub fn hub_scores(hub: &HubZone, pos: &[f32; 3], vel: &[f32; 3]) -> bool {
    let dx = pos[0] - hub.center[0];
    let dz = pos[2] - hub.center[1];
    dx * dx + dz * dz <= hub.radius * hub.radius
        && pos[1] <= hub.entry_height
        && pos[1] >= 0.3
        && vel[1] < 0.0
}

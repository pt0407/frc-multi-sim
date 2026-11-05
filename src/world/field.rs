//! Field arena geometry loaded from a TOML field spec (issue #14, PLAN C1).
//!
//! The spec describes the playable area, wall segments, generic box/cylinder
//! obstacles, and placeholder scoring-zone / climb-point definitions. All
//! colliders are primitives for now; when GLB field meshes land (C1
//! follow-up) the spec's `obstacles`/`walls` entries are the seam where
//! trimesh colliders will be produced instead.
use anyhow::{bail, Context, Result};
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Playable-area half-dimensions (issue #44; provenance: fieldcore
/// SimWorld.ts — 16.541 x 8.069 m field). These replace the historical
/// 16.5 x 8.0 guess; mirrored in `sim.rs`'s legacy world.
pub const FIELD_HALF_X: f32 = 8.2705;
pub const FIELD_HALF_Z: f32 = 4.0345;

/// FuelSim FIELD_COR (sqrt(22/51.5)). Field surfaces use this via the
/// `Min` restitution combine rule: fuel-fuel keeps FUEL_COR while
/// fuel-field bounces use FIELD_COR.
pub const FIELD_COR: f32 = 0.6537;

/// fieldcore field collider material (issue #44): friction 0.82,
/// restitution 0.08 on mesh field colliders.
fn default_ground_friction() -> f32 {
    0.82
}
fn default_ground_restitution() -> f32 {
    0.08
}
fn default_wall_friction() -> f32 {
    0.82
}
fn default_wall_restitution() -> f32 {
    0.08
}

/// Ground slab under the whole arena.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GroundSpec {
    /// cuboid half-extents [x, y, z], meters
    pub half_extents: [f32; 3],
    /// collider center (usually y = -half_extents[1] so the top face is y=0)
    pub pos: [f32; 3],
    #[serde(default = "default_ground_friction")]
    pub friction: f32,
    #[serde(default = "default_ground_restitution")]
    pub restitution: f32,
}

/// A box wall segment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WallSpec {
    /// collider center [x, y, z]
    pub pos: [f32; 3],
    /// cuboid half-extents [x, y, z]
    pub half_extents: [f32; 3],
    #[serde(default = "default_wall_friction")]
    pub friction: f32,
    #[serde(default = "default_wall_restitution")]
    pub restitution: f32,
    /// optional label carried through to viz
    #[serde(default)]
    pub name: Option<String>,
}

/// A generic field obstacle (primitive collider).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum ObstacleSpec {
    Box {
        pos: [f32; 3],
        half_extents: [f32; 3],
        #[serde(default = "default_wall_friction")]
        friction: f32,
        #[serde(default)]
        restitution: f32,
        #[serde(default)]
        name: Option<String>,
    },
    Cylinder {
        pos: [f32; 3],
        radius: f32,
        half_height: f32,
        #[serde(default = "default_wall_friction")]
        friction: f32,
        #[serde(default)]
        restitution: f32,
        #[serde(default)]
        name: Option<String>,
    },
}

/// Placeholder scoring zone: an axis-aligned rectangle on the floor.
/// No game logic reads this yet — it is emitted to viz and hashed into
/// replay headers so the definition is pinned.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScoringZoneSpec {
    pub name: String,
    /// min corner on the floor plane [x, z]
    pub min: [f32; 2],
    /// max corner on the floor plane [x, z]
    pub max: [f32; 2],
}

/// Placeholder climb point: a named world-space anchor.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClimbPointSpec {
    pub name: String,
    pub pos: [f32; 3],
}

/// The complete field layout the world is built from.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FieldSpec {
    /// human-readable name (defaults to file stem on load)
    #[serde(default)]
    pub name: Option<String>,
    /// playable-area half extents; used for viz floor and wall defaults
    pub half_x: f32,
    pub half_z: f32,
    pub ground: GroundSpec,
    /// perimeter/other wall segments (each is its own cuboid collider)
    #[serde(default)]
    pub walls: Vec<WallSpec>,
    /// free-standing obstacles inside the field
    #[serde(default)]
    pub obstacles: Vec<ObstacleSpec>,
    /// placeholder scoring zones (no colliders)
    #[serde(default)]
    pub scoring_zones: Vec<ScoringZoneSpec>,
    /// placeholder climb points (no colliders)
    #[serde(default)]
    pub climb_points: Vec<ClimbPointSpec>,
}

impl Default for FieldSpec {
    /// The historical hard-coded layout: 16.5 x 8 m field, four thin walls.
    fn default() -> Self {
        FieldSpec {
            name: Some("default".to_string()),
            half_x: FIELD_HALF_X,
            half_z: FIELD_HALF_Z,
            ground: GroundSpec {
                half_extents: [20.0, 0.1, 20.0],
                pos: [0.0, -0.1, 0.0],
                friction: 0.82,
                restitution: 0.08,
            },
            walls: vec![
                WallSpec {
                    pos: [0.0, 0.3, -FIELD_HALF_Z],
                    half_extents: [FIELD_HALF_X, 0.6, 0.05],
                    friction: 0.82,
                    restitution: 0.08,
                    name: Some("south".to_string()),
                },
                WallSpec {
                    pos: [0.0, 0.3, FIELD_HALF_Z],
                    half_extents: [FIELD_HALF_X, 0.6, 0.05],
                    friction: 0.82,
                    restitution: 0.08,
                    name: Some("north".to_string()),
                },
                WallSpec {
                    pos: [-FIELD_HALF_X, 0.3, 0.0],
                    half_extents: [0.05, 0.6, FIELD_HALF_Z],
                    friction: 0.82,
                    restitution: 0.08,
                    name: Some("west".to_string()),
                },
                WallSpec {
                    pos: [FIELD_HALF_X, 0.3, 0.0],
                    half_extents: [0.05, 0.6, FIELD_HALF_Z],
                    friction: 0.82,
                    restitution: 0.08,
                    name: Some("east".to_string()),
                },
            ],
            obstacles: Vec::new(),
            scoring_zones: Vec::new(),
            climb_points: Vec::new(),
        }
    }
}

fn check_finite(v: f32, what: &str) -> Result<()> {
    if !v.is_finite() {
        bail!("{what} must be finite (got {v})");
    }
    Ok(())
}

fn check_pos3(pos: &[f32; 3], what: &str) -> Result<()> {
    for (i, v) in pos.iter().enumerate() {
        check_finite(*v, &format!("{what}[{i}]"))?;
    }
    Ok(())
}

fn check_extents(h: &[f32; 3], what: &str) -> Result<()> {
    for (i, v) in h.iter().enumerate() {
        check_finite(*v, &format!("{what}[{i}]"))?;
        if *v <= 0.0 {
            bail!("{what}[{i}] must be > 0 (got {v})");
        }
    }
    Ok(())
}

fn check_material(friction: f32, restitution: f32, what: &str) -> Result<()> {
    check_finite(friction, &format!("{what}.friction"))?;
    check_finite(restitution, &format!("{what}.restitution"))?;
    if friction < 0.0 {
        bail!("{what}.friction must be >= 0 (got {friction})");
    }
    if restitution < 0.0 {
        bail!("{what}.restitution must be >= 0 (got {restitution})");
    }
    Ok(())
}

impl FieldSpec {
    /// Load + validate a field spec from a TOML file. Bad configs fail
    /// loudly here rather than producing a silently mis-shaped world.
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("reading field spec {}", path.display()))?;
        let mut spec: FieldSpec = toml::from_str(&text)
            .with_context(|| format!("parsing field spec {}", path.display()))?;
        if spec.name.is_none() {
            spec.name = path.file_stem().map(|s| s.to_string_lossy().into_owned());
        }
        spec.validate()
            .with_context(|| format!("invalid field spec {}", path.display()))?;
        Ok(spec)
    }

    /// Schema-level sanity checks (TOML parse handles types; this checks
    /// physical plausibility).
    pub fn validate(&self) -> Result<()> {
        check_finite(self.half_x, "half_x")?;
        check_finite(self.half_z, "half_z")?;
        if self.half_x <= 0.0 || self.half_z <= 0.0 {
            bail!("half_x/half_z must be > 0 (got {} x {})", self.half_x, self.half_z);
        }
        check_pos3(&self.ground.pos, "ground.pos")?;
        check_extents(&self.ground.half_extents, "ground.half_extents")?;
        check_material(self.ground.friction, self.ground.restitution, "ground")?;
        for (i, w) in self.walls.iter().enumerate() {
            let what = format!("walls[{i}]");
            check_pos3(&w.pos, &format!("{what}.pos"))?;
            check_extents(&w.half_extents, &format!("{what}.half_extents"))?;
            check_material(w.friction, w.restitution, &what)?;
        }
        for (i, o) in self.obstacles.iter().enumerate() {
            let what = format!("obstacles[{i}]");
            match o {
                ObstacleSpec::Box {
                    pos,
                    half_extents,
                    friction,
                    restitution,
                    ..
                } => {
                    check_pos3(pos, &format!("{what}.pos"))?;
                    check_extents(half_extents, &format!("{what}.half_extents"))?;
                    check_material(*friction, *restitution, &what)?;
                }
                ObstacleSpec::Cylinder {
                    pos,
                    radius,
                    half_height,
                    friction,
                    restitution,
                    ..
                } => {
                    check_pos3(pos, &format!("{what}.pos"))?;
                    check_finite(*radius, &format!("{what}.radius"))?;
                    check_finite(*half_height, &format!("{what}.half_height"))?;
                    if *radius <= 0.0 {
                        bail!("{what}.radius must be > 0 (got {radius})");
                    }
                    if *half_height <= 0.0 {
                        bail!("{what}.half_height must be > 0 (got {half_height})");
                    }
                    check_material(*friction, *restitution, &what)?;
                }
            }
        }
        for (i, z) in self.scoring_zones.iter().enumerate() {
            let what = format!("scoring_zones[{i}] ({})", z.name);
            for v in z.min.iter().chain(z.max.iter()) {
                check_finite(*v, &what)?;
            }
            if !(z.min[0] < z.max[0] && z.min[1] < z.max[1]) {
                bail!("{what}: min must be strictly less than max (got {:?}..{:?})", z.min, z.max);
            }
        }
        for (i, c) in self.climb_points.iter().enumerate() {
            check_pos3(&c.pos, &format!("climb_points[{i}] ({})", c.name))?;
        }
        Ok(())
    }

    /// Number of fixed colliders this spec will produce
    /// (1 ground + walls + obstacles) — handy for tests.
    pub fn collider_count(&self) -> usize {
        1 + self.walls.len() + self.obstacles.len()
    }
}

/// Insert the fixed ground body and every field collider described by
/// `spec`. Returns the fixed body handle all field colliders hang off.
pub fn build_arena(
    spec: &FieldSpec,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
) -> RigidBodyHandle {
    let ground = bodies.insert(RigidBodyBuilder::fixed());
    let g = &spec.ground;
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(g.half_extents[0], g.half_extents[1], g.half_extents[2])
            .translation(vector![g.pos[0], g.pos[1], g.pos[2]])
            .friction(g.friction)
            .restitution(g.restitution)
            .restitution_combine_rule(CoefficientCombineRule::Min)
            .build(),
        ground,
        bodies,
    );
    for w in &spec.walls {
        colliders.insert_with_parent(
            ColliderBuilder::cuboid(w.half_extents[0], w.half_extents[1], w.half_extents[2])
                .translation(vector![w.pos[0], w.pos[1], w.pos[2]])
                .friction(w.friction)
                .restitution(w.restitution)
                .restitution_combine_rule(CoefficientCombineRule::Min)
                .build(),
            ground,
            bodies,
        );
    }
    for o in &spec.obstacles {
        let builder = match o {
            ObstacleSpec::Box {
                pos,
                half_extents,
                friction,
                restitution,
                ..
            } => ColliderBuilder::cuboid(half_extents[0], half_extents[1], half_extents[2])
                .translation(vector![pos[0], pos[1], pos[2]])
                .friction(*friction)
                .restitution(*restitution),
            ObstacleSpec::Cylinder {
                pos,
                radius,
                half_height,
                friction,
                restitution,
                ..
            } => ColliderBuilder::cylinder(*half_height, *radius)
                .translation(vector![pos[0], pos[1], pos[2]])
                .friction(*friction)
                .restitution(*restitution),
        };
        colliders.insert_with_parent(builder.build(), ground, bodies);
    }
    ground
}

//! Robot archetype config (issue #8): a TOML file describes a robot's
//! physical stats and drivetrain profile. Agents use `max_force` for their
//! drive loop; the orchestrator uses `mass`/`half_extents` at spawn.
use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Debug, Clone, Deserialize)]
pub struct Archetype {
    /// human-readable name (optional; defaults to file stem)
    #[serde(default)]
    pub name: Option<String>,
    /// chassis mass, kg
    pub mass: f32,
    /// cuboid half-extents [x, y, z], meters
    pub half_extents: [f32; 3],
    /// max net drive force the drivetrain can produce, N
    pub max_force: f32,
    /// drivetrain profile name — placeholder until profiles are real
    #[serde(default = "default_drivetrain")]
    pub drivetrain: String,
    /// optional robot-relative intake region (issue #3)
    #[serde(default)]
    pub intake: Option<IntakeSpec>,
    /// optional `[sensors]` noise/latency config (issue #7); absent = off
    #[serde(default)]
    pub sensors: crate::world::noise::SensorSpec,
    /// games this archetype is built for (issue #23): game names (the
    /// game TOML `name` key or file stem). Empty = compatible with
    /// everything; otherwise the server warns on a mismatch.
    #[serde(default)]
    pub compatible_games: Vec<String>,
    /// articulated mechanism parts (issue #61): each `[[mech]]` table is
    /// one rigid-body joint (issue #57 spike pattern). Empty = single rigid
    /// body, identical to before.
    #[serde(default, rename = "mech")]
    pub mechs: Vec<MechSpec>,
    /// optional `[model]` visual asset (issue #43): GLB the web client
    /// renders instead of the fallback box. Display-only — physics stays
    /// authoritative on `mass`/`half_extents`.
    #[serde(default)]
    pub model: Option<ModelSpec>,
}

fn default_drivetrain() -> String {
    "generic-holonomic".to_string()
}

/// Visual robot model reference (issue #43): a GLB the web viz loads in
/// place of the primitive chassis box. Purely cosmetic — the physics
/// robot remains the cuboid described by `half_extents`.
///
/// Optional `[model]` table in the archetype TOML:
///
/// ```toml
/// [model]
/// url = "/robot-assets/kitbot-2026.glb"   # served from web/public
/// scale = 1.0                              # uniform display scale
/// offset = [-0.30, 0.0, 0.05]              # chassis-frame offset, m
/// rotation_deg = [90.0, 0.0, 90.0]         # euler X-then-Y-then-Z
/// ```
///
/// `scale`/`offset`/`rotation_deg` orient the asset into the chassis
/// frame (+x forward, +y up, meters). They are serialized into the viz
/// config message verbatim; clients that can't load the URL fall back to
/// the primitive box.
#[derive(Debug, Clone, Deserialize, Serialize, PartialEq)]
pub struct ModelSpec {
    /// GLB URL (typically a `/robot-assets/` path served by the web client)
    pub url: String,
    /// uniform display scale applied to the model (default 1.0)
    #[serde(default = "one")]
    pub scale: f32,
    /// chassis-frame translation applied to the model root, meters
    #[serde(default)]
    pub offset: [f32; 3],
    /// chassis-frame rotation, degrees, applied X then Y then Z
    #[serde(default)]
    pub rotation_deg: [f32; 3],
}

fn one() -> f32 {
    1.0
}

/// Robot-relative active intake region (issue #3, FuelSim `registerIntake`
/// pattern): pieces whose position falls inside this AABB (chassis frame,
/// +x forward) while the agent's `intake_active` flag is set are captured.
/// Optional `[intake]` table in the archetype TOML:
///
/// ```toml
/// [intake]
/// x_min = 0.3        # just in front of the bumper
/// x_max = 0.7
/// z_min = -0.3
/// z_max = 0.3
/// y_min = -0.4       # relative to chassis center
/// y_max = 0.2
/// ```
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq)]
pub struct IntakeSpec {
    pub x_min: f32,
    pub x_max: f32,
    pub z_min: f32,
    pub z_max: f32,
    /// capture region vertical bounds relative to chassis center
    pub y_min: f32,
    pub y_max: f32,
}

/// Articulated mechanism spec (issue #57 spike): ONE revolute arm hung off
/// the chassis — the intake-arm pattern. The arm is a cuboid rigid body
/// pinned to `pivot` (chassis frame, +x forward, +y up) by a revolute
/// joint about the lateral z axis, position-servoed to
/// `ActuatorFrame::mech.target`. The arm extends `half_extents[0]` along
/// its local +x from the pivot at angle 0; positive angle swings it up,
/// negative down toward the floor in front of the bumper.
///
/// ```toml
/// [mech]
/// pivot = [0.40, 0.35, 0.0]        # front-top chassis corner
/// half_extents = [0.45, 0.03, 0.35] # 0.9 m bar, 0.7 m wide
/// mass = 4.0
/// stow_angle = 1.3                 # rad, tipped up over the bumper
/// limits = [-0.8, 1.4]             # rad
/// motor_stiffness = 300.0          # N*m/rad
/// motor_damping = 30.0
/// motor_max_force = 400.0          # N*m
/// ```
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq)]
pub struct MechSpec {
    /// chassis-frame pivot point [x, y, z]
    pub pivot: [f32; 3],
    /// arm cuboid half-extents [x, y, z]; x is along the arm
    pub half_extents: [f32; 3],
    /// arm mass, kg
    pub mass: f32,
    /// initial joint angle (stowed), rad
    pub stow_angle: f32,
    /// revolute joint limits [min, max], rad
    pub limits: [f32; 2],
    /// position-servo stiffness (ForceBased motor model)
    pub motor_stiffness: f32,
    /// position-servo damping
    pub motor_damping: f32,
    /// max motor torque, N*m
    pub motor_max_force: f32,
}

impl Archetype {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("reading archetype {}", path.display()))?;
        let mut a: Archetype = toml::from_str(&text)
            .with_context(|| format!("parsing archetype {}", path.display()))?;
        if a.name.is_none() {
            a.name = path.file_stem().map(|s| s.to_string_lossy().into_owned());
        }
        anyhow::ensure!(a.mass > 0.0, "archetype mass must be > 0");
        anyhow::ensure!(
            a.half_extents.iter().all(|h| *h > 0.0),
            "archetype half_extents must be > 0"
        );
        anyhow::ensure!(a.max_force > 0.0, "archetype max_force must be > 0");
        Ok(a)
    }

    /// Optional `--archetype` flag value -> loaded config or None.
    pub fn load_opt(path: &Option<std::path::PathBuf>) -> Result<Option<Self>> {
        path.as_deref().map(Self::load).transpose()
    }
}

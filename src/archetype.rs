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
    /// kinematic mechanism joints (MoSim-style): 1-DOF states simulated
    /// server-side with motor models and streamed to the viz, which binds
    /// them to named CAD subtrees. `[mech]` stays for physical rigid-body
    /// joints; `[[joint]]` covers everything else (rollers, flywheel,
    /// swerve steer/wheel) without spawning physics bodies.
    #[serde(default, rename = "joint")]
    pub joints: Vec<JointSpec>,
    /// optional `[camera]` vision sensor: detects field AprilTags and gates
    /// the `vision_pose` estimate on real line-of-sight. Absent = legacy
    /// ungated synthetic vision (when `[sensors]` is enabled).
    #[serde(default)]
    pub camera: Option<CameraSpec>,
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

/// Vision camera spec (AprilTag localization). `[camera]` table in the
/// archetype TOML:
///
/// ```toml
/// [camera]
/// pos = [0.30, 0.45, 0.0]   # chassis frame, +x fwd, +y up, meters
/// yaw = 0.0                  # heading offset, radians
/// hfov_deg = 80.0            # horizontal field of view
/// vfov_deg = 60.0            # vertical field of view (defaults to hfov)
/// max_range = 10.0           # meters
/// ```
///
/// A tag is "seen" when it is inside the frustum, within range, on the
/// tag's front face, and roughly at the camera's height band. Detected
/// tag ids stream in `SensorFrame.vision_tags`; `vision_pose` only
/// publishes while at least one tag is visible.
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq)]
pub struct CameraSpec {
    /// camera position in the chassis frame [x, y, z]
    #[serde(default = "cam_pos_default")]
    pub pos: [f32; 3],
    /// heading offset from chassis forward, radians
    #[serde(default)]
    pub yaw: f32,
    /// horizontal field of view, degrees
    #[serde(default = "cam_hfov_default")]
    pub hfov_deg: f32,
    /// vertical field of view, degrees (defaults to hfov_deg)
    #[serde(default)]
    pub vfov_deg: Option<f32>,
    /// max detection range, meters
    #[serde(default = "cam_range_default")]
    pub max_range: f32,
}

fn cam_pos_default() -> [f32; 3] {
    [0.0, 0.45, 0.0]
}
fn cam_hfov_default() -> f32 {
    80.0
}
fn cam_range_default() -> f32 {
    10.0
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
    /// hopper capacity: max total held pieces (carried + preload) the
    /// mechanism can physically hold. `None` = HOPPER_DEFAULT (16).
    #[serde(default)]
    pub capacity: Option<u32>,
}

/// Default hopper capacity when `[intake]` doesn't set one — a realistic
/// fuel count for a mid-size hopper (preload 8 + a few scoops).
pub const HOPPER_DEFAULT: u32 = 16;

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
#[derive(Debug, Clone, Copy, Default, Deserialize, Serialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum JointType {
    /// pivot joint — intake arms, hoppers
    #[default]
    Revolute,
    /// sliding joint along +y — elevators, climbers
    Prismatic,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq)]
pub struct MechSpec {
    /// joint type (issue #61): "revolute" (arm) or "prismatic" (elevator
    /// stage sliding on a vertical axis). Default revolute.
    #[serde(default)]
    pub joint_type: JointType,
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

/// Mechanism joint mode — how the joint's state evolves.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum JointMode {
    /// continuous rotation about an axle; angle accumulates, velocity is
    /// servoed to the driver target (rad/s). Rollers, flywheel, drum.
    Spin,
    /// position servo: angle tracks a commanded target (rad) with
    /// accel/velocity limits. Arms that aren't physical `[mech]`s,
    /// hood angles, elevators (revolute form).
    Position,
    /// per-module steering yaw: one channel per drivetrain module,
    /// servoed toward that module's commanded steer angle.
    Steer,
    /// per-module wheel spin: one channel per module, angle integrated
    /// from chassis speed projected on the steer direction / wheel radius.
    Wheel,
}

/// Who supplies a spin/position joint's target.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum JointDriver {
    /// agent commands via `ActuatorFrame::joints`
    Command,
    /// spins at `max_vel` while the robot's intake is active
    Intake,
    /// spins up while a shot is requested or `shooter_on` is set;
    /// shots are gated on reaching speed (FlywheelBehaviour-style)
    Shooter,
    /// position joint tracking `ShotCommand::turret_yaw` with accel/vel
    /// limits — the physical turret. Launch aim uses the joint's actual
    /// angle, not the raw command, so turret slew lag is real.
    Turret,
}

/// Kinematic mechanism joint spec — the data-driven equivalent of
/// MoSim's roller/flywheel/turret MonoBehaviours. The `mv` regex selects
/// the CAD subtree(s) to animate; `scope` optionally restricts matches
/// to descendants of a named ancestor.
///
/// ```toml
/// [[joint]]
/// name = "shooter"
/// mode = "spin"
/// driver = "shooter"
/// mv = '^(Blades|Main Drum.*)$'
/// scope = "32 - R2 Shooter & Feeder"
/// max_vel = 40.0      # rad/s flywheel speed
/// accel = 60.0        # rad/s^2 spin-up (flywheel spin-up time)
/// ```
#[derive(Debug, Clone, Deserialize, Serialize, PartialEq)]
pub struct JointSpec {
    /// human-readable label (telemetry)
    pub name: String,
    /// regex matched against CAD node names (after "occurrence of"/
    /// " <n>" stripping) to find the subtree(s) to animate
    pub mv: String,
    /// optional ancestor regex — only nodes inside a matching subtree bind
    #[serde(default)]
    pub scope: Option<String>,
    /// joint mode (default "spin")
    #[serde(default = "jm_spin")]
    pub mode: JointMode,
    /// target source for spin/position joints (default "command")
    #[serde(default = "jd_command")]
    pub driver: JointDriver,
    /// rotation axis override in holder/model space; default = auto-detect
    /// from part bounding-box proportions (axle of the cylinder)
    #[serde(default)]
    pub axis: Option<[f32; 3]>,
    /// pivot override in holder/model space; default = bounds centre of
    /// the matched meshes
    #[serde(default)]
    pub pivot: Option<[f32; 3]>,
    /// max joint speed, rad/s (spin/steer) — also the position servo's
    /// max velocity
    #[serde(default = "jv_default")]
    pub max_vel: f32,
    /// accel limit, rad/s^2 — gives mechanisms spin-up/down time like a
    /// real motor instead of instant velocity
    #[serde(default = "ja_default")]
    pub accel: f32,
    /// position-mode joint limits [min, max], rad
    #[serde(default)]
    pub limits: Option<[f32; 2]>,
    /// wheel-mode rolling radius, m — omega = module_speed / radius
    #[serde(default)]
    pub wheel_radius: f32,
    /// roller contact zone anchor. With `on_arm = false` (default) this is
    /// a chassis-frame [x,y,z] offset. With `on_arm = true`, `roller_pos`
    /// is arm-local instead: origin at the `[mech]` pivot, +x along the
    /// arm, and it rotates with the arm angle — for rollers mounted on
    /// articulated mechanisms (intake arms). The MoSim InteractionRoller
    /// effect: spinning rollers drag free pieces toward the intake.
    #[serde(default)]
    pub roller_pos: Option<[f32; 3]>,
    /// mount the roller zone on the robot's `[mech]` arm (revolute only)
    #[serde(default)]
    pub on_arm: bool,
    /// roller interaction radius, m (default 0.3)
    #[serde(default)]
    pub roller_radius: Option<f32>,
    /// shooter exit radius, m: when set on a `driver = "shooter"` joint,
    /// launch speed is physics-derived as vel·radius·1.8 (compression-
    /// wheel exit factor, MoSim FlywheelBehaviour) instead of trusting
    /// the agent's declared launch_velocity magnitude
    #[serde(default)]
    pub launch_radius: Option<f32>,
    /// muzzle position in chassis frame [x,y,z] on a `driver = "shooter"`
    /// joint: pieces launch from this point (yaw-rotated) instead of the
    /// chassis center + `launch_height`. Real shooters sit out on the
    /// mechanism, not at the chassis CoM.
    #[serde(default)]
    pub muzzle_pos: Option<[f32; 3]>,
    /// feed time, s: how long a carried piece takes to travel from the
    /// carry anchor to the muzzle on `driver = "shooter"` joints
    /// (default 0.15). 0 = instant release at the muzzle.
    #[serde(default)]
    pub feed_time: Option<f32>,
}

fn jm_spin() -> JointMode {
    JointMode::Spin
}
fn jd_command() -> JointDriver {
    JointDriver::Command
}
fn jv_default() -> f32 {
    20.0
}
fn ja_default() -> f32 {
    40.0
}

impl Archetype {
    /// Resolve an `--archetype` argument to a TOML path. Bare names like
    /// `2910` or `fast-light` map to `archetypes/<name>.toml` (also
    /// `archetypes/<name>` and `archetypes/<name>.toml` for subdirs like
    /// `scouted/red-track-m12`); anything that already exists passes
    /// through unchanged.
    pub fn resolve(path: &Path) -> std::path::PathBuf {
        if path.exists() {
            return path.to_path_buf();
        }
        let name = path.to_string_lossy();
        for cand in [
            format!("archetypes/{name}.toml"),
            format!("archetypes/{name}"),
        ] {
            let p = std::path::PathBuf::from(cand);
            if p.exists() {
                return p;
            }
        }
        path.to_path_buf()
    }

    pub fn load(path: &Path) -> Result<Self> {
        let resolved = Self::resolve(path);
        let path = resolved.as_path();
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

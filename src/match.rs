//! Match state machine + per-phase scoring (issue #10, PLAN C3).
//!
//! The match clock is a **pure function of the physics tick** — no wall
//! clock, no extra event log records — so replays reproduce phase
//! transitions bit-for-bit as long as the `MatchConfig` is pinned by the
//! replay header's config hash.
//!
//! States: `disabled → auto → teleop → endgame → complete`, durations from
//! match config (`games/<game>.toml`, `[match]` section). `match_state`
//! and `match_time` (seconds remaining in the current phase) are wired
//! into every `SensorFrame`.
use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Physics ticks per second (duplicated from `world::PHYSICS_HZ` to keep
/// this module free of Rapier deps — they must stay equal).
pub const TICK_HZ: f32 = 400.0;

/// Match phase, serialized lowercase on the wire (`"auto"`, `"teleop"`,
/// ...). Additive on `SensorFrame`; defaults to `disabled` for old agents.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MatchPhase {
    /// Robots are not enabled; actuator commands are ignored.
    #[default]
    Disabled,
    /// Autonomous period — robots enabled, scripted agents may switch
    /// behavior on this flag.
    Auto,
    Teleop,
    /// Final window of teleop; climb bonuses are scored here.
    Endgame,
    /// Match over; commands ignored, final scores frozen.
    Complete,
}

impl MatchPhase {
    /// Whether actuator commands are applied during this phase.
    pub fn enabled(self) -> bool {
        matches!(
            self,
            MatchPhase::Auto | MatchPhase::Teleop | MatchPhase::Endgame
        )
    }

    /// Wire/debug name (matches the serde lowercase representation).
    pub fn name(self) -> &'static str {
        match self {
            MatchPhase::Disabled => "disabled",
            MatchPhase::Auto => "auto",
            MatchPhase::Teleop => "teleop",
            MatchPhase::Endgame => "endgame",
            MatchPhase::Complete => "complete",
        }
    }
}

/// Per-phase score breakdown for one alliance (issue #10). `World::scores`
/// stays the running total; this attributes each hub point (and the
/// endgame climb bonus) to the phase it was scored in.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PhaseScore {
    pub auto: u32,
    pub teleop: u32,
    /// Points scored during endgame *plus* the climb bonus awarded at
    /// match completion.
    pub endgame: u32,
}

impl PhaseScore {
    /// Bucket a hub-scored point lands in for `phase`. Points scored while
    /// disabled/complete (piece still falling) are attributed to teleop.
    fn bucket(phase: MatchPhase) -> fn(&mut PhaseScore) -> &mut u32 {
        match phase {
            MatchPhase::Auto => |s| &mut s.auto,
            MatchPhase::Endgame => |s| &mut s.endgame,
            _ => |s| &mut s.teleop,
        }
    }

    pub fn add(&mut self, phase: MatchPhase, points: u32) {
        *Self::bucket(phase)(self) += points;
    }
}

/// Match timing + endgame rules, loaded from a game TOML (`[match]`
/// section) or built from `Default` (REBUILT-flavored FRC timing: 15 s
/// auto, 105 s teleop, 30 s endgame = 2:30 total).
///
/// `disabled_s` defaults to 0 so a bare `World` — tests, replays — starts
/// enabled exactly like before #10 landed.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(default)]
pub struct MatchConfig {
    /// pre-match disabled window (0 = start enabled, legacy behavior)
    pub disabled_s: f32,
    pub auto_s: f32,
    pub teleop_s: f32,
    pub endgame_s: f32,
    /// bonus awarded at the complete transition to each robot within
    /// `climb_radius_m` of a field climb point (placeholder climb rule;
    /// real joint attachment is B3 follow-up)
    pub climb_bonus_points: u32,
    /// horizontal distance to a climb point that counts as "climbed"
    pub climb_radius_m: f32,
    /// REBUILT hub-shift window in teleop (issue: real-match sim): each
    /// alliance's hub is active for `shift_s` seconds at a time, alternating.
    /// The auto winner's hub goes first; 0 = both hubs always active.
    #[serde(default)]
    pub shift_s: f32,
    /// fuel each robot starts holding (game manual 6.3.4, up to 8)
    #[serde(default)]
    pub preload_fuel: u32,
    /// tower-protection foul: an enemy robot bumping a robot on the
    /// tower during endgame awards this many penalty points to the
    /// tower's alliance per instance (MoSim TowerProtection = 15)
    #[serde(default = "tfoul_default")]
    pub tower_foul_points: u32,
    /// per-robot cooldown between tower fouls, seconds (MoSim = 3)
    #[serde(default = "fcool_default")]
    pub foul_cooldown_s: f32,
}

fn tfoul_default() -> u32 {
    15
}
fn fcool_default() -> f32 {
    3.0
}

impl Default for MatchConfig {
    fn default() -> Self {
        MatchConfig {
            disabled_s: 0.0,
            auto_s: 15.0,
            teleop_s: 105.0,
            endgame_s: 30.0,
            climb_bonus_points: 10,
            climb_radius_m: 1.0,
            shift_s: 0.0,
            preload_fuel: 0,
            tower_foul_points: 15,
            foul_cooldown_s: 3.0,
        }
    }
}

/// Full per-game surface (issue #23, PLAN C4): a single `games/*.toml`
/// names the field layout, the game-piece spawn/scoring table, and the
/// `[match]` timing/endgame rules, so the server can be pointed at one
/// file (`--game games/rebuilt.toml`) instead of separate
/// `--field`/`--pieces`/`--game` flags. `field`/`pieces` paths are
/// resolved relative to the game file's directory; the explicit CLI
/// flags still win when both are given.
#[derive(Debug, Default, Deserialize)]
#[serde(default)]
pub struct GameSpec {
    /// human-readable game name (defaults to the file stem). Used for
    /// logging and for matching against archetype `compatible_games`.
    pub name: Option<String>,
    /// field layout TOML (`FieldSpec`), relative to this file's dir
    pub field: Option<std::path::PathBuf>,
    /// game-piece spawn/scoring table TOML (`PieceTable`), relative dir
    pub pieces: Option<std::path::PathBuf>,
    /// match timing + endgame rules
    #[serde(rename = "match")]
    pub match_cfg: MatchConfig,
}

impl GameSpec {
    /// Load a game TOML: name + field/pieces refs + `[match]` section.
    /// Invalid/missing referenced paths fail loudly when later resolved.
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("reading game config {}", path.display()))?;
        let mut g: GameSpec = toml::from_str(&text)
            .with_context(|| format!("parsing game config {}", path.display()))?;
        let dir = path.parent().unwrap_or_else(|| Path::new("."));
        for r in [&mut g.field, &mut g.pieces].into_iter().flatten() {
            if r.is_relative() {
                *r = dir.join(&r);
            }
        }
        g.match_cfg
            .validate()
            .with_context(|| format!("invalid match config {}", path.display()))?;
        Ok(g)
    }

    /// Game name for logs/compat checks: `name` key or the file stem.
    pub fn name(&self, path: &Path) -> String {
        self.name
            .clone()
            .or_else(|| path.file_stem().map(|s| s.to_string_lossy().into_owned()))
            .unwrap_or_else(|| "unnamed".to_string())
    }
}

impl MatchConfig {
    /// Load the `[match]` section of a game TOML. Missing/invalid values
    /// fail loudly (schema-checked, like FieldSpec).
    pub fn load(path: &Path) -> Result<Self> {
        Ok(GameSpec::load(path)?.match_cfg)
    }

    pub fn validate(&self) -> Result<()> {
        for (name, v) in [
            ("disabled_s", self.disabled_s),
            ("auto_s", self.auto_s),
            ("teleop_s", self.teleop_s),
            ("endgame_s", self.endgame_s),
            ("climb_radius_m", self.climb_radius_m),
        ] {
            if !v.is_finite() || v < 0.0 {
                anyhow::bail!("{name} must be >= 0 and finite (got {v})");
            }
        }
        Ok(())
    }

    /// Phase-boundary offsets in seconds, cumulative from t=0.
    pub fn bounds(&self) -> [f32; 4] {
        let d = self.disabled_s;
        [d, d + self.auto_s, d + self.auto_s + self.teleop_s,
         d + self.auto_s + self.teleop_s + self.endgame_s]
    }
}

/// The match clock: a pure function `tick -> (phase, time remaining)`.
/// Deterministic by construction; carry one per `World`.
#[derive(Debug, Clone, Copy)]
pub struct MatchClock {
    pub config: MatchConfig,
}

impl Default for MatchClock {
    fn default() -> Self {
        MatchClock {
            config: MatchConfig::default(),
        }
    }
}

impl MatchClock {
    pub fn new(config: MatchConfig) -> Self {
        MatchClock { config }
    }

    /// Phase at physics `tick` (t = tick / 400 Hz).
    pub fn phase_at(&self, tick: u64) -> MatchPhase {
        let t = tick as f32 / TICK_HZ;
        let b = self.config.bounds();
        if t < b[0] {
            MatchPhase::Disabled
        } else if t < b[1] {
            MatchPhase::Auto
        } else if t < b[2] {
            MatchPhase::Teleop
        } else if t < b[3] {
            MatchPhase::Endgame
        } else {
            MatchPhase::Complete
        }
    }

    /// Is `alliance`'s hub scoring-active at `tick`? (REBUILT hub shifts:
    /// teleop alternates active windows; auto/endgame/complete = both on.)
    pub fn hub_active(&self, alliance: usize, tick: u64) -> bool {
        if self.config.shift_s <= 0.0 {
            return true;
        }
        if self.phase_at(tick) != MatchPhase::Teleop {
            return true;
        }
        let b = self.config.bounds();
        let t = tick as f32 / TICK_HZ;
        let teleop_t = t - b[1]; // seconds into teleop
        let window = (teleop_t / self.config.shift_s) as usize;
        // alliance 0 active in even windows (auto winner picks first window
        // in the real game; deterministic default here)
        (window % 2 == 0) == (alliance == 0)
    }

    /// Seconds remaining in the phase at `tick` (0.0 once complete).
    pub fn time_remaining(&self, tick: u64) -> f32 {
        let t = tick as f32 / TICK_HZ;
        let b = self.config.bounds();
        for bound in b {
            if t < bound {
                return bound - t;
            }
        }
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_starts_enabled_and_progresses() {
        let c = MatchClock::default(); // disabled_s = 0
        assert_eq!(c.phase_at(0), MatchPhase::Auto);
        assert_eq!(c.phase_at(15 * 400 - 1), MatchPhase::Auto);
        assert_eq!(c.phase_at(15 * 400), MatchPhase::Teleop);
        assert_eq!(c.phase_at(120 * 400), MatchPhase::Endgame);
        assert_eq!(c.phase_at(150 * 400), MatchPhase::Complete);
        assert_eq!(c.time_remaining(150 * 400), 0.0);
    }

    #[test]
    fn disabled_window_gates_then_auto() {
        let mut cfg = MatchConfig::default();
        cfg.disabled_s = 1.0;
        cfg.auto_s = 1.0;
        let c = MatchClock::new(cfg);
        assert_eq!(c.phase_at(0), MatchPhase::Disabled);
        assert_eq!(c.phase_at(399), MatchPhase::Disabled);
        assert_eq!(c.phase_at(400), MatchPhase::Auto);
        assert_eq!(c.phase_at(800), MatchPhase::Teleop);
        assert!(!MatchPhase::Disabled.enabled());
        assert!(MatchPhase::Auto.enabled());
        assert!(!MatchPhase::Complete.enabled());
    }

    #[test]
    fn match_file_parses() {
        let cfg: GameSpec = toml::from_str(
            r#"
[match]
auto_s = 20.0
teleop_s = 100.0
endgame_s = 20.0
climb_bonus_points = 15
climb_radius_m = 1.2
"#,
        )
        .unwrap();
        assert_eq!(cfg.match_cfg.auto_s, 20.0);
        assert_eq!(cfg.match_cfg.climb_bonus_points, 15);
        assert_eq!(cfg.match_cfg.disabled_s, 0.0);
    }
}

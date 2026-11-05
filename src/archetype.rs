//! Robot archetype config (issue #8): a TOML file describes a robot's
//! physical stats and drivetrain profile. Agents use `max_force` for their
//! drive loop; the orchestrator uses `mass`/`half_extents` at spawn.
use anyhow::{Context, Result};
use serde::Deserialize;
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
}

fn default_drivetrain() -> String {
    "generic-holonomic".to_string()
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

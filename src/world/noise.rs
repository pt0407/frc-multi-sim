//! Sensor noise + latency model (issue #7 / PLAN B2).
//!
//! Orchestrator-side filtering: `World::sensor_frame` passes the ground-truth
//! frame through a per-robot [`SensorNoise`] model so every agent sees
//! identically-filtered truth. All randomness draws from the world's `SimRng`
//! (PLAN §0 determinism contract) — same seed + inputs → identical frames.
//!
//! Config lives in the archetype TOML under `[sensors]`:
//!
//! ```toml
//! [sensors]
//! preset = "competition"   # "off" | "clean" | "competition"
//! # any field below may be set explicitly to override the preset
//! # gyro_std = 0.005
//! ```
//!
//! `enabled = false` (the default) produces bit-identical output to the
//! pre-noise code and performs zero RNG draws, preserving replay compat.
use crate::rng::SimRng;
use crate::SensorFrame;
use rand_core::RngCore;
use serde::Deserialize;
use std::collections::VecDeque;

/// Agent-tick rate (sensor frames are produced per agent tick).
const SENSOR_HZ: f32 = crate::world::PHYSICS_HZ / crate::world::AGENT_EVERY as f32;
/// Approx. tag distance (m) at which vision noise reaches `vision_std_max`.
const VISION_FAR_M: f32 = 8.0;

/// Resolved, numeric noise config — what lives on `RobotSpec` and travels in
/// replay headers. `Copy` so `RobotSpec` stays `Copy`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorConfig {
    pub enabled: bool,
    /// gyro (angular velocity) Gaussian stddev, rad/s
    pub gyro_std: f32,
    /// pose Gaussian stddev, m
    pub pose_std: f32,
    /// linear velocity Gaussian stddev, m/s
    pub vel_std: f32,
    /// per-boot gyro bias stddev, rad/s
    pub gyro_bias: f32,
    /// per-boot pose bias stddev, m
    pub pose_bias: f32,
    /// per-boot velocity bias stddev, m/s
    pub vel_bias: f32,
    /// vision pipeline latency, seconds
    pub vision_latency_s: f32,
    /// vision estimate update rate, Hz
    pub vision_hz: f32,
    /// per-update dropout probability [0,1]
    pub vision_dropout: f32,
    /// vision pose noise stddev at ~0 m tag distance, m
    pub vision_std_min: f32,
    /// vision pose noise stddev at >=VISION_FAR_M tag distance, m
    pub vision_std_max: f32,
}

impl Default for SensorConfig {
    /// Disabled: byte-identical to pre-noise output.
    fn default() -> Self {
        SensorConfig {
            enabled: false,
            gyro_std: 0.0,
            pose_std: 0.0,
            vel_std: 0.0,
            gyro_bias: 0.0,
            pose_bias: 0.0,
            vel_bias: 0.0,
            vision_latency_s: 0.0,
            vision_hz: 0.0,
            vision_dropout: 0.0,
            vision_std_min: 0.0,
            vision_std_max: 0.0,
        }
    }
}

const CLEAN: SensorConfig = SensorConfig {
    enabled: true,
    gyro_std: 0.0005,
    pose_std: 0.001,
    vel_std: 0.002,
    gyro_bias: 0.0,
    pose_bias: 0.0,
    vel_bias: 0.0,
    vision_latency_s: 0.04,
    vision_hz: 30.0,
    vision_dropout: 0.0,
    vision_std_min: 0.005,
    vision_std_max: 0.03,
};

const COMPETITION: SensorConfig = SensorConfig {
    enabled: true,
    gyro_std: 0.005,
    pose_std: 0.005,
    vel_std: 0.02,
    gyro_bias: 0.002,
    pose_bias: 0.01,
    vel_bias: 0.01,
    vision_latency_s: 0.08,
    vision_hz: 25.0,
    vision_dropout: 0.05,
    vision_std_min: 0.02,
    vision_std_max: 0.15,
};

/// `[sensors]` table as written in archetype TOML: a named preset plus
/// optional per-field overrides. Resolves to a [`SensorConfig`].
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct SensorSpec {
    /// "off" | "clean" | "competition"
    pub preset: Option<String>,
    pub enabled: Option<bool>,
    pub gyro_std: Option<f32>,
    pub pose_std: Option<f32>,
    pub vel_std: Option<f32>,
    pub gyro_bias: Option<f32>,
    pub pose_bias: Option<f32>,
    pub vel_bias: Option<f32>,
    pub vision_latency_s: Option<f32>,
    pub vision_hz: Option<f32>,
    pub vision_dropout: Option<f32>,
    pub vision_std_min: Option<f32>,
    pub vision_std_max: Option<f32>,
}

impl SensorSpec {
    /// Preset base + explicit overrides. No preset and no `enabled` → off.
    pub fn resolved(&self) -> SensorConfig {
        let mut c = match self.preset.as_deref() {
            Some("off") => SensorConfig::default(),
            Some("clean") => CLEAN,
            None | Some("competition") => COMPETITION,
            Some(other) => {
                eprintln!("unknown sensors preset {other:?}; using \"competition\"");
                COMPETITION
            }
        };
        c.enabled = self.enabled.unwrap_or(self.preset.is_some() && c.enabled);
        macro_rules! o {
            ($f:ident) => {
                if let Some(v) = self.$f {
                    c.$f = v;
                }
            };
        }
        o!(gyro_std);
        o!(pose_std);
        o!(vel_std);
        o!(gyro_bias);
        o!(pose_bias);
        o!(vel_bias);
        o!(vision_latency_s);
        o!(vision_hz);
        o!(vision_dropout);
        o!(vision_std_min);
        o!(vision_std_max);
        c
    }
}

/// Standard normal sample via Box-Muller, scaled by `std`. Draws from the
/// world `SimRng` only.
fn gauss(rng: &mut SimRng, std: f32) -> f32 {
    if std <= 0.0 {
        return 0.0;
    }
    let u1 = ((rng.next_u64() >> 11) as f64 + 1.0) / ((1u64 << 53) as f64 + 1.0);
    let u2 = (rng.next_u64() >> 11) as f64 / (1u64 << 53) as f64;
    (std as f64 * (-2.0f64 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()) as f32
}

fn uniform(rng: &mut SimRng) -> f32 {
    (rng.next_u64() >> 40) as f32 / (1u32 << 24) as f32
}

/// Per-robot noise state: per-boot biases plus the vision latency queue.
/// Draws happen exactly once per `sensor_frame` call per robot, so server,
/// replay, and tests stay in lockstep.
pub struct SensorNoise {
    cfg: SensorConfig,
    gyro_bias: [f32; 3],
    pose_bias: [f32; 3],
    vel_bias: [f32; 3],
    /// delayed ground-truth poses: (agent tick, position)
    vision_queue: VecDeque<(u64, [f32; 3])>,
    /// last published vision estimate (survives dropouts)
    vision_est: Option<[f32; 3]>,
    /// next agent tick at which the vision pipeline updates
    vision_due: u64,
}

impl SensorNoise {
    /// Build per-boot state. When `cfg.enabled` is false this draws nothing
    /// from `rng`, keeping the RNG stream identical to pre-noise builds.
    pub fn new(cfg: SensorConfig, rng: &mut SimRng) -> Self {
        let mut b3 = |std: f32| [gauss(rng, std), gauss(rng, std), gauss(rng, std)];
        let (gyro_bias, pose_bias, vel_bias) = if cfg.enabled {
            (b3(cfg.gyro_bias), b3(cfg.pose_bias), b3(cfg.vel_bias))
        } else {
            ([0.0; 3], [0.0; 3], [0.0; 3])
        };
        SensorNoise {
            cfg,
            gyro_bias,
            pose_bias,
            vel_bias,
            vision_queue: VecDeque::new(),
            vision_est: None,
            vision_due: 0,
        }
    }

    pub fn enabled(&self) -> bool {
        self.cfg.enabled
    }

    /// Filter a ground-truth frame in place: Gaussian noise + per-boot bias
    /// on pose/velocity/gyro, plus the delayed/dropout vision pose estimate.
    /// `true_pos` is the unfiltered position used to feed the latency queue.
    pub fn apply(&mut self, rng: &mut SimRng, frame: &mut SensorFrame, true_pos: [f32; 3]) {
        if !self.cfg.enabled {
            return;
        }
        for i in 0..3 {
            frame.position[i] += self.pose_bias[i] + gauss(rng, self.cfg.pose_std);
            frame.linvel[i] += self.vel_bias[i] + gauss(rng, self.cfg.vel_std);
            frame.angvel[i] += self.gyro_bias[i] + gauss(rng, self.cfg.gyro_std);
        }
        self.update_vision(rng, frame.tick, true_pos);
        frame.vision_pose = self.vision_est;
    }

    /// PhotonVision-style pose estimate: a latency-delayed sample of ground
    /// truth, refreshed at ~`vision_hz`, with distance-dependent noise and
    /// seeded dropouts (a dropout keeps the previous estimate, like a real
    /// pipeline publishing nothing that frame).
    fn update_vision(&mut self, rng: &mut SimRng, tick: u64, true_pos: [f32; 3]) {
        let latency_ticks =
            (self.cfg.vision_latency_s * SENSOR_HZ).round().max(0.0) as u64;
        self.vision_queue.push_back((tick, true_pos));
        // retain only samples that could still be the delayed observation
        while self
            .vision_queue
            .front()
            .map(|&(t, _)| t + latency_ticks < tick)
            .unwrap_or(false)
        {
            self.vision_queue.pop_front();
        }
        if self.cfg.vision_hz <= 0.0 {
            return;
        }
        if tick < self.vision_due {
            return;
        }
        // schedule the next update (~vision_hz on the agent tick grid)
        let period = (SENSOR_HZ / self.cfg.vision_hz).round().max(1.0) as u64;
        self.vision_due = tick + period;

        if uniform(rng) < self.cfg.vision_dropout {
            return; // dropped frame: keep stale estimate
        }
        // delayed observation = newest sample at or beyond the latency horizon
        let obs = self
            .vision_queue
            .iter()
            .rev()
            .find(|&&(t, _)| t + latency_ticks <= tick)
            .map(|&(_, p)| p)
            .unwrap_or(true_pos);
        // distance-dependent noise: proxy tag distance by range from field
        // center (tags live around the perimeter)
        let dist = (obs[0] * obs[0] + obs[2] * obs[2]).sqrt();
        let t = (dist / VISION_FAR_M).clamp(0.0, 1.0);
        let std = self.cfg.vision_std_min + (self.cfg.vision_std_max - self.cfg.vision_std_min) * t;
        self.vision_est = Some([
            obs[0] + gauss(rng, std),
            obs[1] + gauss(rng, std),
            obs[2] + gauss(rng, std),
        ]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_resolve() {
        let off = SensorSpec {
            preset: Some("off".into()),
            ..Default::default()
        };
        assert!(!off.resolved().enabled);
        let comp = SensorSpec {
            preset: Some("competition".into()),
            gyro_std: Some(0.1),
            ..Default::default()
        };
        let c = comp.resolved();
        assert!(c.enabled);
        assert_eq!(c.gyro_std, 0.1);
        assert_eq!(c.vision_hz, 25.0);
        assert!(!SensorSpec::default().resolved().enabled);
    }
}

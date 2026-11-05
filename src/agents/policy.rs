//! In-process RL policy agent (issue #12 / PLAN D1).
//!
//! A `PolicyAgent` sits behind the same `SensorFrame -> ActuatorFrame`
//! interface as every other agent, but runs inside the orchestrator so it
//! can also see the `ObsWorld` entity snapshot (other chassis + pieces —
//! the global-state view, like Team1690 StrategyEngine's `GlobalState`,
//! which is not part of the WS `SensorFrame` protocol). The orchestrator
//! drives one [`PolicyAgent::tick`] per agent tick for each robot claimed
//! via `--policy`.
//!
//! Policy backends implement [`Policy`]. `LinearPolicy` is the shipped
//! stub: a deterministic linear layer (weights drawn from a seeded LCG —
//! no `rand`, no I/O) that clips its output into legal actuator ranges.
//! An ONNX backend via `ort` drops in behind the same trait without
//! touching obs building, logging, or the server wiring.
//!
//! Every tick appends `(obs, action, reward, done)` to an
//! `experiences.jsonl` via [`ExperienceLogger`] — the trajectory format a
//! PPO learner consumes offline.
use crate::obs::{ObsBuilder, ObsConfig, ObsWorld, FORCE_MAX, TORQUE_MAX};
use crate::r#match::MatchPhase;
use crate::{ActuatorFrame, SensorFrame};
use serde::Serialize;
use std::io::Write;
use std::path::Path;

/// Action dimension of the shipped policies: [fx, fz, torque].
pub const ACTION_DIM: usize = 3;

/// Policy backend: obs in, actuator frame out. Implementations must be
/// deterministic given the same obs (PLAN §0 — no unseeded randomness,
/// no wall-clock) so rollouts replay exactly.
pub trait Policy: Send {
    fn act(&mut self, obs: &[f32]) -> ActuatorFrame;
}

/// Deterministic linear-policy stub (3 outputs: fx, fz, torque).
/// Weights come from a fixed-seed LCG hashed with the obs dim, so the
/// same build always produces the same net — it exists to exercise the
/// obs/act/log pipeline end-to-end until a trained ONNX policy lands.
/// `zeros` gives the null policy (all-zero output) for tests.
pub struct LinearPolicy {
    /// weights[output][input]
    weights: Vec<Vec<f32>>,
    bias: [f32; ACTION_DIM],
    /// output scale (N, N·m)
    max_force: f32,
    max_torque: f32,
}

impl LinearPolicy {
    /// Deterministic pseudo-random weights in [-w, w]. `seed` varies the
    /// net per robot so policy fleets don't move identically.
    pub fn seeded(obs_dim: usize, seed: u64, w: f32, max_force: f32, max_torque: f32) -> Self {
        let mut state = seed ^ 0x9E37_79B9_7F4A_7C15;
        let mut next = || {
            // splitmix64 — deterministic, no external deps
            state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = state;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^= z >> 31;
            // -> uniform [-1, 1]
            (z as f64 / u64::MAX as f64 * 2.0 - 1.0) as f32
        };
        let weights = (0..ACTION_DIM)
            .map(|_| (0..obs_dim).map(|_| next() * w).collect())
            .collect();
        let bias = [next() * 0.1, next() * 0.1, next() * 0.1];
        LinearPolicy {
            weights,
            bias,
            max_force,
            max_torque,
        }
    }

    /// Null policy: always emits a zeroed frame (test-friendly default).
    pub fn zeros(obs_dim: usize) -> Self {
        LinearPolicy {
            weights: vec![vec![0.0; obs_dim]; ACTION_DIM],
            bias: [0.0; ACTION_DIM],
            max_force: FORCE_MAX,
            max_torque: TORQUE_MAX,
        }
    }
}

impl Policy for LinearPolicy {
    fn act(&mut self, obs: &[f32]) -> ActuatorFrame {
        let mut out = [0.0f32; ACTION_DIM];
        for (o, row) in self.weights.iter().enumerate() {
            // tanh squash keeps any weight set inside legal ranges
            let acc: f32 = row
                .iter()
                .zip(obs.iter())
                .map(|(w, x)| w * x)
                .sum::<f32>()
                + self.bias[o];
            out[o] = acc.tanh();
        }
        ActuatorFrame {
            force: [out[0] * self.max_force, out[1] * self.max_force],
            torque: out[2] * self.max_torque,
            modules: None,
            intake_active: out[0].abs() < 0.5, // stub heuristic: intake while not driving hard
            shoot: None,
            mech: None,
        }
    }
}

/// One experience row — the JSONL schema consumed by the offline learner.
#[derive(Debug, Clone, Serialize)]
pub struct Experience {
    pub robot: usize,
    pub tick: u64,
    pub obs: Vec<f32>,
    /// normalized action as emitted by the policy [fx/F, fz/F, torque/T]
    pub action: [f32; 3],
    /// reward for the transition *into* this tick (score delta +
    /// pickup bonus since the previous tick)
    pub reward: f32,
    pub done: bool,
}

/// Appends `Experience` rows to a `.jsonl` file, one line each.
/// Opened lazily per file; buffering is a `BufWriter` so hot loops don't
/// syscall per row.
pub struct ExperienceLogger {
    w: std::io::BufWriter<std::fs::File>,
}

impl ExperienceLogger {
    pub fn create(path: &Path) -> std::io::Result<Self> {
        Ok(ExperienceLogger {
            w: std::io::BufWriter::new(std::fs::File::create(path)?),
        })
    }

    pub fn log(&mut self, e: &Experience) -> std::io::Result<()> {
        serde_json::to_writer(&mut self.w, e)?;
        self.w.write_all(b"\n")?;
        Ok(())
    }

    pub fn flush(&mut self) -> std::io::Result<()> {
        self.w.flush()
    }
}

/// Per-robot in-process agent: obs builder + policy + reward accounting.
/// One instance per `--policy` robot slot.
pub struct PolicyAgent {
    pub robot: usize,
    builder: ObsBuilder,
    policy: Box<dyn Policy>,
    /// previous-tick bookkeeping for reward shaping
    last_score: u32,
    last_held: u32,
    /// normalized action emitted last tick (for experience logging)
    last_action: [f32; 3],
}

impl PolicyAgent {
    /// Linear-stub agent; `seed` decorrelates fleets.
    pub fn linear(robot: usize, cfg: ObsConfig, seed: u64) -> Self {
        let dim = ObsBuilder::new(cfg.clone()).dim();
        let policy = LinearPolicy::seeded(dim, seed, 0.05, FORCE_MAX, TORQUE_MAX);
        Self::with_policy(robot, cfg, Box::new(policy))
    }

    pub fn with_policy(robot: usize, cfg: ObsConfig, policy: Box<dyn Policy>) -> Self {
        PolicyAgent {
            robot,
            builder: ObsBuilder::new(cfg),
            policy,
            last_score: 0,
            last_held: 0,
            last_action: [0.0; ACTION_DIM],
        }
    }

    pub fn obs_dim(&self) -> usize {
        self.builder.dim()
    }

    /// One agent tick: build obs from the filtered `SensorFrame` +
    /// shared `ObsWorld`, run the policy, and produce the experience row
    /// for the transition since the previous tick.
    ///
    /// Reward (v1 shaping): +1 per alliance point gained, +0.5 per piece
    /// picked up — sparse, deterministic, derived from filtered state.
    pub fn tick(
        &mut self,
        frame: &SensorFrame,
        world: &ObsWorld,
    ) -> (ActuatorFrame, Experience) {
        let obs = self.builder.build(self.robot, frame, world);
        let cmd = self.policy.act(&obs);
        self.builder.record_action(&cmd);
        let action = [
            cmd.force[0] / FORCE_MAX,
            cmd.force[1] / FORCE_MAX,
            cmd.torque / TORQUE_MAX,
        ];
        let reward = (frame.score.saturating_sub(self.last_score)) as f32
            + 0.5 * (frame.pieces_held.saturating_sub(self.last_held)) as f32;
        let done = frame.match_state == MatchPhase::Complete;
        self.last_score = frame.score;
        self.last_held = frame.pieces_held;
        self.last_action = action;
        let exp = Experience {
            robot: self.robot,
            tick: frame.tick,
            obs,
            action,
            reward,
            done,
        };
        (cmd, exp)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::obs::ObsWorld;

    fn frame() -> SensorFrame {
        SensorFrame {
            tick: 0,
            position: [0.0, 0.45, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            linvel: [0.0; 3],
            angvel: [0.0; 3],
            contact_impulse: 0.0,
            pieces_held: 0,
            score: 0,
            match_state: MatchPhase::Teleop,
            match_time: 100.0,
            vision_pose: None,
        }
    }

    fn world() -> ObsWorld {
        ObsWorld {
            robots: vec![([0.0, 0.45, 0.0], [0.0; 3])],
            pieces: vec![[1.0, 0.1, 0.0]],
            field_half: [8.2705, 4.0345],
            scores: [0, 0],
        }
    }

    #[test]
    fn linear_policy_is_deterministic_and_bounded() {
        let mut a = LinearPolicy::seeded(64, 7, 0.05, FORCE_MAX, TORQUE_MAX);
        let mut b = LinearPolicy::seeded(64, 7, 0.05, FORCE_MAX, TORQUE_MAX);
        let obs: Vec<f32> = (0..64).map(|i| i as f32 / 64.0).collect();
        let c1 = a.act(&obs);
        let c2 = b.act(&obs);
        assert_eq!(c1, c2);
        assert!(c1.force.iter().all(|f| f.abs() <= FORCE_MAX));
        assert!(c1.torque.abs() <= TORQUE_MAX);
    }

    #[test]
    fn policy_agent_produces_valid_actuator_and_experience() {
        let mut agent = PolicyAgent::with_policy(
            0,
            ObsConfig::default(),
            Box::new(LinearPolicy::zeros(1)),
        );
        let (cmd, exp) = agent.tick(&frame(), &world());
        assert!(cmd.force.iter().all(|f| f.is_finite()));
        assert!(cmd.torque.is_finite());
        assert_eq!(exp.obs.len(), agent.obs_dim());
        assert_eq!(exp.reward, 0.0);
        assert!(!exp.done);

        // score delta -> reward; done at match end
        let mut f2 = frame();
        f2.score = 2;
        f2.pieces_held = 1;
        f2.match_state = MatchPhase::Complete;
        let (_c, exp2) = agent.tick(&f2, &world());
        assert!((exp2.reward - 2.5).abs() < 1e-6);
        assert!(exp2.done);
    }

    #[test]
    fn experience_serializes_to_jsonl() {
        let dir = std::env::temp_dir().join(format!("fms-exp-{}", std::process::id()));
        let _ = std::fs::remove_file(&dir);
        let mut log = ExperienceLogger::create(&dir).unwrap();
        log.log(&Experience {
            robot: 0,
            tick: 8,
            obs: vec![0.1, -0.2],
            action: [0.0, 0.0, 0.0],
            reward: 1.0,
            done: false,
        })
        .unwrap();
        log.flush().unwrap();
        drop(log);
        let text = std::fs::read_to_string(&dir).unwrap();
        let row: serde_json::Value = serde_json::from_str(text.trim()).unwrap();
        assert_eq!(row["robot"], 0);
        assert_eq!(row["reward"], 1.0);
        let _ = std::fs::remove_file(&dir);
    }
}

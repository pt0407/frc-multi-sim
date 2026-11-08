//! RL observation vector builder (issue #12 / PLAN D1).
//!
//! `ObsBuilder` turns a robot's filtered `SensorFrame` plus the
//! orchestrator's `ObsWorld` entity snapshot (other chassis + game pieces —
//! the same global state the spectator stream and Team1690's
//! StrategyEngine `GlobalState` carry) into a flat, ~[-1, 1] normalized
//! feature vector suitable for feeding a policy network.
//!
//! Layout (all scalars tanh-clamped to [-1, 1]):
//!
//! ```text
//! [ own: pos.x/hx, pos.z/hz, sin(yaw), cos(yaw),
//!   vel.x/VMAX, vel.z/VMAX, angvel.y/WMAX, contact/IMP_MAX ]            (8)
//! [ k_pieces  x [present, rel.x/L, rel.z/L]  (chassis frame, sorted) ]  (3k)
//! [ k_enemies x [present, rel.x/L, rel.z/L] ]                          (3k)
//! [ k_allies  x [present, rel.x/L, rel.z/L] ]                          (3k)
//! [ n_rays proximity ranges / RAY_MAX, chassis-frame, front = ray 0 ]
//! [ held/HELD_MAX, score_delta/SCORE_MAX, match_time/T_MAX, phase/4 ]  (4)
//! [ last_n_actions x [fx/FMAX, fz/FMAX, torque/TMAX] ]                 (3n)
//! ```
//!
//! Determinism contract (PLAN §0): `build` is a pure function of its
//! inputs plus the builder's own last-action ring buffer — no RNG, no
//! wall-clock. Own-robot scalars come from the *filtered* `SensorFrame`
//! (issue #7 noise layer), never from ground truth; other entities come
//! from the same snapshot the viz broadcasts. Same seed + same inputs →
//! bit-identical obs streams.
use crate::drivetrain::{rot2, yaw_from_quat};
use crate::{ActuatorFrame, SensorFrame};
use std::collections::VecDeque;

/// Normalization constants (orders of magnitude from the field and
/// archetype stats; the point is scale, not exactness).
pub const VEL_MAX: f32 = 8.0; // m/s
pub const ANGVEL_MAX: f32 = 10.0; // rad/s
pub const IMPULSE_MAX: f32 = 50.0; // N·s bumper hit
pub const FORCE_MAX: f32 = 1200.0; // N, ~= archetype max_force
pub const TORQUE_MAX: f32 = 200.0; // N·m
pub const HELD_MAX: f32 = 8.0;
pub const SCORE_DELTA_MAX: f32 = 32.0;
pub const MATCH_TIME_MAX: f32 = 160.0; // s, ~ REBUILT match length
pub const PIECE_RAY_RADIUS: f32 = 0.11; // fuel ball radius (pieces.rs)
pub const ROBOT_RAY_RADIUS: f32 = 0.5; // chassis circumscribed ~0.57, trimmed

/// World-side entity context for one build call: the global-state view
/// (all chassis + live pieces). Built by `World::obs_world` once per
/// agent tick and shared by every policy agent.
#[derive(Debug, Clone)]
pub struct ObsWorld {
    /// per-robot (position, linvel) ground poses; index = robot id.
    /// Own robot is included (the builder skips it for enemies/allies).
    pub robots: Vec<([f32; 3], [f32; 3])>,
    /// live piece positions
    pub pieces: Vec<[f32; 3]>,
    /// playable half-extents [x, z]
    pub field_half: [f32; 2],
    /// per-alliance scores [0]=even ids, [1]=odd ids
    pub scores: [u32; 2],
}

#[derive(Debug, Clone)]
pub struct ObsConfig {
    /// nearest-K game pieces to encode
    pub k_pieces: usize,
    /// nearest-K enemy robots
    pub k_enemies: usize,
    /// nearest-K allied robots (excludes self)
    pub k_allies: usize,
    /// proximity ray count, evenly spaced around the chassis, ray 0 = +x fwd
    pub n_rays: usize,
    /// ray range cap, m
    pub ray_max: f32,
    /// ring buffer of previous actions appended to the obs
    pub last_actions: usize,
}

impl Default for ObsConfig {
    fn default() -> Self {
        ObsConfig {
            k_pieces: 4,
            k_enemies: 2,
            k_allies: 2,
            n_rays: 16,
            ray_max: 6.0,
            last_actions: 2,
        }
    }
}

/// Per-robot obs builder. Holds the last-action ring buffer, so one
/// instance per policy robot (builders are not shareable).
pub struct ObsBuilder {
    pub cfg: ObsConfig,
    history: VecDeque<[f32; 3]>,
}

impl ObsBuilder {
    pub fn new(cfg: ObsConfig) -> Self {
        ObsBuilder {
            cfg,
            history: VecDeque::new(),
        }
    }

    /// Length of the vector `build` produces.
    pub fn dim(&self) -> usize {
        8 + 3 * self.cfg.k_pieces
            + 3 * self.cfg.k_enemies
            + 3 * self.cfg.k_allies
            + self.cfg.n_rays
            + 4
            + 3 * self.cfg.last_actions
    }

    /// Build the normalized obs vector. `frame` is this robot's filtered
    /// `SensorFrame`; `world` is the shared entity snapshot. `score_delta`
    /// is computed as own-alliance minus opposing-alliance score.
    pub fn build(&mut self, robot: usize, frame: &SensorFrame, world: &ObsWorld) -> Vec<f32> {
        let cfg = self.cfg.clone();
        let mut v = Vec::with_capacity(self.dim());
        let hx = world.field_half[0].max(1e-3);
        let hz = world.field_half[1].max(1e-3);
        let diag = (hx * hx + hz * hz).sqrt();
        let yaw = yaw_from_quat(frame.rotation);

        // --- own state (filtered sensor truth only) ---
        push(&mut v, frame.position[0] / hx);
        push(&mut v, frame.position[2] / hz);
        push(&mut v, yaw.sin());
        push(&mut v, yaw.cos());
        push(&mut v, frame.linvel[0] / VEL_MAX);
        push(&mut v, frame.linvel[2] / VEL_MAX);
        push(&mut v, frame.angvel[1] / ANGVEL_MAX);
        push(&mut v, frame.contact_impulse / IMPULSE_MAX);

        // --- nearest-K entities, chassis-frame relative, sorted by range ---
        let pos = [frame.position[0], frame.position[2]];
        let mut pieces: Vec<(f32, [f32; 2])> = world
            .pieces
            .iter()
            .map(|p| rel(yaw, pos, [p[0], p[2]]))
            .collect();
        pieces.sort_by(|a, b| a.0.total_cmp(&b.0));
        emit_entities(&mut v, &pieces, cfg.k_pieces, diag);

        let my_alliance = robot % 2;
        let mut enemies: Vec<(f32, [f32; 2])> = Vec::new();
        let mut allies: Vec<(f32, [f32; 2])> = Vec::new();
        for (i, (p, _v)) in world.robots.iter().enumerate() {
            if i == robot {
                continue;
            }
            let e = rel(yaw, pos, [p[0], p[2]]);
            if i % 2 == my_alliance {
                allies.push(e);
            } else {
                enemies.push(e);
            }
        }
        enemies.sort_by(|a, b| a.0.total_cmp(&b.0));
        allies.sort_by(|a, b| a.0.total_cmp(&b.0));
        emit_entities(&mut v, &enemies, cfg.k_enemies, diag);
        emit_entities(&mut v, &allies, cfg.k_allies, diag);

        // --- proximity rays (chassis frame, ray 0 straight ahead) ---
        for ray in 0..cfg.n_rays {
            let ang = yaw + std::f32::consts::TAU * ray as f32 / cfg.n_rays as f32;
            let dir = [ang.cos(), ang.sin()];
            let d = cast_ray(pos, dir, world, robot, cfg.ray_max);
            push(&mut v, d / cfg.ray_max);
        }

        // --- match / inventory context ---
        let opp = (my_alliance + 1) % world.scores.len().max(1);
        let delta = world.scores[my_alliance % world.scores.len()] as f32
            - world.scores[opp] as f32;
        push(&mut v, frame.pieces_held as f32 / HELD_MAX);
        push(&mut v, delta / SCORE_DELTA_MAX);
        push(&mut v, frame.match_time / MATCH_TIME_MAX);
        push(&mut v, phase_index(frame.match_state) / 4.0);

        // --- last-N actions ---
        for h in &self.history {
            push(&mut v, h[0]);
            push(&mut v, h[1]);
            push(&mut v, h[2]);
        }
        for _ in self.history.len()..cfg.last_actions {
            v.extend_from_slice(&[0.0, 0.0, 0.0]);
        }
        v
    }

    /// Record the action actually sent so it shows up in the next obs'
    /// last-action block (normalized by the same constants).
    pub fn record_action(&mut self, cmd: &ActuatorFrame) {
        self.history.push_front([
            cmd.force[0] / FORCE_MAX,
            cmd.force[1] / FORCE_MAX,
            cmd.torque / TORQUE_MAX,
        ]);
        while self.history.len() > self.cfg.last_actions {
            self.history.pop_back();
        }
    }
}

/// clamp-push helper: every emitted scalar is bounded to [-1, 1]
fn push(v: &mut Vec<f32>, x: f32) {
    v.push(if x.is_finite() { x.clamp(-1.0, 1.0) } else { 0.0 });
}

/// entity rel-offset in chassis frame (+x fwd, +z left), returns (range, rel)
fn rel(yaw: f32, pos: [f32; 2], p: [f32; 2]) -> (f32, [f32; 2]) {
    let d = [p[0] - pos[0], p[1] - pos[1]];
    let r = rot2(d, -yaw); // world -> chassis frame
    (d[0].hypot(d[1]), r)
}

fn emit_entities(v: &mut Vec<f32>, ents: &[(f32, [f32; 2])], k: usize, norm: f32) {
    for i in 0..k {
        if let Some((_, r)) = ents.get(i) {
            push(v, 1.0);
            push(v, r[0] / norm);
            push(v, r[1] / norm);
        } else {
            v.extend_from_slice(&[0.0, 0.0, 0.0]);
        }
    }
}

/// Analytic ray cast: min distance from `pos` along `dir` to the field
/// perimeter AABB, other chassis (circles), and pieces (circles), capped
/// at `cfg.ray_max`. Deterministic, no physics queries — the ray model is
/// a sensor abstraction like StrategyEngine's proximity rays.
fn cast_ray(
    pos: [f32; 2],
    dir: [f32; 2],
    world: &ObsWorld,
    self_id: usize,
    ray_max: f32,
) -> f32 {
    let mut best = f32::MAX;
    // perimeter walls: field AABB [-hx,hx] x [-hz,hz]
    for axis in 0..2 {
        let half = world.field_half[axis];
        if dir[axis].abs() > 1e-6 {
            for sign in [-1.0f32, 1.0] {
                let t = (sign * half - pos[axis]) / dir[axis];
                if t > 0.0 {
                    // verify the hit lands inside the other axis' span
                    let o = pos[1 - axis] + t * dir[1 - axis];
                    if o.abs() <= half + 0.1 {
                        best = best.min(t);
                    }
                }
            }
        }
    }
    // chassis + pieces as circles
    let mut circles: Vec<([f32; 2], f32)> = world
        .robots
        .iter()
        .enumerate()
        .filter(|(i, _)| *i != self_id)
        .map(|(_, (p, _))| ([p[0], p[2]], ROBOT_RAY_RADIUS))
        .collect();
    circles.extend(world.pieces.iter().map(|p| ([p[0], p[2]], PIECE_RAY_RADIUS)));
    for (c, r) in circles {
        // solve |pos + t*dir - c| = r for the smallest positive t
        let oc = [pos[0] - c[0], pos[1] - c[1]];
        let b = oc[0] * dir[0] + oc[1] * dir[1];
        let cc = oc[0] * oc[0] + oc[1] * oc[1] - r * r;
        let disc = b * b - cc;
        if disc >= 0.0 {
            let t = -b - disc.sqrt();
            if t > 0.0 {
                best = best.min(t);
            }
        }
    }
    best.min(ray_max)
}

fn phase_index(p: crate::r#match::MatchPhase) -> f32 {
    use crate::r#match::MatchPhase::*;
    match p {
        Disabled => 0.0,
        Auto => 1.0,
        Teleop => 2.0,
        Endgame => 3.0,
        Complete => 4.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::r#match::MatchPhase;

    fn frame() -> SensorFrame {
        SensorFrame {
            tick: 0,
            position: [1.0, 0.45, -2.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            linvel: [2.0, 0.0, -1.0],
            angvel: [0.0, 1.0, 0.0],
            contact_impulse: 10.0,
            pieces_held: 3,
            score: 5,
            match_state: MatchPhase::Teleop,
            match_time: 90.0,
            vision_pose: None,
            vision_tags: Vec::new(),
            mech_angle: None,
            hub_active: true,
            joints: Vec::new(),
        }
    }

    fn world() -> ObsWorld {
        ObsWorld {
            robots: vec![
                ([1.0, 0.45, -2.0], [0.0; 3]),   // self (id 0, alliance 0)
                ([3.0, 0.45, 0.0], [0.0; 3]),    // id 1 enemy
                ([-1.0, 0.45, -2.0], [0.0; 3]),  // id 2 ally
                ([4.0, 0.45, 1.0], [0.0; 3]),    // id 3 enemy
            ],
            pieces: vec![[1.5, 0.1, -2.0], [0.0, 0.1, 0.0], [7.0, 0.1, 3.0]],
            field_half: [8.2705, 4.0345],
            scores: [5, 3],
        }
    }

    #[test]
    fn obs_dim_matches_vector() {
        let cfg = ObsConfig::default();
        let mut b = ObsBuilder::new(cfg.clone());
        let obs = b.build(0, &frame(), &world());
        assert_eq!(obs.len(), b.dim());
        assert_eq!(obs.len(), 8 + 3 * 4 + 3 * 2 + 3 * 2 + 16 + 4 + 3 * 2);
    }

    #[test]
    fn obs_bounded_and_finite() {
        let mut b = ObsBuilder::new(ObsConfig::default());
        let mut w = world();
        // adversarial: huge velocities/scores must still clamp
        w.robots[1].0 = [100.0, 0.45, 100.0];
        w.scores = [999, 0];
        let mut f = frame();
        f.linvel = [50.0, 0.0, -50.0];
        let obs = b.build(0, &f, &w);
        assert!(obs.iter().all(|x| x.is_finite() && x.abs() <= 1.0));
    }

    #[test]
    fn obs_deterministic_same_inputs() {
        let mut a = ObsBuilder::new(ObsConfig::default());
        let mut b = ObsBuilder::new(ObsConfig::default());
        let f = frame();
        let w = world();
        for _ in 0..5 {
            assert_eq!(a.build(0, &f, &w), b.build(0, &f, &w));
        }
    }

    #[test]
    fn nearest_k_sorted_and_chassis_frame() {
        let mut b = ObsBuilder::new(ObsConfig {
            k_pieces: 2,
            ..Default::default()
        });
        let obs = b.build(0, &frame(), &world());
        // first piece block starts at index 8: present, rel_x/L, rel_z/L.
        // piece at [1.5,-2.0] is +0.5 fwd of self at [1.0,-2.0], yaw=0.
        assert_eq!(obs[8], 1.0);
        let diag = (8.2705f32.powi(2) + 4.0345f32.powi(2)).sqrt();
        assert!((obs[9] - 0.5 / diag).abs() < 1e-6);
        assert!(obs[10].abs() < 1e-6);
    }

    #[test]
    fn rays_hit_walls_and_entities() {
        let cfg = ObsConfig { n_rays: 8, ..Default::default() };
        let mut b = ObsBuilder::new(cfg.clone());
        // self at [1,-2] yaw=0: ray 0 (+x) hits the piece at x=1.5
        // (circle r=0.11 -> d=0.39) before the enemy at x=3 or the wall.
        let obs = b.build(0, &frame(), &world());
        let ray0 = obs[8 + 3 * 4 + 3 * 2 + 3 * 2]; // first ray after entities
        assert!((ray0 - 0.39 / cfg.ray_max).abs() < 1e-5, "ray0={ray0}");
        // ray 4 (+z after half turn? rays go around; ray 2 = +z dir) —
        // just check all rays are in (0,1]
        let base = 8 + 3 * 4 + 3 * 2 + 3 * 2;
        for i in 0..8 {
            assert!(obs[base + i] > 0.0 && obs[base + i] <= 1.0);
        }
    }

    #[test]
    fn last_actions_recorded() {
        let mut b = ObsBuilder::new(ObsConfig::default());
        let f = frame();
        let w = world();
        b.build(0, &f, &w);
        b.record_action(&ActuatorFrame {
            force: [600.0, -1200.0],
            torque: 100.0,
            modules: None,
            intake_active: false,
            shoot: None,
            mech: None,
            joints: None,
            shooter_on: false,
        });
        let obs = b.build(0, &f, &w);
        let tail = &obs[obs.len() - 6..];
        assert!((tail[0] - 0.5).abs() < 1e-6); // 600/1200
        assert!((tail[1] - -1.0).abs() < 1e-6);
        assert!((tail[2] - 0.5).abs() < 1e-6);
    }
}

//! Seeded fault injection on agent links (issue #32): per-robot-slot
//! packet loss, added latency (frame queueing), jitter (variable delay),
//! and periodic disconnect bursts, applied to both the inbound actuator
//! path and the outbound sensor path.
//!
//! Determinism contract (PLAN §0): every injector owns a `SimRng` seeded
//! from `world_seed ^ mix(link_index)`, and draws in a fixed order (one
//! loss roll, then one jitter roll iff the frame survived and jitter is
//! configured). Same seed + same call sequence => same fault pattern, so
//! fault-injected runs are replay-compatible.
//!
//! Config (`--faults file.toml`):
//! ```toml
//! [faults]
//! [[faults.link]]
//! robot = 0
//! loss = 0.25                    # drop probability for packets either way
//! latency_ticks = 2              # outbound sensor frames held this many ticks
//! jitter_ticks = 3               # + uniform(0..=jitter_ticks) extra hold
//! disconnect_period_ticks = 500  # every N ticks...
//! disconnect_ticks = 25          # ...drop everything for this long
//! ```
//! `[[link]]` tables at the top level are also accepted.

use crate::rng::SimRng;
use crate::SensorFrame;
use rand_core::RngCore;
use serde::Deserialize;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

/// Per-link fault knobs. All fields default to "no fault".
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct LinkFault {
    /// Robot slot this rule applies to.
    pub robot: usize,
    /// Probability [0,1] of dropping any packet (inbound command or
    /// outbound sensor frame).
    pub loss: f64,
    /// Fixed outbound sensor latency, in agent ticks.
    pub latency_ticks: u64,
    /// Additional uniform random outbound latency, 0..=jitter_ticks.
    pub jitter_ticks: u64,
    /// Disconnect burst period in ticks (0 = never). Inbound uses the
    /// injector's own message counter as its tick.
    pub disconnect_period_ticks: u64,
    /// Length of each disconnect burst in ticks.
    pub disconnect_ticks: u64,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default)]
struct FaultsSection {
    link: Vec<LinkFault>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default)]
struct FaultsFile {
    faults: FaultsSection,
    link: Vec<LinkFault>,
}

/// Stream mixer so link RNG streams differ from the world's SimRng and
/// from each other, while staying a pure function of (seed, link index).
fn link_seed(world_seed: u64, link_index: usize) -> u64 {
    let mut x = world_seed ^ (link_index as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    x ^= x >> 30;
    x = x.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    x ^= x >> 27;
    x = x.wrapping_mul(0x94D0_49BB_1331_11EB);
    x ^ (x >> 31)
}

/// Stateful fault injector for one agent link. Cheap to keep behind a
/// mutex shared by the conn's read and write tasks.
pub struct FaultInjector {
    cfg: LinkFault,
    rng: SimRng,
    /// (deliver_at_tick, frame) outbound hold queue.
    queue: VecDeque<(u64, SensorFrame)>,
    /// Inbound message counter — the "tick" for inbound disconnect checks
    /// (commands arrive asynchronously; there is no tick on the wire).
    inbound_msgs: u64,
    dropped_out: u64,
    dropped_in: u64,
}

impl FaultInjector {
    pub fn new(cfg: LinkFault, world_seed: u64) -> Self {
        FaultInjector {
            rng: SimRng::seeded(link_seed(world_seed, cfg.robot)),
            cfg,
            queue: VecDeque::new(),
            inbound_msgs: 0,
            dropped_out: 0,
            dropped_in: 0,
        }
    }

    fn disconnected(&self, phase_tick: u64) -> bool {
        let p = self.cfg.disconnect_period_ticks;
        p > 0 && phase_tick % p < self.cfg.disconnect_ticks
    }

    fn roll(&mut self) -> f64 {
        // uniform [0,1): top 53 bits, same trick as rand's Standard sampling
        (self.rng.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Should an inbound actuator command be delivered? Draws exactly one
    /// roll per call and bumps the inbound counter — fixed draw order is
    /// what keeps the pattern deterministic.
    pub fn allow_inbound(&mut self) -> bool {
        let tick = self.inbound_msgs;
        self.inbound_msgs += 1;
        let roll = self.roll();
        if self.disconnected(tick) || roll < self.cfg.loss.clamp(0.0, 1.0) {
            self.dropped_in += 1;
            return false;
        }
        true
    }

    /// Offer an outbound sensor frame. Either drops it (loss/disconnect)
    /// or holds it until `tick + latency + jitter`. Fixed draw order:
    /// loss roll always, jitter roll only if the frame survives.
    pub fn outbound(&mut self, tick: u64, frame: SensorFrame) {
        let loss_roll = self.roll();
        if self.disconnected(tick) || loss_roll < self.cfg.loss.clamp(0.0, 1.0) {
            self.dropped_out += 1;
            return;
        }
        let mut delay = self.cfg.latency_ticks;
        if self.cfg.jitter_ticks > 0 {
            delay += self.rng.next_u64() % (self.cfg.jitter_ticks + 1);
        }
        self.queue.push_back((tick + delay, frame));
    }

    /// Frames whose delay has elapsed at `tick`, in arrival order.
    pub fn drain_due(&mut self, tick: u64) -> Vec<SensorFrame> {
        // queue is not strictly sorted (jitter varies), so scan it
        let mut due = Vec::new();
        let mut i = 0;
        while i < self.queue.len() {
            if self.queue[i].0 <= tick {
                due.push(self.queue.remove(i).unwrap().1);
            } else {
                i += 1;
            }
        }
        due
    }

    /// (dropped_inbound, dropped_outbound) — diagnostic counters.
    pub fn stats(&self) -> (u64, u64) {
        (self.dropped_in, self.dropped_out)
    }
}

/// Per-robot-slot injectors, built once from `--faults` config + seed.
pub struct Faults {
    links: Vec<Option<Arc<Mutex<FaultInjector>>>>,
}

impl Faults {
    pub fn load(path: &std::path::Path, num_robots: usize, seed: u64) -> anyhow::Result<Faults> {
        let text = std::fs::read_to_string(path)?;
        let file: FaultsFile = toml::from_str(&text)
            .map_err(|e| anyhow::anyhow!("invalid faults TOML {}: {e}", path.display()))?;
        let mut links: Vec<Option<Arc<Mutex<FaultInjector>>>> = vec![None; num_robots];
        for cfg in file.faults.link.into_iter().chain(file.link) {
            if cfg.robot >= num_robots {
                eprintln!(
                    "faults: robot {} out of range ({num_robots} robots), rule skipped",
                    cfg.robot
                );
                continue;
            }
            println!(
                "faults: robot {} loss={:.2} latency={}j{} ticks disconnect={}/{}",
                cfg.robot,
                cfg.loss,
                cfg.latency_ticks,
                cfg.jitter_ticks,
                cfg.disconnect_ticks,
                cfg.disconnect_period_ticks
            );
            let robot = cfg.robot;
            links[robot] = Some(Arc::new(Mutex::new(FaultInjector::new(cfg, seed))));
        }
        Ok(Faults { links })
    }

    pub fn get(&self, robot: usize) -> Option<Arc<Mutex<FaultInjector>>> {
        self.links.get(robot).cloned().flatten()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(tick: u64) -> SensorFrame {
        SensorFrame {
            tick,
            position: [tick as f32, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            linvel: [0.0; 3],
            angvel: [0.0; 3],
            contact_impulse: 0.0,
            pieces_held: 0,
            score: 0,
            match_state: crate::r#match::MatchPhase::Teleop,
            match_time: 0.0,
            vision_pose: None,
            mech_angle: None,
        }
    }

    fn rule() -> LinkFault {
        LinkFault {
            robot: 0,
            loss: 0.3,
            latency_ticks: 0,
            jitter_ticks: 5,
            ..Default::default()
        }
    }

    fn pattern(seed: u64, cfg: &LinkFault, n: u64) -> Vec<Option<u64>> {
        let mut inj = FaultInjector::new(cfg.clone(), seed);
        // deliver tick for each frame offered at tick i (None = dropped)
        let mut delivered = vec![None; n as usize];
        for i in 0..n + 50 {
            if i < n {
                inj.outbound(i, frame(i));
            }
            for f in inj.drain_due(i) {
                delivered[f.tick as usize] = Some(i);
            }
        }
        delivered
    }

    #[test]
    fn seeded_run_identical_pattern() {
        let a = pattern(42, &rule(), 200);
        let b = pattern(42, &rule(), 200);
        assert_eq!(a, b);
        // and a different seed should differ somewhere
        let c = pattern(7, &rule(), 200);
        assert_ne!(a, c);
        // 30% loss over 200 frames must drop some but not all
        let dropped = a.iter().filter(|d| d.is_none()).count();
        assert!(dropped > 10 && dropped < 120, "dropped={dropped}");
    }

    #[test]
    fn total_loss_starves_commands() {
        let mut inj = FaultInjector::new(
            LinkFault {
                robot: 0,
                loss: 1.0,
                ..Default::default()
            },
            0,
        );
        for _ in 0..100 {
            assert!(!inj.allow_inbound());
        }
        for i in 0..10 {
            inj.outbound(i, frame(i));
        }
        assert!(inj.drain_due(1000).is_empty());
        assert_eq!(inj.stats(), (100, 10));
    }

    #[test]
    fn latency_delays_but_delivers() {
        let mut inj = FaultInjector::new(
            LinkFault {
                robot: 0,
                latency_ticks: 3,
                ..Default::default()
            },
            0,
        );
        inj.outbound(0, frame(0));
        assert!(inj.drain_due(2).is_empty());
        let due = inj.drain_due(3);
        assert_eq!(due.len(), 1);
        assert_eq!(due[0].tick, 0);
    }

    #[test]
    fn jitter_within_range() {
        let cfg = LinkFault {
            robot: 0,
            jitter_ticks: 4,
            ..Default::default()
        };
        let delivered = pattern(1, &cfg, 100);
        for (i, d) in delivered.iter().enumerate() {
            let at = d.unwrap();
            assert!(at <= i as u64 + 4);
        }
    }

    #[test]
    fn config_parses_both_sections() {
        // [[link]] top-level
        let f: FaultsFile = toml::from_str(
            "[[link]]\nrobot = 1\nloss = 0.5\nlatency_ticks = 2\n",
        )
        .unwrap();
        assert_eq!(f.link.len(), 1);
        assert_eq!(f.link[0].robot, 1);
        // [faults] + [[faults.link]]
        let f: FaultsFile = toml::from_str(
            "[faults]\n[[faults.link]]\nrobot = 2\njitter_ticks = 3\n             disconnect_period_ticks = 100\ndisconnect_ticks = 10\n",
        )
        .unwrap();
        assert_eq!(f.faults.link.len(), 1);
        assert_eq!(f.faults.link[0].disconnect_ticks, 10);
    }

    #[test]
    fn disconnect_burst_drops_window() {
        let mut inj = FaultInjector::new(
            LinkFault {
                robot: 0,
                disconnect_period_ticks: 10,
                disconnect_ticks: 3,
                ..Default::default()
            },
            0,
        );
        // ticks 0..3, 10..13, ... are disconnected
        for t in 0..30 {
            inj.outbound(t, frame(t));
        }
        let due: Vec<u64> = (0..31)
            .flat_map(|t| inj.drain_due(t))
            .map(|f| f.tick)
            .collect();
        for t in 0..30 {
            let in_burst = t % 10 < 3;
            assert_eq!(due.contains(&t), !in_burst, "tick {t}");
        }
        // same for inbound
        let ok: Vec<bool> = (0..30).map(|_| inj.allow_inbound()).collect();
        for t in 0..30 {
            assert_eq!(ok[t], t % 10 >= 3, "inbound {t}");
        }
    }
}

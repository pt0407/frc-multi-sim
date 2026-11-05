//! Seeded simulation RNG (PLAN §0 determinism contract / §3).
//!
//! All stochastic features (piece spawns, sensor noise, domain
//! randomization) must draw from a `SimRng` owned by `World` — never from
//! `rand::thread_rng()`. Backed by `rand_pcg::Pcg32`: small, fast,
//! reproducible across platforms and Rust versions.
use rand_core::{RngCore, SeedableRng};
use rand_pcg::Pcg32;

/// Wrapper so the concrete generator can be swapped without touching
/// call sites, and so the seed travels with the world for replay headers.
pub struct SimRng {
    inner: Pcg32,
    seed: u64,
}

impl SimRng {
    pub fn seeded(seed: u64) -> Self {
        SimRng {
            // use the seed both as state and stream selector — deterministic
            inner: Pcg32::seed_from_u64(seed),
            seed,
        }
    }

    /// The seed this RNG was constructed with (recorded in replay headers).
    pub fn seed(&self) -> u64 {
        self.seed
    }
}

impl RngCore for SimRng {
    fn next_u32(&mut self) -> u32 {
        self.inner.next_u32()
    }
    fn next_u64(&mut self) -> u64 {
        self.inner.next_u64()
    }
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.inner.fill_bytes(dest)
    }
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.inner.try_fill_bytes(dest)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_stream() {
        let mut a = SimRng::seeded(42);
        let mut b = SimRng::seeded(42);
        for _ in 0..100 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn different_seed_differs() {
        let mut a = SimRng::seeded(1);
        let mut b = SimRng::seeded(2);
        assert_ne!(a.next_u64(), b.next_u64());
    }
}

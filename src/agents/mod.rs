//! Agent-facing services: WS protocol server, NT4 bridge, and the shared
//! per-robot command slots they write into (PLAN A4).
pub mod nt_bridge;
pub mod ws;

use crate::ActuatorFrame;
use std::sync::Mutex;

/// Latest actuator command per robot slot, plus a monotonically increasing
/// sequence number per slot (used by `lockstep` mode to detect fresh
/// commands) and a connected-agent flag per slot.
pub struct CommandSlots {
    inner: Mutex<Inner>,
}

struct Inner {
    slots: Vec<Slot>,
}

#[derive(Default, Clone)]
struct Slot {
    cmd: Option<ActuatorFrame>,
    seq: u64,
    ws_connected: bool,
}

impl CommandSlots {
    pub fn new(num_robots: usize) -> Self {
        CommandSlots {
            inner: Mutex::new(Inner {
                slots: vec![Slot::default(); num_robots],
            }),
        }
    }

    /// Current command snapshot for all robots (one physics step's inputs).
    pub fn snapshot(&self) -> Vec<Option<ActuatorFrame>> {
        self.inner
            .lock()
            .unwrap()
            .slots
            .iter()
            .map(|s| s.cmd.clone())
            .collect()
    }

    pub fn set(&self, robot: usize, cmd: ActuatorFrame) {
        let mut g = self.inner.lock().unwrap();
        let s = &mut g.slots[robot];
        s.cmd = Some(cmd);
        s.seq += 1;
    }

    pub fn clear(&self, robot: usize) {
        let mut g = self.inner.lock().unwrap();
        g.slots[robot].cmd = None;
    }

    pub fn seq(&self, robot: usize) -> u64 {
        self.inner.lock().unwrap().slots[robot].seq
    }

    /// Sequence numbers for every robot (lockstep barrier baseline).
    pub fn seqs(&self) -> Vec<u64> {
        self.inner
            .lock()
            .unwrap()
            .slots
            .iter()
            .map(|s| s.seq)
            .collect()
    }

    pub fn set_ws_connected(&self, robot: usize, connected: bool) {
        self.inner.lock().unwrap().slots[robot].ws_connected = connected;
    }

    /// Robots with a live WS agent — these are the ones the `lockstep`
    /// barrier waits on. NT-bridged robots free-run (their poller writes
    /// continuously, so gating on them would deadlock).
    pub fn ws_connected_robots(&self) -> Vec<usize> {
        self.inner
            .lock()
            .unwrap()
            .slots
            .iter()
            .enumerate()
            .filter(|(_, s)| s.ws_connected)
            .map(|(i, _)| i)
            .collect()
    }
}

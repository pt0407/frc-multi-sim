//! Event-sourced input log + replay/diff (issue #5, PLAN A1).
//!
//! Binary format (all little-endian):
//!
//! ```text
//! header:
//!   magic            "FMSL"                       [4]u8
//!   format_version   u32                          2 (v1 logs still read)
//!   protocol_version u32
//!   seed             u64
//!   config_hash      u64
//!   num_robots       u32
//!   per robot: mass f32, half_extents [3]f32, spawn [3]f32,
//!              [v2] has_intake u8, if set: x_min x_max z_min z_max y_min
//!              y_max f32
//!   [v2] has_pieces u8, if set: piece table (n_zones u32, per zone:
//!        center [2]f32 half_extents [2]f32 count u32; n_hubs u32, per hub:
//!        center [2]f32 radius f32 entry_height f32 alliance u32)
//! records (until EOF):
//!   kind u8 = 1 (command):
//!     tick u64, robot u32, has_cmd u8,
//!     if has_cmd: force [2]f32, torque f32, n_modules u32,
//!                 per module: steer f32, force f32
//!                 [v2] intake_active u8, has_shoot u8,
//!                      if has_shoot: launch_velocity, hood_angle,
//!                      turret_yaw, launch_height f32
//!   kind u8 = 2 (state sample, one per robot per agent tick):
//!     tick u64, robot u32, position [3]f32, rotation [4]f32,
//!     linvel [3]f32, angvel [3]f32
//! ```
//!
//! The log is self-contained: the spec table lets `--replay` rebuild the
//! exact world, and the embedded state samples let `--diff` detect
//! divergence without a second log file.
use crate::archetype::IntakeSpec;
use crate::protocol::ShotCommand;
use crate::world::pieces::{HubZone, PieceTable, SpawnZone};
use crate::world::{RobotSpec, World, AGENT_EVERY};
use crate::{ActuatorFrame, SensorFrame};
use anyhow::{bail, Context, Result};
use std::io::{BufWriter, Read, Write};
use std::path::Path;

const MAGIC: &[u8; 4] = b"FMSL";
const FORMAT_VERSION: u32 = 2;
const KIND_CMD: u8 = 1;
const KIND_STATE: u8 = 2;

#[derive(Debug, Clone)]
pub struct LogHeader {
    pub protocol_version: u32,
    pub seed: u64,
    pub config_hash: u64,
    pub specs: Vec<RobotSpec>,
    /// game-piece spawn/scoring table (v2 headers; None for v1 logs)
    pub pieces: Option<PieceTable>,
}

#[derive(Debug, Clone)]
pub enum Record {
    /// The command slot contents applied for `robot` starting at `tick`
    /// (only written on change).
    Cmd {
        tick: u64,
        robot: u32,
        cmd: Option<ActuatorFrame>,
    },
    /// Chassis pose/velocity sample at `tick` (for diffing).
    State { tick: u64, robot: u32, sample: StateSample },
}

/// Chassis state mirror of `SensorFrame` minus contact (deterministic part).
#[derive(Debug, Clone, PartialEq)]
pub struct StateSample {
    pub position: [f32; 3],
    pub rotation: [f32; 4],
    pub linvel: [f32; 3],
    pub angvel: [f32; 3],
}

impl From<&SensorFrame> for StateSample {
    fn from(s: &SensorFrame) -> Self {
        StateSample {
            position: s.position,
            rotation: s.rotation,
            linvel: s.linvel,
            angvel: s.angvel,
        }
    }
}

/// First point of divergence between two state streams.
#[derive(Debug)]
pub struct Divergence {
    pub tick: u64,
    pub robot: u32,
    /// max |component| difference across position+linvel at that tick
    pub magnitude: f32,
    /// number of state samples compared before diverging
    pub compared: usize,
}

// ---- writing -------------------------------------------------------------

/// Incremental binary log writer; flush happens on drop of the BufWriter.
pub struct LogWriter {
    w: BufWriter<std::fs::File>,
}

impl LogWriter {
    pub fn create(path: &Path, header: &LogHeader) -> Result<Self> {
        let f = std::fs::File::create(path)
            .with_context(|| format!("creating replay log {}", path.display()))?;
        let mut w = BufWriter::new(f);
        w.write_all(MAGIC)?;
        w.write_all(&FORMAT_VERSION.to_le_bytes())?;
        w.write_all(&header.protocol_version.to_le_bytes())?;
        w.write_all(&header.seed.to_le_bytes())?;
        w.write_all(&header.config_hash.to_le_bytes())?;
        w.write_all(&(header.specs.len() as u32).to_le_bytes())?;
        for s in &header.specs {
            w.write_all(&s.mass.to_le_bytes())?;
            for v in s.half_extents {
                w.write_all(&v.to_le_bytes())?;
            }
            for v in s.spawn {
                w.write_all(&v.to_le_bytes())?;
            }
            match &s.intake {
                Some(i) => {
                    w.write_all(&[1])?;
                    for v in [i.x_min, i.x_max, i.z_min, i.z_max, i.y_min, i.y_max] {
                        w.write_all(&v.to_le_bytes())?;
                    }
                }
                None => w.write_all(&[0])?,
            }
        }
        // v2: game-piece table
        match &header.pieces {
            Some(t) => {
                w.write_all(&[1])?;
                w.write_all(&(t.zones.len() as u32).to_le_bytes())?;
                for z in &t.zones {
                    for v in z.center {
                        w.write_all(&v.to_le_bytes())?;
                    }
                    for v in z.half_extents {
                        w.write_all(&v.to_le_bytes())?;
                    }
                    w.write_all(&z.count.to_le_bytes())?;
                }
                w.write_all(&(t.hubs.len() as u32).to_le_bytes())?;
                for h in &t.hubs {
                    for v in h.center {
                        w.write_all(&v.to_le_bytes())?;
                    }
                    w.write_all(&h.radius.to_le_bytes())?;
                    w.write_all(&h.entry_height.to_le_bytes())?;
                    w.write_all(&h.alliance.to_le_bytes())?;
                }
            }
            None => w.write_all(&[0])?,
        }
        Ok(LogWriter { w })
    }

    pub fn record(&mut self, r: &Record) -> Result<()> {
        match r {
            Record::Cmd { tick, robot, cmd } => {
                self.w.write_all(&[KIND_CMD])?;
                self.w.write_all(&tick.to_le_bytes())?;
                self.w.write_all(&robot.to_le_bytes())?;
                match cmd {
                    Some(c) => {
                        self.w.write_all(&[1])?;
                        self.write_actuator(c)?;
                    }
                    None => self.w.write_all(&[0])?,
                }
            }
            Record::State {
                tick,
                robot,
                sample,
            } => {
                self.w.write_all(&[KIND_STATE])?;
                self.w.write_all(&tick.to_le_bytes())?;
                self.w.write_all(&robot.to_le_bytes())?;
                self.write_sample(sample)?;
            }
        }
        Ok(())
    }

    fn write_actuator(&mut self, c: &ActuatorFrame) -> Result<()> {
        for v in c.force {
            self.w.write_all(&v.to_le_bytes())?;
        }
        self.w.write_all(&c.torque.to_le_bytes())?;
        let mods = c.modules.as_deref().unwrap_or(&[]);
        self.w.write_all(&(mods.len() as u32).to_le_bytes())?;
        for m in mods {
            self.w.write_all(&m.steer.to_le_bytes())?;
            self.w.write_all(&m.force.to_le_bytes())?;
        }
        // v2: intake flag + optional shot command
        self.w.write_all(&[c.intake_active as u8])?;
        match &c.shoot {
            Some(s) => {
                self.w.write_all(&[1])?;
                for v in [
                    s.launch_velocity,
                    s.hood_angle,
                    s.turret_yaw,
                    s.launch_height,
                ] {
                    self.w.write_all(&v.to_le_bytes())?;
                }
            }
            None => self.w.write_all(&[0])?,
        }
        Ok(())
    }

    fn write_sample(&mut self, s: &StateSample) -> Result<()> {
        for arr in [
            &s.position[..],
            &s.rotation[..],
            &s.linvel[..],
            &s.angvel[..],
        ] {
            for v in arr {
                self.w.write_all(&v.to_le_bytes())?;
            }
        }
        Ok(())
    }

    pub fn flush(&mut self) -> Result<()> {
        self.w.flush().map_err(Into::into)
    }
}

// ---- reading -------------------------------------------------------------

fn r_u32(r: &mut impl Read) -> Result<u32> {
    let mut b = [0u8; 4];
    r.read_exact(&mut b)?;
    Ok(u32::from_le_bytes(b))
}
fn r_u64(r: &mut impl Read) -> Result<u64> {
    let mut b = [0u8; 8];
    r.read_exact(&mut b)?;
    Ok(u64::from_le_bytes(b))
}
fn r_f32(r: &mut impl Read) -> Result<f32> {
    Ok(f32::from_bits(r_u32(r)?))
}
fn r_f32s<const N: usize>(r: &mut impl Read) -> Result<[f32; N]> {
    let mut out = [0.0f32; N];
    for v in out.iter_mut() {
        *v = r_f32(r)?;
    }
    Ok(out)
}

pub fn read_log(path: &Path) -> Result<(LogHeader, Vec<Record>)> {
    let mut r = std::io::BufReader::new(
        std::fs::File::open(path)
            .with_context(|| format!("opening replay log {}", path.display()))?,
    );
    let mut magic = [0u8; 4];
    r.read_exact(&mut magic).context("reading magic")?;
    if &magic != MAGIC {
        bail!("{}: not an FMSL replay log", path.display());
    }
    let fmt = r_u32(&mut r)?;
    if fmt != FORMAT_VERSION && fmt != 1 {
        bail!("unsupported log format version {fmt} (expected 1 or {FORMAT_VERSION})");
    }
    let header = LogHeader {
        protocol_version: r_u32(&mut r)?,
        seed: r_u64(&mut r)?,
        config_hash: r_u64(&mut r)?,
        specs: {
            let n = r_u32(&mut r)? as usize;
            (0..n)
                .map(|_| {
                    let mass = r_f32(&mut r)?;
                    let half_extents = r_f32s(&mut r)?;
                    let spawn = r_f32s(&mut r)?;
                    let intake = if fmt >= 2 {
                        let mut b = [0u8; 1];
                        r.read_exact(&mut b)?;
                        if b[0] == 1 {
                            Some(IntakeSpec {
                                x_min: r_f32(&mut r)?,
                                x_max: r_f32(&mut r)?,
                                z_min: r_f32(&mut r)?,
                                z_max: r_f32(&mut r)?,
                                y_min: r_f32(&mut r)?,
                                y_max: r_f32(&mut r)?,
                            })
                        } else {
                            None
                        }
                    } else {
                        None
                    };
                    Ok(RobotSpec {
                        mass,
                        half_extents,
                        spawn,
                        intake,
                    })
                })
                .collect::<Result<Vec<_>>>()?
        },
        pieces: if fmt >= 2 {
            let mut b = [0u8; 1];
            r.read_exact(&mut b)?;
            if b[0] == 1 {
                let nz = r_u32(&mut r)? as usize;
                let zones = (0..nz)
                    .map(|_| {
                        Ok(SpawnZone {
                            center: r_f32s(&mut r)?,
                            half_extents: r_f32s(&mut r)?,
                            count: r_u32(&mut r)?,
                        })
                    })
                    .collect::<Result<Vec<_>>>()?;
                let nh = r_u32(&mut r)? as usize;
                let hubs = (0..nh)
                    .map(|_| {
                        Ok(HubZone {
                            center: r_f32s(&mut r)?,
                            radius: r_f32(&mut r)?,
                            entry_height: r_f32(&mut r)?,
                            alliance: r_u32(&mut r)?,
                        })
                    })
                    .collect::<Result<Vec<_>>>()?;
                Some(PieceTable { zones, hubs })
            } else {
                None
            }
        } else {
            None
        },
    };
    let mut records = Vec::new();
    loop {
        let mut kind = [0u8; 1];
        match r.read_exact(&mut kind) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => break,
            Err(e) => return Err(e).context("reading record kind"),
        }
        let parsed = match kind[0] {
            KIND_CMD => read_cmd_record(&mut r, fmt),
            KIND_STATE => read_state_record(&mut r),
            k => bail!("unknown record kind {k} in {}", path.display()),
        };
        match parsed {
            Ok(rec) => records.push(rec),
            // tolerate a truncated tail record (e.g. server killed before
            // its final flush) — everything before it is still usable
            Err(e) if is_eof(&e) => {
                eprintln!(
                    "warning: {}: truncated tail record, using {} complete records",
                    path.display(),
                    records.len()
                );
                break;
            }
            Err(e) => return Err(e),
        }
    }
    Ok((header, records))
}

fn is_eof(e: &anyhow::Error) -> bool {
    e.downcast_ref::<std::io::Error>()
        .map(|e| e.kind() == std::io::ErrorKind::UnexpectedEof)
        .unwrap_or(false)
}

fn read_cmd_record(r: &mut impl Read, fmt: u32) -> Result<Record> {
    let tick = r_u64(r)?;
    let robot = r_u32(r)?;
    let has = {
        let mut b = [0u8; 1];
        r.read_exact(&mut b)?;
        b[0]
    };
    let cmd = if has == 1 {
        let force = r_f32s(r)?;
        let torque = r_f32(r)?;
        let n = r_u32(r)? as usize;
        let modules = if n > 0 {
            Some(
                (0..n)
                    .map(|_| {
                        Ok(crate::drivetrain::ModuleCommand {
                            steer: r_f32(r)?,
                            force: r_f32(r)?,
                        })
                    })
                    .collect::<Result<Vec<_>>>()?,
            )
        } else {
            None
        };
        // v2: intake flag + optional shot command
        let (intake_active, shoot) = if fmt >= 2 {
            let mut b = [0u8; 1];
            r.read_exact(&mut b)?;
            let intake = b[0] == 1;
            r.read_exact(&mut b)?;
            let shoot = if b[0] == 1 {
                Some(ShotCommand {
                    launch_velocity: r_f32(r)?,
                    hood_angle: r_f32(r)?,
                    turret_yaw: r_f32(r)?,
                    launch_height: r_f32(r)?,
                })
            } else {
                None
            };
            (intake, shoot)
        } else {
            (false, None)
        };
        Some(ActuatorFrame {
            force,
            torque,
            modules,
            intake_active,
            shoot,
        })
    } else {
        None
    };
    Ok(Record::Cmd { tick, robot, cmd })
}

fn read_state_record(r: &mut impl Read) -> Result<Record> {
    Ok(Record::State {
        tick: r_u64(r)?,
        robot: r_u32(r)?,
        sample: StateSample {
            position: r_f32s(r)?,
            rotation: r_f32s(r)?,
            linvel: r_f32s(r)?,
            angvel: r_f32s(r)?,
        },
    })
}

// ---- replay + diff -------------------------------------------------------

/// Re-run a log headless: rebuild the world from the header's spec table +
/// seed, replay command events at their ticks, and return the resulting
/// state samples (one per robot per agent tick, same cadence as `record`).
pub fn replay(header: &LogHeader, records: &[Record]) -> Vec<(u64, u32, StateSample)> {
    let empty = PieceTable::default();
    let mut world = World::with_config(
        &header.specs,
        header.seed,
        header.pieces.as_ref().unwrap_or(&empty),
    );
    let n = world.num_robots();
    let mut cmds: Vec<Option<ActuatorFrame>> = vec![None; n];
    let cmd_events: Vec<&Record> = records
        .iter()
        .filter(|r| matches!(r, Record::Cmd { .. }))
        .collect();
    let last_tick = records
        .iter()
        .map(|r| match r {
            Record::Cmd { tick, .. } | Record::State { tick, .. } => *tick,
        })
        .max()
        .unwrap_or(0);
    let mut out = Vec::new();
    let mut ev = 0;
    for tick in 0..=last_tick {
        while ev < cmd_events.len() {
            if let Record::Cmd {
                tick: et,
                robot,
                cmd,
            } = cmd_events[ev]
            {
                if *et > tick {
                    break;
                }
                if (*robot as usize) < n {
                    cmds[*robot as usize] = cmd.clone();
                }
                ev += 1;
            }
        }
        world.step(&cmds);
        if tick % AGENT_EVERY as u64 == 0 {
            for r in 0..n {
                out.push((tick, r as u32, StateSample::from(&world.sensor_frame(r, tick))));
            }
        }
    }
    out
}

/// Compare recorded (golden) states against replayed states; returns the
/// first divergence or `None` if identical within `tol` (tol = 0 → bitwise).
pub fn diff(
    golden: &[(u64, u32, StateSample)],
    actual: &[(u64, u32, StateSample)],
    tol: f32,
) -> Option<Divergence> {
    use std::collections::HashMap;
    let actual_map: HashMap<(u64, u32), &StateSample> =
        actual.iter().map(|(t, r, s)| ((*t, *r), s)).collect();
    let mut compared = 0;
    for (tick, robot, g) in golden {
        let Some(a) = actual_map.get(&(*tick, *robot)) else {
            continue;
        };
        compared += 1;
        let mut mag = 0.0f32;
        let mut differ = false;
        for (x, y) in g
            .position
            .iter()
            .chain(g.linvel.iter())
            .chain(g.rotation.iter())
            .chain(g.angvel.iter())
            .zip(
                a.position
                    .iter()
                    .chain(a.linvel.iter())
                    .chain(a.rotation.iter())
                    .chain(a.angvel.iter()),
            )
        {
            let d = (x - y).abs();
            mag = mag.max(d);
            if x.to_bits() != y.to_bits() && d > tol {
                differ = true;
            }
        }
        if differ {
            return Some(Divergence {
                tick: *tick,
                robot: *robot,
                magnitude: mag,
                compared,
            });
        }
    }
    None
}

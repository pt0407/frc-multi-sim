//! Per-wheel drivetrain traction model (issue #2).
//!
//! Conceptually ported from maple-sim's `SwerveModuleSimulation`
//! (Shenzhen-Robotics-Alliance/maple-sim): each wheel/module is a force
//! application point on the chassis with a friction-circle grip limit
//! `F_max = mu * N_module`. The requested propelling force is applied in the
//! module's steered direction; if its magnitude exceeds the grip limit the
//! module is skidding and only `F_max` is transmitted (the excess is slip).
//! maple-sim runs on dyn4j/2D; here we adapt the idea to rapier3d by
//! returning per-module (application point, world force) pairs that the
//! server applies via `RigidBody::add_force_at_point`, so yaw torque emerges
//! naturally from off-center forces.
//!
//! Two profiles:
//! - `Simple`: the legacy sliding box — chassis-level force+torque applied at
//!   the center of mass, no traction limit (M0 behavior, kept as fallback).
//! - `Swerve`: 4 steerable modules; each command is (steer angle in robot
//!   frame, force in newtons). A compatibility shim maps a legacy
//!   chassis-level `force[2]+torque` command onto the modules by splitting
//!   the force evenly and adding a tangential component per module for the
//!   requested torque — so `drive`/`robot_process` agents keep working.

use crate::ActuatorFrame;
use serde::{Deserialize, Serialize};

pub const GRAVITY: f32 = 9.81;

/// Per-module command: steer direction (robot frame, radians, 0 = +x) and
/// propelling force in newtons along that direction.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ModuleCommand {
    pub steer: f32,
    pub force: f32,
}

/// A resolved force to apply to the chassis rigid body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolvedForce {
    /// application point in the chassis XZ plane, robot frame (meters)
    pub local_point: [f32; 2],
    /// world-frame force [x, z] (newtons)
    pub force: [f32; 2],
    /// extra yaw torque to apply directly at the CoM (N*m). Only used by the
    /// `Simple` profile; swerve produces torque through off-center forces.
    pub torque: f32,
}

/// Static placement of a drive module relative to chassis center (robot
/// frame, meters, +x forward, +z left).
#[derive(Debug, Clone, Copy)]
pub struct ModuleSpec {
    pub x: f32,
    pub z: f32,
}

/// Per-module bookkeeping for slip reporting (e.g. future SensorFrame field).
#[derive(Debug, Clone, Copy, Default)]
pub struct ModuleState {
    pub requested: f32,
    pub applied: f32,
    /// fraction of requested force lost to slip, 0.0 = full grip, ->1.0 = skid
    pub slip: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrivetrainProfile {
    Simple,
    Swerve,
}

pub struct Drivetrain {
    pub profile: DrivetrainProfile,
    /// robot mass (kg) — used to derive per-module normal force
    pub mass: f32,
    /// coefficient of friction wheel<->carpet (FRC ~1.1-1.3)
    pub mu: f32,
    /// motor/software cap on a single module's propelling force (N)
    pub max_module_force: f32,
    pub modules: Vec<ModuleSpec>,
    states: Vec<ModuleState>,
}

/// Rotate a robot-frame 2D vector into world frame given chassis yaw.
/// Rapier rotation about +y: (x,z) -> (x cos a + z sin a, -x sin a + z cos a).
pub fn rot2(v: [f32; 2], yaw: f32) -> [f32; 2] {
    let (s, c) = yaw.sin_cos();
    [v[0] * c + v[1] * s, -v[0] * s + v[1] * c]
}

/// Extract yaw from a quaternion [x, y, z, w] (same convention as the
/// server's NT bridge).
pub fn yaw_from_quat(q: [f32; 4]) -> f32 {
    let [qx, qy, qz, qw] = q;
    2.0 * (qw * qy + qx * qz).atan2(1.0 - 2.0 * (qy * qy + qx * qx))
}

impl Drivetrain {
    /// Legacy sliding box: forces applied at CoM, unlimited traction.
    pub fn simple(mass: f32) -> Self {
        Drivetrain {
            profile: DrivetrainProfile::Simple,
            mass,
            mu: f32::INFINITY,
            max_module_force: f32::INFINITY,
            modules: vec![ModuleSpec { x: 0.0, z: 0.0 }],
            states: vec![ModuleState::default()],
        }
    }

    /// 4-module swerve: modules at the corners of a `chassis_half`-sized box,
    /// inset to `chassis_half * 0.75` (typical module inset from the frame
    /// perimeter).
    pub fn swerve(mass: f32, chassis_half: f32) -> Self {
        let o = chassis_half * 0.75;
        let modules = vec![
            ModuleSpec { x: o, z: o },   // front-left
            ModuleSpec { x: o, z: -o },  // front-right
            ModuleSpec { x: -o, z: o },  // back-left
            ModuleSpec { x: -o, z: -o }, // back-right
        ];
        let n = modules.len();
        Drivetrain {
            profile: DrivetrainProfile::Swerve,
            mass,
            mu: 1.2,
            max_module_force: 800.0,
            modules,
            states: vec![ModuleState::default(); n],
        }
    }

    pub fn normal_per_module(&self) -> f32 {
        self.mass * GRAVITY / self.modules.len().max(1) as f32
    }

    /// Friction-circle grip limit for one contact patch.
    pub fn traction_limit(&self, normal: f32) -> f32 {
        self.mu * normal
    }

    /// Clamp a scalar requested force to the grip limit and motor cap.
    /// Returns (applied, slip_fraction).
    pub fn clamp_traction(&self, requested: f32, normal: f32) -> (f32, f32) {
        let cap = self.traction_limit(normal).min(self.max_module_force);
        let applied = requested.clamp(-cap, cap);
        let slip = if requested.abs() > f32::EPSILON {
            (1.0 - (applied / requested).abs()).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (applied, slip)
    }

    /// Compatibility shim: allocate a legacy chassis-level (force, torque)
    /// command across the modules as world-frame force vectors. Each module
    /// gets `F_net / n` plus a tangential component `T / (n * r)` so the
    /// off-center forces reproduce the requested yaw torque.
    fn alloc_chassis_command(&self, force: [f32; 2], torque: f32, yaw: f32) -> Vec<[f32; 2]> {
        let n = self.modules.len() as f32;
        self.modules
            .iter()
            .map(|m| {
                let r = rot2([m.x, m.z], yaw);
                let r_len = (r[0] * r[0] + r[1] * r[1]).sqrt();
                // tangential dir with r x t = +|r| (positive yaw torque):
                let t = if r_len > 1e-6 {
                    [r[1] / r_len, -r[0] / r_len]
                } else {
                    [0.0, 0.0]
                };
                let f_t = torque / (n * r_len.max(1e-6));
                [force[0] / n + t[0] * f_t, force[1] / n + t[1] * f_t]
            })
            .collect()
    }

    /// Turn an ActuatorFrame into per-point forces (robot-frame application
    /// points, world-frame force vectors). Mutates `states` for slip
    /// reporting.
    pub fn resolve(&mut self, cmd: &ActuatorFrame, yaw: f32) -> Vec<ResolvedForce> {
        match self.profile {
            DrivetrainProfile::Simple => {
                self.states[0].requested = (cmd.force[0].powi(2) + cmd.force[1].powi(2)).sqrt();
                self.states[0].applied = self.states[0].requested;
                self.states[0].slip = 0.0;
                vec![ResolvedForce {
                    local_point: [0.0, 0.0],
                    force: [cmd.force[0], cmd.force[1]],
                    torque: cmd.torque,
                }]
            }
            DrivetrainProfile::Swerve => {
                let normal = self.normal_per_module();
                // desired world-frame force per module
                let desired: Vec<[f32; 2]> = match &cmd.modules {
                    Some(mods) => self
                        .modules
                        .iter()
                        .enumerate()
                        .map(|(i, _)| {
                            let mc = mods.get(i).copied().unwrap_or(ModuleCommand {
                                steer: 0.0,
                                force: 0.0,
                            });
                            let dir = rot2([mc.steer.cos(), mc.steer.sin()], yaw);
                            [dir[0] * mc.force, dir[1] * mc.force]
                        })
                        .collect(),
                    None => self.alloc_chassis_command(cmd.force, cmd.torque, yaw),
                };
                let cap = self.traction_limit(normal).min(self.max_module_force);
                let mut out = Vec::with_capacity(self.modules.len());
                for (i, m) in self.modules.iter().enumerate() {
                    let f = desired[i];
                    let mag = (f[0] * f[0] + f[1] * f[1]).sqrt();
                    // friction circle: clamp the *combined* 2D force
                    // magnitude, preserving direction.
                    let applied_mag = mag.clamp(0.0, cap);
                    let scale = if mag > 1e-9 { applied_mag / mag } else { 0.0 };
                    let st = &mut self.states[i];
                    st.requested = mag;
                    st.applied = applied_mag;
                    st.slip = if mag > f32::EPSILON {
                        (1.0 - applied_mag / mag).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    out.push(ResolvedForce {
                        local_point: [m.x, m.z],
                        force: [f[0] * scale, f[1] * scale],
                        torque: 0.0,
                    });
                }
                out
            }
        }
    }

    /// Slip states of each module (same order as `modules`).
    pub fn states(&self) -> &[ModuleState] {
        &self.states
    }

    /// Worst slip fraction across modules, for logging/telemetry.
    pub fn max_slip(&self) -> f32 {
        self.states.iter().map(|s| s.slip).fold(0.0, f32::max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn swerve() -> Drivetrain {
        Drivetrain::swerve(60.0, 0.4)
    }

    #[test]
    fn traction_limit_is_mu_times_normal() {
        let d = swerve();
        let n = d.normal_per_module(); // 60*9.81/4 = 147.15
        assert!((n - 147.15).abs() < 0.01);
        assert!((d.traction_limit(n) - 1.2 * n).abs() < 0.01);
    }

    #[test]
    fn under_limit_no_slip() {
        let d = swerve();
        let (applied, slip) = d.clamp_traction(100.0, 147.15);
        assert_eq!(applied, 100.0);
        assert_eq!(slip, 0.0);
    }

    #[test]
    fn over_limit_clamps_and_reports_slip() {
        let d = swerve();
        let cap = d.traction_limit(147.15); // ~176.6 N, below motor cap
        let (applied, slip) = d.clamp_traction(300.0, 147.15);
        assert!((applied - cap).abs() < 1e-4);
        assert!(slip > 0.4 && slip < 0.5); // 1 - 176.6/300
    }

    #[test]
    fn negative_and_zero_cases() {
        let d = swerve();
        assert_eq!(d.clamp_traction(-50.0, 147.15).0, -50.0);
        assert_eq!(d.clamp_traction(-9999.0, 147.15).0, -d.traction_limit(147.15));
        assert_eq!(d.clamp_traction(10.0, 0.0).0, 0.0); // no normal -> no grip
        assert_eq!(d.clamp_traction(0.0, 147.15).1, 0.0);
    }

    #[test]
    fn simple_profile_passthrough() {
        let mut d = Drivetrain::simple(60.0);
        let cmd = ActuatorFrame {
            force: [1200.0, -300.0],
            torque: 5.0,
            modules: None,
            intake_active: false,
            shoot: None,
        };
        let out = d.resolve(&cmd, 0.7);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].force, [1200.0, -300.0]);
        assert_eq!(out[0].torque, 5.0);
        assert_eq!(out[0].local_point, [0.0, 0.0]);
    }

    #[test]
    fn swerve_allocates_net_force_and_torque() {
        let mut d = swerve();
        // small command: under grip limit, so applied == requested
        let cmd = ActuatorFrame {
            force: [100.0, 0.0],
            torque: 4.0,
            modules: None,
            intake_active: false,
            shoot: None,
        };
        let out = d.resolve(&cmd, 0.0);
        assert_eq!(out.len(), 4);
        // net force sums to commanded force
        let fx: f32 = out.iter().map(|f| f.force[0]).sum();
        let fz: f32 = out.iter().map(|f| f.force[1]).sum();
        assert!((fx - 100.0).abs() < 1e-3, "fx={fx}");
        assert!(fz.abs() < 1e-3, "fz={fz}");
        // net yaw torque: tau_y = r_z*F_x - r_x*F_z
        let tau: f32 = out
            .iter()
            .map(|f| f.local_point[1] * f.force[0] - f.local_point[0] * f.force[1])
            .sum();
        assert!((tau - 4.0).abs() < 1e-3, "tau={tau}");
    }

    #[test]
    fn swerve_big_command_slips() {
        let mut d = swerve();
        let cmd = ActuatorFrame {
            force: [2400.0, 0.0],
            torque: 0.0,
            modules: None,
            intake_active: false,
            shoot: None,
        };
        let out = d.resolve(&cmd, 0.0);
        let total: f32 = out
            .iter()
            .map(|f| (f.force[0].powi(2) + f.force[1].powi(2)).sqrt())
            .sum();
        // grip caps total near mu*m*g = 1.2*60*9.81 = 706.3 N
        assert!(total < 710.0 && total > 700.0, "total={total}");
        assert!(d.max_slip() > 0.5);
    }

    #[test]
    fn per_module_steer_direction() {
        let mut d = swerve();
        // drive all modules sideways (+z robot frame) with robot yawed 90deg
        let cmd = ActuatorFrame {
            force: [0.0, 0.0],
            torque: 0.0,
            modules: Some(vec![
                ModuleCommand { steer: std::f32::consts::FRAC_PI_2, force: 50.0 };
                4
            ]),
            intake_active: false,
            shoot: None,
        };
        let out = d.resolve(&cmd, std::f32::consts::FRAC_PI_2);
        for f in &out {
            // robot +z rotated by yaw=+90deg about +y -> world +x
            assert!(f.force[0] > 40.0, "f={:?}", f);
            assert!(f.force[1].abs() < 5.0);
        }
    }

    #[test]
    fn rot2_rotates_about_y() {
        let v = rot2([1.0, 0.0], std::f32::consts::FRAC_PI_2);
        assert!((v[0]).abs() < 1e-6 && (v[1] + 1.0).abs() < 1e-6);
    }
}

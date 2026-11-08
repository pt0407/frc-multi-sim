//! Kinematic mechanism joints — the MoSim-style layer. Each `JointSpec`
//! is a 1-DOF mechanism (roller, flywheel, swerve steer, wheel) whose
//! state is integrated server-side with an accel-limited motor model and
//! streamed to the viz, which binds it to named CAD subtrees. No rigid
//! bodies: these are display/gameplay-state joints, like MoSim's
//! kinematic roller/flywheel components.
//!
//! Motor model (per channel, fixed dt):
//!   desired_vel = servo(target)            // position joints
//!   vel        += clamp(desired_vel - vel, ±accel·dt)
//!   pos        += vel·dt                   // angle accumulates (rad)
//! Spin joints servo velocity directly; position joints servo a
//! velocity toward the target angle (trapezoidal-ish, max_vel capped);
//! steer/wheel derive their target from drivetrain module state.
use crate::archetype::{JointDriver, JointMode, JointSpec, ShotPreset};

/// Resolve a robot's shot preset for this tick: the first joint spec
/// carrying a `shot_table` interpolates at `dist` (MoSim
/// RebuiltInterpolation — one table drives both flywheel RPM and hood).
pub fn resolve_shot_preset(joints: &[JointSim], dist: Option<f32>) -> Option<ShotPreset> {
    let dist = dist?;
    joints
        .iter()
        .find_map(|j| j.spec.shot_table.as_deref().and_then(|t| crate::archetype::shot_preset_at(t, dist)))
}
use crate::drivetrain::Drivetrain;
use crate::protocol::{ActuatorFrame, JointCmdMode};

/// Runtime state for one robot's mechanism joints.
pub struct JointSim {
    pub spec: JointSpec,
    /// channel count: 1 for spin/position, n_modules for steer/wheel
    n: usize,
    /// joint angle per channel (rad); spin/wheel angles accumulate
    /// unbounded, position/steer stay near their targets
    pub pos: Vec<f32>,
    /// joint velocity per channel (rad/s)
    pub vel: Vec<f32>,
    /// last commanded spin target (rad/s) — `shooter_at_speed` compares
    /// against this so distance-keyed shot tables scale the tolerance
    speed_target: f32,
    /// last position-joint target (rad) — `hood_at_target`/
    /// `turret_at_target` compare the actual angle against it
    pos_target: f32,
}

impl JointSim {
    pub fn new(spec: JointSpec, n_modules: usize) -> Self {
        let n = match spec.mode {
            JointMode::Steer | JointMode::Wheel => n_modules.max(1),
            _ => 1,
        };
        let speed_target = spec.max_vel;
        JointSim {
            spec,
            n,
            pos: vec![0.0; n],
            vel: vec![0.0; n],
            speed_target,
            pos_target: 0.0,
        }
    }

    /// Advance one physics tick. `dt` is the fixed step (1/PHYSICS_HZ).
    /// `chassis_lv_robot` is chassis linear velocity in the robot frame
    /// [x, z] — wheel joints project it onto each module's steer dir.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        idx: usize,
        cmd: &ActuatorFrame,
        dt: f32,
        intake_active: bool,
        shooter_request: bool,
        shooter_on: bool,
        dt_: &Drivetrain,
        chassis_lv_robot: [f32; 2],
        // resolved distance-keyed shot preset (flywheel frac + hood rad);
        // drives `shot_table` automation on shooter/hood drivers
        shot_preset: Option<ShotPreset>,
        // SWM velocity-lead for turret aim (rad, robot frame): the
        // barrel must aim `lead` off the commanded yaw so chassis motion
        // doesn't drag the shot off target (MoSim GetShootOnMoveTarget)
        turret_lead: f32,
    ) {
        let spec = &self.spec;
        let preset = shot_preset;
        let dv = spec.accel * dt;
        match spec.mode {
            JointMode::Spin => {
                // target velocity from the driver
                let mut target_vel = 0.0f32;
                match spec.driver {
                    JointDriver::Intake => {
                        if intake_active {
                            target_vel = spec.max_vel;
                        }
                    }
                    JointDriver::Shooter => {
                        // spin up while a shot is requested or the flywheel
                        // is enabled; spin down otherwise. A shot_table
                        // scales the target by distance (close shots need
                        // less RPM — MoSim alliance/pass-zone interpolation)
                        if shooter_request || shooter_on {
                            target_vel = spec.max_vel
                                * preset.map(|p| p.flywheel).unwrap_or(1.0);
                        }
                    }
                    JointDriver::Command => {
                        if let Some(jc) = cmd
                            .joints
                            .as_ref()
                            .and_then(|js| js.iter().find(|j| j.joint == idx))
                        {
                            target_vel = match jc.mode {
                                JointCmdMode::Velocity => jc.target,
                                JointCmdMode::OnOff => {
                                    if jc.target != 0.0 {
                                        jc.target.signum() * spec.max_vel
                                    } else {
                                        0.0
                                    }
                                }
                                // position command on a spin joint: treat as
                                // velocity toward that angle (unwrapped)
                                JointCmdMode::Position => {
                                    let err = jc.target - self.pos[0];
                                    err.clamp(-spec.max_vel, spec.max_vel)
                                }
                            };
                        }
                    }
                    // turret/hood are position-mode drivers; meaningless on spin
                    JointDriver::Turret | JointDriver::Hood => {}
                }
                // MoSim FlywheelBehaviour: asymmetric spin — spools down
                // ~2x faster than it spools up (1.2s up / 0.6s down)
                let rate = if target_vel.abs() > self.vel[0].abs() {
                    dv
                } else {
                    dv * 2.0
                };
                self.vel[0] += (target_vel - self.vel[0]).clamp(-rate, rate);
                self.speed_target = target_vel;
                self.pos[0] += self.vel[0] * dt;
            }
            JointMode::Position => {
                // turret/hood drivers: track the commanded angle from the
                // shot request; hold the last angle when none is sent
                if matches!(
                    spec.driver,
                    JointDriver::Turret | JointDriver::Hood
                ) {
                    // a shot_table on the hood joint overrides the
                    // commanded angle with the distance-interpolated one
                    // (MoSim: hood angle is automation, not a command)
                    let mut target = if spec.driver == JointDriver::Hood {
                        preset.map(|p| p.hood)
                    } else {
                        None
                    }
                    .or_else(|| {
                        cmd.shoot.map(|s| {
                            if spec.driver == JointDriver::Turret {
                                s.turret_yaw + turret_lead
                            } else {
                                s.hood_angle
                            }
                        })
                    })
                    .unwrap_or(self.pos[0]);
                    if let Some(l) = spec.limits {
                        target = target.clamp(l[0], l[1]);
                    }
                    self.pos_target = target;
                    let desired =
                        ((target - self.pos[0]) * 6.0).clamp(-spec.max_vel, spec.max_vel);
                    self.vel[0] += (desired - self.vel[0]).clamp(-dv, dv);
                    self.pos[0] = (self.pos[0] + self.vel[0] * dt)
                        .clamp(
                            spec.limits.map(|l| l[0]).unwrap_or(f32::NEG_INFINITY),
                            spec.limits.map(|l| l[1]).unwrap_or(f32::INFINITY),
                        );
                    return;
                }
                // target angle from the agent's position command (radians)
                let mut target = self.pos[0];
                if let Some(jc) = cmd
                    .joints
                    .as_ref()
                    .and_then(|js| js.iter().find(|j| j.joint == idx))
                {
                    match jc.mode {
                        JointCmdMode::Position => target = jc.target,
                        JointCmdMode::Velocity => {
                            // velocity command on a position joint:
                            // integrate the target ourselves
                            target = (self.pos[0] + jc.target * dt)
                                .clamp(
                                    spec.limits.map(|l| l[0]).unwrap_or(f32::NEG_INFINITY),
                                    spec.limits.map(|l| l[1]).unwrap_or(f32::INFINITY),
                                );
                        }
                        JointCmdMode::OnOff => {
                            target = if jc.target != 0.0 {
                                spec.limits.map(|l| l[1]).unwrap_or(self.pos[0] + 1.0)
                            } else {
                                spec.limits.map(|l| l[0]).unwrap_or(self.pos[0] - 1.0)
                            };
                        }
                    }
                }
                if let Some(l) = spec.limits {
                    target = target.clamp(l[0], l[1]);
                }
                // trapezoidal-ish servo: velocity toward target, capped
                let desired = ((target - self.pos[0]) * 6.0).clamp(-spec.max_vel, spec.max_vel);
                self.vel[0] += (desired - self.vel[0]).clamp(-dv, dv);
                self.pos[0] = (self.pos[0] + self.vel[0] * dt)
                    .clamp(
                        spec.limits.map(|l| l[0]).unwrap_or(f32::NEG_INFINITY),
                        spec.limits.map(|l| l[1]).unwrap_or(f32::INFINITY),
                    );
            }
            JointMode::Steer => {
                // one channel per module: servo toward that module's
                // commanded steer angle (robot frame, shortest path)
                for ch in 0..self.n {
                    let target = dt_.steer(ch).unwrap_or(0.0);
                    let mut err = target - self.pos[ch];
                    // wrap to [-pi, pi]
                    while err > std::f32::consts::PI {
                        err -= 2.0 * std::f32::consts::PI;
                    }
                    while err < -std::f32::consts::PI {
                        err += 2.0 * std::f32::consts::PI;
                    }
                    // proportional gain 8 like position mode: large errors
                    // command max_vel, small errors slow smoothly
                    let desired = (err * 8.0).clamp(-spec.max_vel, spec.max_vel);
                    self.vel[ch] += (desired - self.vel[ch]).clamp(-dv, dv);
                    self.pos[ch] += self.vel[ch] * dt;
                }
            }
            JointMode::Wheel => {
                // omega = module speed along steer dir / wheel radius;
                // module speed ≈ chassis speed projected on steer dir
                let r = if spec.wheel_radius > 1e-4 {
                    spec.wheel_radius
                } else {
                    0.05
                };
                for ch in 0..self.n {
                    if let Some(steer) = dt_.steer(ch) {
                        let dir = [steer.cos(), steer.sin()];
                        let v_mod = chassis_lv_robot[0] * dir[0] + chassis_lv_robot[1] * dir[1];
                        let omega = v_mod / r;
                        // wheel has no meaningful accel limit in this model —
                        // it follows ground speed; tiny smoothing only
                        self.vel[ch] += (omega - self.vel[ch]).clamp(-dv * 4.0, dv * 4.0);
                        self.pos[ch] += self.vel[ch] * dt;
                    }
                }
            }
        }
    }

    /// Is every shooter-driver joint at speed? Used to gate `shoot`
    /// commands — a real flywheel can't launch until it's at RPM.
    pub fn shooter_at_speed(&self) -> bool {
        if self.spec.driver != JointDriver::Shooter {
            return true;
        }
        let t = self.speed_target.abs();
        self.vel.iter().all(|v| v.abs() >= t * 0.8)
    }

    /// Hood settled on its target angle? (MoSim `hoodAngleLockTolerance`
    /// = 0.1 rad) — a real robot can't release fuel into a moving hood,
    /// so shot release gates on this like it gates on flywheel speed.
    pub fn hood_at_target(&self) -> bool {
        if self.spec.driver != JointDriver::Hood {
            return true;
        }
        (self.pos[0] - self.pos_target).abs() <= 0.1
    }

    /// Turret settled on the commanded aim? (MoSim
    /// `turretErrorTolerance` ≈ 2° = 0.035 rad) — same release gate.
    pub fn turret_at_target(&self) -> bool {
        if self.spec.driver != JointDriver::Turret {
            return true;
        }
        (self.pos[0] - self.pos_target).abs() <= 0.035
    }

    /// Ball-contact dip (MoSim FlywheelBehaviour): each launched piece
    /// costs ~8% of flywheel RPM; the velocity servo naturally recovers
    /// it, matching their 0.25 s recovery profile.
    pub fn ball_dip(&mut self) {
        if self.spec.driver == JointDriver::Shooter {
            for v in self.vel.iter_mut() {
                *v *= 0.92;
            }
        }
    }

    /// Telemetry snapshot for SensorFrame/viz.
    pub fn state(&self) -> crate::protocol::JointState {
        crate::protocol::JointState {
            name: self.spec.name.clone(),
            mode: format!("{:?}", self.spec.mode).to_lowercase(),
            pos: self.pos.clone(),
            vel: self.vel.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::archetype::JointSpec;
    use crate::drivetrain::Drivetrain;

    fn spin_spec(driver: JointDriver) -> JointSpec {
        JointSpec {
            name: "test".into(),
            mv: "x".into(),
            scope: None,
            mode: JointMode::Spin,
            driver,
            axis: None,
            pivot: None,
            max_vel: 10.0,
            accel: 20.0,
            limits: None,
            wheel_radius: 0.0,
            roller_pos: None,
            roller_radius: None,
            roller_axis: None,
            launch_radius: None,
            muzzle_pos: None,
            feed_time: None,
            feed_path: None,
            shot_table: None,
            on_arm: false,
        }
    }

    fn cmd() -> ActuatorFrame {
        ActuatorFrame {
            force: [0.0, 0.0],
            torque: 0.0,
            modules: None,
            intake_active: false,
            shoot: None,
            mech: None,
            joints: None,
            shooter_on: false,
        }
    }

    #[test]
    fn spin_respects_accel_limit() {
        let mut j = JointSim::new(spin_spec(JointDriver::Intake), 4);
        let dt = Drivetrain::swerve(60.0, 0.4);
        let c = cmd();
        // intake on: target vel = max_vel, but accel caps the ramp
        for _ in 0..10 {
            j.step(0, &c, 1.0 / 400.0, true, false, false, &dt, [0.0, 0.0], None, 0.0);
        }
        assert!(j.vel[0] > 0.0 && j.vel[0] < 10.0, "vel={}", j.vel[0]);
        assert!(j.pos[0] > 0.0);
        // after enough time it reaches target speed
        for _ in 0..400 {
            j.step(0, &c, 1.0 / 400.0, true, false, false, &dt, [0.0, 0.0], None, 0.0);
        }
        assert!((j.vel[0] - 10.0).abs() < 0.5, "vel={}", j.vel[0]);
        // intake off: spins down
        for _ in 0..400 {
            j.step(0, &c, 1.0 / 400.0, false, false, false, &dt, [0.0, 0.0], None, 0.0);
        }
        assert!(j.vel[0].abs() < 1.0, "vel={}", j.vel[0]);
    }

    #[test]
    fn shooter_gates_on_speed() {
        let mut j = JointSim::new(spin_spec(JointDriver::Shooter), 4);
        let dt = Drivetrain::swerve(60.0, 0.4);
        let mut c = cmd();
        // no request: not at speed even though vel target is 0
        assert!(!j.shooter_at_speed());
        // shoot request spins it up
        c.shoot = Some(crate::protocol::ShotCommand {
            launch_velocity: 10.0,
            hood_angle: 0.5,
            turret_yaw: 0.0,
            launch_height: 0.5,
        });
        for _ in 0..400 {
            j.step(0, &c, 1.0 / 400.0, false, true, false, &dt, [0.0, 0.0], None, 0.0);
        }
        assert!(j.shooter_at_speed(), "vel={}", j.vel[0]);
    }

    #[test]
    fn turret_tracks_aim_with_slew_lag() {
        let mut spec = spin_spec(JointDriver::Turret);
        spec.mode = JointMode::Position;
        spec.max_vel = 3.0;
        spec.accel = 6.0;
        let mut j = JointSim::new(spec, 4);
        let dt = Drivetrain::swerve(60.0, 0.4);
        let mut c = cmd();
        c.shoot = Some(crate::protocol::ShotCommand {
            launch_velocity: 10.0,
            hood_angle: 0.5,
            turret_yaw: 1.0,
            launch_height: 0.5,
        });
        // early ticks: moving toward 1.0 rad but lagging (real slew)
        for _ in 0..20 {
            j.step(0, &c, 1.0 / 400.0, false, true, false, &dt, [0.0, 0.0], None, 0.0);
        }
        assert!(j.pos[0] > 0.0 && j.pos[0] < 0.5, "pos={}", j.pos[0]);
        // settles on target
        for _ in 0..2000 {
            j.step(0, &c, 1.0 / 400.0, false, true, false, &dt, [0.0, 0.0], None, 0.0);
        }
        assert!((j.pos[0] - 1.0).abs() < 0.05, "pos={}", j.pos[0]);
        // no shot command: holds last angle
        c.shoot = None;
        for _ in 0..400 {
            j.step(0, &c, 1.0 / 400.0, false, false, false, &dt, [0.0, 0.0], None, 0.0);
        }
        assert!((j.pos[0] - 1.0).abs() < 0.05, "pos={}", j.pos[0]);
    }

    #[test]
    fn hood_tracks_angle_with_slew_lag() {
        let mut spec = spin_spec(JointDriver::Hood);
        spec.mode = JointMode::Position;
        spec.max_vel = 3.0;
        spec.accel = 6.0;
        let mut j = JointSim::new(spec, 4);
        let dt = Drivetrain::swerve(60.0, 0.4);
        let mut c = cmd();
        c.shoot = Some(crate::protocol::ShotCommand {
            launch_velocity: 10.0,
            hood_angle: 0.8,
            turret_yaw: 0.0,
            launch_height: 0.5,
        });
        for _ in 0..20 {
            j.step(0, &c, 1.0 / 400.0, false, true, false, &dt, [0.0, 0.0], None, 0.0);
        }
        assert!(j.pos[0] > 0.0 && j.pos[0] < 0.4, "pos={}", j.pos[0]);
        for _ in 0..2000 {
            j.step(0, &c, 1.0 / 400.0, false, true, false, &dt, [0.0, 0.0], None, 0.0);
        }
        assert!((j.pos[0] - 0.8).abs() < 0.05, "pos={}", j.pos[0]);
        // hood holds its angle when no shot is requested
        c.shoot = None;
        for _ in 0..400 {
            j.step(0, &c, 1.0 / 400.0, false, false, false, &dt, [0.0, 0.0], None, 0.0);
        }
        assert!((j.pos[0] - 0.8).abs() < 0.05, "pos={}", j.pos[0]);
    }

    #[test]
    fn steer_has_one_channel_per_module() {
        let mut spec = spin_spec(JointDriver::Command);
        spec.mode = JointMode::Steer;
        let mut j = JointSim::new(spec, 4);
        assert_eq!(j.pos.len(), 4);
        let mut dt = Drivetrain::swerve(60.0, 0.4);
        // command all modules to steer +90deg (robot +z)
        let mut c = cmd();
        c.modules = Some(vec![
            crate::drivetrain::ModuleCommand {
                steer: std::f32::consts::FRAC_PI_2,
                force: 0.0,
            };
            4
        ]);
        // drive the drivetrain once so last_steer updates — resolve needs
        // a chassis state; zero velocity is fine
        let kin = crate::drivetrain::ChassisState {
            yaw: 0.0,
            linvel: [0.0, 0.0],
            yaw_rate: 0.0,
            dt: 1.0 / 400.0,
        };
        dt.resolve(&c, &kin);
        for _ in 0..400 {
            j.step(0, &c, 1.0 / 400.0, false, false, false, &dt, [0.0, 0.0], None, 0.0);
        }
        for (i, p) in j.pos.iter().enumerate() {
            let err = (p - std::f32::consts::FRAC_PI_2)
                .abs()
                .min((p + std::f32::consts::FRAC_PI_2).abs());
            assert!(err < 0.2, "ch{i} pos={p}");
        }
    }

    #[test]
    fn shot_table_scales_flywheel_and_hood_by_distance() {
        use crate::archetype::{shot_preset_at, ShotPreset};
        let table = vec![
            ShotPreset { distance: 2.0, flywheel: 0.4, hood: 0.2 },
            ShotPreset { distance: 6.0, flywheel: 0.8, hood: 0.6 },
        ];
        // interpolation + clamping
        let near = shot_preset_at(&table, 1.0).unwrap();
        assert!((near.flywheel - 0.4).abs() < 1e-6 && (near.hood - 0.2).abs() < 1e-6);
        let mid = shot_preset_at(&table, 4.0).unwrap();
        assert!((mid.flywheel - 0.6).abs() < 1e-6 && (mid.hood - 0.4).abs() < 1e-6);
        let far = shot_preset_at(&table, 9.0).unwrap();
        assert!((far.flywheel - 0.8).abs() < 1e-6);

        // shooter joint: preset halves the flywheel target
        let mut spec = spin_spec(JointDriver::Shooter);
        spec.max_vel = 100.0;
        spec.accel = 100000.0; // fast servo so we settle quickly
        let mut j = JointSim::new(spec, 4);
        let dt = Drivetrain::swerve(60.0, 0.4);
        let mut c = cmd();
        c.shoot = Some(crate::protocol::ShotCommand {
            launch_velocity: 10.0,
            hood_angle: 0.9,
            turret_yaw: 0.0,
            launch_height: 0.5,
        });
        let preset = shot_preset_at(&table, 4.0).unwrap();
        for _ in 0..200 {
            j.step(0, &c, 1.0 / 400.0, false, true, false, &dt, [0.0, 0.0], Some(preset), 0.0);
        }
        assert!(
            (j.vel[0] - 60.0).abs() < 2.0,
            "flywheel should settle at 60% of max_vel, got {}",
            j.vel[0]
        );
        // without a preset it runs flat out
        for _ in 0..400 {
            j.step(0, &c, 1.0 / 400.0, false, true, false, &dt, [0.0, 0.0], None, 0.0);
        }
        assert!(j.vel[0] > 95.0, "vel={}", j.vel[0]);
    }

    #[test]
    fn shot_table_drives_hood_angle() {
        use crate::archetype::{shot_preset_at, ShotPreset};
        let table = vec![
            ShotPreset { distance: 2.0, flywheel: 0.4, hood: 0.2 },
            ShotPreset { distance: 6.0, flywheel: 0.8, hood: 0.6 },
        ];
        let mut spec = spin_spec(JointDriver::Hood);
        spec.mode = JointMode::Position;
        spec.max_vel = 10.0;
        spec.accel = 100000.0;
        spec.limits = Some([0.0, 1.2]);
        let mut j = JointSim::new(spec, 4);
        let dt = Drivetrain::swerve(60.0, 0.4);
        let mut c = cmd();
        // agent commands a wildly different hood angle — table wins
        c.shoot = Some(crate::protocol::ShotCommand {
            launch_velocity: 10.0,
            hood_angle: 1.1,
            turret_yaw: 0.0,
            launch_height: 0.5,
        });
        let preset = shot_preset_at(&table, 6.0).unwrap(); // hood = 0.6
        for _ in 0..200 {
            j.step(0, &c, 1.0 / 400.0, false, false, false, &dt, [0.0, 0.0], Some(preset), 0.0);
        }
        assert!(
            (j.pos[0] - 0.6).abs() < 0.05,
            "hood should settle at table angle 0.6, got {}",
            j.pos[0]
        );
    }

    #[test]
    fn hood_and_turret_gate_release_until_settled() {
        let dt = Drivetrain::swerve(60.0, 0.4);
        let mut c = cmd();
        c.shoot = Some(crate::protocol::ShotCommand {
            launch_velocity: 10.0,
            hood_angle: 0.6,
            turret_yaw: 0.5,
            launch_height: 0.5,
        });
        // hood joint: far from target right after the command lands
        let mut hspec = spin_spec(JointDriver::Hood);
        hspec.mode = JointMode::Position;
        hspec.max_vel = 2.0; // slow slew so the gate is observable
        hspec.accel = 4.0;
        let mut hood = JointSim::new(hspec, 4);
        hood.step(0, &c, 1.0 / 400.0, false, false, false, &dt, [0.0, 0.0], None, 0.0);
        assert!(!hood.hood_at_target(), "hood still slewing");
        for _ in 0..400 {
            hood.step(0, &c, 1.0 / 400.0, false, false, false, &dt, [0.0, 0.0], None, 0.0);
        }
        assert!(hood.hood_at_target(), "hood settled, pos={}", hood.pos[0]);
        // turret joint: same gate at the tighter 2° tolerance
        let mut tspec = spin_spec(JointDriver::Turret);
        tspec.mode = JointMode::Position;
        tspec.max_vel = 2.0;
        tspec.accel = 4.0;
        let mut turret = JointSim::new(tspec, 4);
        turret.step(0, &c, 1.0 / 400.0, false, false, false, &dt, [0.0, 0.0], None, 0.0);
        assert!(!turret.turret_at_target(), "turret still slewing");
        for _ in 0..400 {
            turret.step(0, &c, 1.0 / 400.0, false, false, false, &dt, [0.0, 0.0], None, 0.0);
        }
        assert!(turret.turret_at_target(), "turret settled, pos={}", turret.pos[0]);
        // non-positional drivers never gate
        let intake = JointSim::new(spin_spec(JointDriver::Intake), 4);
        assert!(intake.hood_at_target() && intake.turret_at_target());
    }

    #[test]
    fn turret_lead_offsets_aim_target() {
        let dt = Drivetrain::swerve(60.0, 0.4);
        let mut spec = spin_spec(JointDriver::Turret);
        spec.mode = JointMode::Position;
        spec.max_vel = 20.0;
        spec.accel = 100000.0;
        let mut j = JointSim::new(spec, 4);
        let mut c = cmd();
        c.shoot = Some(crate::protocol::ShotCommand {
            launch_velocity: 10.0,
            hood_angle: 0.5,
            turret_yaw: 0.4,
            launch_height: 0.5,
        });
        // SWM lead of +0.3 rad: turret should settle at 0.4 + 0.3 = 0.7
        for _ in 0..200 {
            j.step(0, &c, 1.0 / 400.0, false, false, false, &dt, [0.0, 0.0], None, 0.3);
        }
        assert!(
            (j.pos[0] - 0.7).abs() < 0.05,
            "turret should settle at yaw+lead=0.7, got {}",
            j.pos[0]
        );
    }
}

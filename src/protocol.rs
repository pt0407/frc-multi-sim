//! Agent <-> orchestrator protocol types (issue #5 / PLAN §0 versioning).
//!
//! Every handshake carries `protocol_version`; the orchestrator rejects
//! agents whose version differs rather than silently misbehaving.
use serde::{Deserialize, Serialize};

/// Bump on any breaking change to `AgentMsg`/`ServerMsg` or the
/// `SensorFrame`/`ActuatorFrame` payload contracts.
pub const PROTOCOL_VERSION: u32 = 1;

/// What the orchestrator sends to each agent every agent-tick.
/// Mirrors what a WPILib robot would observe: its own filtered state only.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorFrame {
    pub tick: u64,
    /// world-frame position [x, y, z] of the chassis center
    pub position: [f32; 3],
    /// quaternion [x, y, z, w]
    pub rotation: [f32; 4],
    /// world-frame linear velocity
    pub linvel: [f32; 3],
    /// world-frame angular velocity (gyro)
    pub angvel: [f32; 3],
    /// contact-normal force magnitudes felt this tick (bumper hits)
    pub contact_impulse: f32,
    /// game pieces currently held by this robot (issue #3)
    #[serde(default)]
    pub pieces_held: u32,
    /// this robot's alliance score (pieces scored into its hub, issue #3)
    #[serde(default)]
    pub score: u32,
    /// vision-style pose estimate (issue #7): latency-delayed, ~15-30 Hz,
    /// distance-dependent noise, occasional dropout. `None` when no estimate
    /// has been published yet or sensor noise is disabled. Backwards
    /// compatible: absent in serialized frames it deserializes to `None`.
    #[serde(default)]
    pub vision_pose: Option<[f32; 3]>,
}

/// What an agent sends back. In a real adapter these are per-wheel/module
/// voltages; for M0 we use chassis-level desired forces, which is equivalent
/// to a holonomic drivetrain's net force request.
/// Parameterized ballistic shot request (issue #3, FuelSim `launchFuel`
/// port). Fires one held piece: launch speed along (turret_yaw + chassis
/// yaw) at `hood_angle` above horizontal, from `launch_height` above the
/// chassis center, plus the chassis's current velocity.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ShotCommand {
    /// flywheel exit speed, m/s
    pub launch_velocity: f32,
    /// hood angle, radians (0 = horizontal, pi/2 = straight up)
    pub hood_angle: f32,
    /// robot-relative turret yaw, radians
    pub turret_yaw: f32,
    /// launch height above chassis center, m
    pub launch_height: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ActuatorFrame {
    /// desired force in world frame [x, z] (newtons)
    pub force: [f32; 2],
    /// desired yaw torque (N*m), + = counterclockwise from above
    pub torque: f32,
    /// optional per-module commands for steerable drivetrains
    /// (`drivetrain::ModuleCommand`). When `None`, the server's drivetrain
    /// compatibility shim allocates `force`/`torque` across the modules, so
    /// legacy agents keep working unchanged.
    #[serde(default)]
    pub modules: Option<Vec<crate::drivetrain::ModuleCommand>>,
    /// run the intake this tick: pieces entering the archetype's intake
    /// region are captured (issue #3). Defaults false so legacy agents
    /// don't intake unintentionally.
    #[serde(default)]
    pub intake_active: bool,
    /// optional one-shot ballistic launch; consumes one held piece (issue #3)
    #[serde(default)]
    pub shoot: Option<ShotCommand>,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum AgentMsg {
    Hello {
        name: String,
        robot_id: usize,
        /// PLAN §0: agents must present a matching protocol version.
        /// Missing field deserializes to 0 and is rejected.
        #[serde(default)]
        protocol_version: u32,
    },
    Actuator(ActuatorFrame),
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ServerMsg {
    Welcome {
        robot_id: usize,
        protocol_version: u32,
    },
    /// Sent instead of `Welcome` when the Hello is rejected (bad robot id,
    /// protocol version mismatch). The connection is closed afterwards.
    Error { message: String },
    Sensor(SensorFrame),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn actuator_roundtrip() {
        let m = AgentMsg::Actuator(ActuatorFrame {
            force: [1.0, 2.0],
            torque: 0.5,
            modules: None,
            intake_active: true,
            shoot: Some(ShotCommand {
                launch_velocity: 10.0,
                hood_angle: 0.7,
                turret_yaw: 0.0,
                launch_height: 0.5,
            }),
        });
        let s = serde_json::to_string(&m).unwrap();
        eprintln!("{}", s);
        let back: AgentMsg = serde_json::from_str(&s).unwrap();
        matches!(back, AgentMsg::Actuator(_));
    }

    #[test]
    fn hello_carries_version() {
        let h = AgentMsg::Hello {
            name: "x".into(),
            robot_id: 0,
            protocol_version: PROTOCOL_VERSION,
        };
        let s = serde_json::to_string(&h).unwrap();
        assert!(s.contains("protocol_version"));
        // a hello with no version field parses with version 0 (rejected)
        let legacy: AgentMsg =
            serde_json::from_str(r#"{"type":"Hello","name":"x","robot_id":0}"#).unwrap();
        match legacy {
            AgentMsg::Hello {
                protocol_version, ..
            } => assert_eq!(protocol_version, 0),
            _ => panic!(),
        }
    }
}

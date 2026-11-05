pub mod drivetrain;
pub mod archetype;
pub mod nt_server;
pub mod sim;

use serde::{Deserialize, Serialize};

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
}

/// What an agent sends back. In a real adapter these are per-wheel/module
/// voltages; for M0 we use chassis-level desired forces, which is equivalent
/// to a holonomic drivetrain's net force request.
#[derive(Debug, Clone, Serialize, Deserialize)]
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
    pub modules: Option<Vec<drivetrain::ModuleCommand>>,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum AgentMsg {
    Hello { name: String, robot_id: usize },
    Actuator(ActuatorFrame),
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ServerMsg {
    Welcome { robot_id: usize },
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
        });
        let s = serde_json::to_string(&m).unwrap();
        eprintln!("{}", s);
        let back: AgentMsg = serde_json::from_str(&s).unwrap();
        matches!(back, AgentMsg::Actuator(_));
    }
}
pub mod nt_client_min;

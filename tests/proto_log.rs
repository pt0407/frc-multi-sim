//! Protocol-log roundtrip tests (issue #31): envelopes written through
//! `ProtoLog` decode back as tick-stamped AgentMsg/ServerMsg traffic.
use frc_sim_m0::agents::proto_log::{self, Dir, ProtoLog};
use frc_sim_m0::{ActuatorFrame, AgentMsg, ServerMsg, PROTOCOL_VERSION};

#[test]
fn envelopes_roundtrip() {
    let dir = std::env::temp_dir();
    let path = dir.join(format!("proto_log_test_{}.jsonl", std::process::id()));
    {
        let log = ProtoLog::create(&path).unwrap();
        log.note_tick(42);
        log.log(
            0,
            Dir::In,
            &AgentMsg::Hello {
                name: "test".into(),
                robot_id: 0,
                protocol_version: PROTOCOL_VERSION,
            },
        );
        log.log(
            0,
            Dir::Out,
            &ServerMsg::Welcome {
                robot_id: 0,
                protocol_version: PROTOCOL_VERSION,
            },
        );
        log.log_at(
            48,
            0,
            Dir::In,
            &AgentMsg::Actuator(ActuatorFrame {
                force: [10.0, -5.0],
                torque: 1.5,
                modules: None,
                intake_active: true,
                shoot: None,
            }),
        );
        log.log_raw(0, Dir::In, "not json at all");
        log.flush();
    }
    let entries = proto_log::read_log(&path).unwrap();
    std::fs::remove_file(&path).ok();
    assert_eq!(entries.len(), 4);
    assert_eq!(entries[0].tick, 42);
    assert_eq!(entries[0].dir, Dir::In);
    // the msg payload must decode as the real protocol types
    let hello: AgentMsg = serde_json::from_value(entries[0].msg.clone()).unwrap();
    matches!(hello, AgentMsg::Hello { robot_id: 0, .. });
    let welcome: ServerMsg = serde_json::from_value(entries[1].msg.clone()).unwrap();
    matches!(welcome, ServerMsg::Welcome { robot_id: 0, .. });
    let act: AgentMsg = serde_json::from_value(entries[2].msg.clone()).unwrap();
    match act {
        AgentMsg::Actuator(a) => {
            assert_eq!(a.force, [10.0, -5.0]);
            assert!(a.intake_active);
        }
        _ => panic!(),
    }
    assert_eq!(entries[3].msg["raw"], "not json at all");
}

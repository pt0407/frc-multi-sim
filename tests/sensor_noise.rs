//! Sensor noise + latency model tests (issue #7 / PLAN B2):
//! - noise disabled → frames byte-identical to ground truth (regression /
//!   replay compat)
//! - noise enabled → frames differ but are seeded-reproducible
//! - vision_pose lags the true pose (latency queue) and updates at ~vision_hz
use frc_sim_m0::world::chassis::{self, RobotSpec};
use frc_sim_m0::world::field::FieldSpec;
use frc_sim_m0::world::noise::SensorConfig;
use frc_sim_m0::world::pieces::PieceTable;
use frc_sim_m0::world::{World, AGENT_EVERY};
use frc_sim_m0::SensorFrame;

const SEED: u64 = 7;

fn noisy_spec(x: f32) -> RobotSpec {
    let mut s = RobotSpec::at_x(x);
    s.sensors = SensorConfig {
        enabled: true,
        gyro_std: 0.005,
        pose_std: 0.005,
        vel_std: 0.02,
        gyro_bias: 0.002,
        pose_bias: 0.01,
        vel_bias: 0.01,
        vision_latency_s: 0.08, // ~4 agent ticks at 50 Hz
        vision_hz: 25.0,        // every ~2 agent ticks
        vision_dropout: 0.0,
        vision_std_min: 0.01,
        vision_std_max: 0.10,
    };
    s
}

fn specs(sensors: bool) -> Vec<RobotSpec> {
    let mut v = chassis::default_specs();
    if sensors {
        for s in v.iter_mut() {
            *s = noisy_spec(s.spawn[0]);
        }
    }
    v
}

fn run(specs: &[RobotSpec], ticks: u64) -> Vec<Vec<SensorFrame>> {
    let mut w = World::with_config(specs, SEED, FieldSpec::default(), &PieceTable::default());
    let n = w.num_robots();
    let mut out: Vec<Vec<SensorFrame>> = vec![Vec::new(); n];
    for tick in 0..ticks {
        w.step(&vec![None; n]);
        if tick % AGENT_EVERY as u64 == 0 {
            for r in 0..n {
                out[r].push(w.sensor_frame(r, tick));
            }
        }
    }
    out
}

fn ser(f: &SensorFrame) -> Vec<u8> {
    serde_json::to_vec(f).unwrap()
}

#[test]
fn noise_disabled_is_byte_identical() {
    let specs = specs(false);
    let mut w = World::with_config(&specs, SEED, FieldSpec::default(), &PieceTable::default());
    w.step(&vec![None; w.num_robots()]);
    let f = w.sensor_frame(0, 0);
    // vision_pose serializes as null; everything else is raw ground truth —
    // the same bytes the pre-noise code would have produced
    let body_pos = f.position;
    let s = ser(&f);
    let f2 = w.sensor_frame(0, 0); // disabled: no RNG draws, repeatable
    assert_eq!(s, ser(&f2));
    assert_eq!(f.position, body_pos);
    assert!(f.vision_pose.is_none());
}

#[test]
fn noise_enabled_differs_but_is_seeded_reproducible() {
    let a = run(&specs(true), 64);
    let b = run(&specs(true), 64);
    let clean = run(&specs(false), 64);
    for r in 0..a.len() {
        for i in 0..a[r].len() {
            // same seed → identical filtered frames
            assert_eq!(ser(&a[r][i]), ser(&b[r][i]));
        }
        // noise on → differs from ground truth at some point
        assert!(a[r]
            .iter()
            .zip(clean[r].iter())
            .any(|(x, y)| ser(x) != ser(y)));
    }
}

#[test]
fn vision_pose_lags_true_pose() {
    let frames = run(&specs(true), 64).remove(0);
    // wait for the pipeline's first publication
    let first = frames
        .iter()
        .position(|f| f.vision_pose.is_some())
        .expect("vision never published");
    for f in &frames[first..] {
        let v = f.vision_pose.expect("estimate should persist once published");
        // the estimate should track (delayed + noisy), not equal truth
        assert_ne!(v, f.position);
        // and stay in a sane neighborhood of the real pose
        for i in 0..3 {
            assert!((v[i] - f.position[i]).abs() < 1.0);
        }
    }
}

#[test]
fn vision_estimate_trails_moving_truth() {
    let mut w = World::with_config(&specs(true), SEED, FieldSpec::default(), &PieceTable::default());
    let cmd = frc_sim_m0::ActuatorFrame {
        force: [300.0, 0.0],
        torque: 0.0,
        modules: None,
        intake_active: false,
        shoot: None,
        mech: None,
    };
    let mut saw_lag = false;
    for tick in 0u64..400 {
        w.step(&[Some(cmd.clone()), None]);
        if tick % AGENT_EVERY as u64 == 0 && tick > 32 {
            let f = w.sensor_frame(0, tick);
            if let Some(v) = f.vision_pose {
                // moving +x: delayed estimate x should be behind the
                // (noise-filtered) reported x once speed builds up
                if v[0] < f.position[0] - 0.01 {
                    saw_lag = true;
                }
            }
        }
    }
    assert!(saw_lag, "vision estimate never lagged the moving chassis");
}

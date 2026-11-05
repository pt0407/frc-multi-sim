//! Mock WPILib robot process: hosts its own NT4 server (same topology as a
//! real robot), publishes actuator command topics, polls sensor topics the
//! orchestrator publishes. The orchestrator connects to this as an NT4
//! client — exactly like a WPILib sim would expose.
use clap::Parser;
use frc_sim_m0::nt_server::NtServer;
use rmpv::Value;
use std::time::Duration;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    port: u16,
    #[arg(long, num_args = 2, allow_hyphen_values = true)]
    target: Vec<f64>,
    #[arg(long, default_value_t = 1200.0)]
    max_force: f64,
}

#[tokio::main]
async fn main() -> std::io::Result<()> {
    let args = Args::parse();
    let nt = NtServer::start(&format!("127.0.0.1:{}", args.port)).await?;
    println!("robot NT4 server on :{}", args.port);

    // actuator topics (robot -> world); type id 3 = double
    nt.publish("/sim/cmd_fx", 3, Value::from(0.0f64));
    nt.publish("/sim/cmd_fz", 3, Value::from(0.0f64));
    nt.publish("/sim/cmd_torque", 3, Value::from(0.0f64));

    let [tx, tz] = [args.target[0], args.target[1]];
    let mut tick = 0u64;
    loop {
        tokio::time::sleep(Duration::from_millis(20)).await;
        let (Some(px), Some(pz), Some(vx), Some(vz)) = (
            nt.get_f64("/sim/pose_x"),
            nt.get_f64("/sim/pose_z"),
            nt.get_f64("/sim/vel_x"),
            nt.get_f64("/sim/vel_z"),
        ) else {
            continue;
        };
        let contact = nt.get_f64("/sim/contact").unwrap_or(0.0);

        let ex = tx - px;
        let ez = tz - pz;
        let mut fx = 300.0 * ex - 60.0 * vx;
        let mut fz = 300.0 * ez - 60.0 * vz;
        let mag = (fx * fx + fz * fz).sqrt();
        if mag > args.max_force {
            fx *= args.max_force / mag;
            fz *= args.max_force / mag;
        }

        if contact > 0.0 {
            println!("robot(:{}) tick {}: CONTACT pos=({:.2},{:.2})", args.port, tick, px, pz);
        }
        tick += 1;

        nt.set("/sim/cmd_fx", Value::from(fx));
        nt.set("/sim/cmd_fz", Value::from(fz));
    }
}

// WPILib-side adapter (issue #6): connects the real robot's motor/output
// logic to the frc-sim NT4 bridge. Your robot code calls robotPeriodic()
// each loop; the adapter writes /sim/cmd_fx|fz|torque and exposes sensor
// state via getSensorState(). Requires a WPILib-NT4-capable robot project.
package frc.sim;

public class SimAdapter {
    private final int robotId;
    // TODO: wire real NT4 topics once the WPILib sim is running —
    // for now the shape matches what src/agents/nt_bridge.rs expects.

    public SimAdapter(int robotId) {
        this.robotId = robotId;
    }

    /// Called each robot loop. Override to drive your mechanisms.
    public void robotPeriodic() {
        double[] st = getSensorState();
        double[] cmd = decide(st);
        setActuator(cmd[0], cmd[1], cmd[2]);
    }

    /// Replace with real NT4 reads: /sim/pose_x, /sim/pose_z,
    /// /sim/vel_x, /sim/vel_z, /sim/contact — flattened doubles.
    public double[] getSensorState() {
        return new double[5];
    }

    /// Replace with real NT4 writes: /sim/cmd_fx, /sim/cmd_fz,
    /// /sim/cmd_torque (doubles, newtons / N*m).
    public void setActuator(double fx, double fz, double torque) {}

    /// Your control policy: sensor state -> [fx, fz, torque]. This stub
    /// drives a simple patrol so the adapter does something real.
    private double[] decide(double[] s) {
        double x = s[0], z = s[1];
        // drive toward field center; real code reads waypoints/odometry
        double k = 600.0;
        return new double[] { -k * x, -k * z, 0.0 };
    }
}

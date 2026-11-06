// WPILib-side adapter (issue #6): bridges robot code to the sim's NT4
// contract. Reads /sim/pose_x|z, /sim/vel_x|z, /sim/contact and writes
// /sim/cmd_fx|fz|torque — the sim's nt_bridge turns those into physics.
package frc.sim;

import edu.wpi.first.networktables.*;

public class SimAdapter {
    private final NetworkTableInstance inst;
    private final DoubleSubscriber poseX, poseZ, velX, velZ, contact;
    private final DoublePublisher cmdFx, cmdFz, cmdTorque;

    public SimAdapter(int robotId) {
        inst = NetworkTableInstance.getDefault();
        // robot code hosts the NT4 server (default port 5810); the sim's
        // nt_bridge connects to us (issue #6)
        inst.startServer("nt.json");

        NetworkTable t = inst.getTable("/sim");
        poseX = t.getDoubleTopic("pose_x").subscribe(0.0);
        poseZ = t.getDoubleTopic("pose_z").subscribe(0.0);
        velX = t.getDoubleTopic("vel_x").subscribe(0.0);
        velZ = t.getDoubleTopic("vel_z").subscribe(0.0);
        contact = t.getDoubleTopic("contact").subscribe(0.0);
        cmdFx = t.getDoubleTopic("cmd_fx").publish();
        cmdFz = t.getDoubleTopic("cmd_fz").publish();
        cmdTorque = t.getDoubleTopic("cmd_torque").publish();
    }

    /// Call each robot loop. Your control code goes in decide().
    public void robotPeriodic() {
        double[] s = getSensorState();
        double[] c = decide(s);
        cmdFx.set(c[0]); cmdFz.set(c[1]); cmdTorque.set(c[2]);
    }

    public double[] getSensorState() {
        return new double[] {
            poseX.get(), poseZ.get(), velX.get(), velZ.get(), contact.get()
        };
    }

    /// Stub controller: drives toward field center. Replace with real
    /// robot logic — odometry, waypoints, mechanism commands.
    private double[] decide(double[] s) {
        double k = 600.0;
        return new double[] { -k * s[0], -k * s[1], 0.0 };
    }
}

// Runnable adapter loop (issue #6): runs robotPeriodic() at 50Hz to
// match the sim's agent-tick rate. Launch with `gradle run` in a robot
// project, or `java -cp build/libs/sim-adapter.jar frc.sim.SimRobot <id>`.
package frc.sim;

public class SimRobot {
    public static void main(String[] args) throws InterruptedException {
        int id = args.length > 0 ? Integer.parseInt(args[0]) : 0;
        SimAdapter adapter = new SimAdapter(id);
        while (true) {
            adapter.robotPeriodic();
            Thread.sleep(20); // 50 Hz — sim agent tick rate
        }
    }
}

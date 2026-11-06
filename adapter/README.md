# WPILib-side adapter (issue #6)

`SimAdapter` connects robot code to the sim over **real NT4** — the same
protocol a real robot's NetworkTables uses:

- **In** (sim -> robot): `/sim/pose_x`, `/sim/pose_z`, `/sim/vel_x`,
  `/sim/vel_z`, `/sim/contact`
- **Out** (robot -> sim): `/sim/cmd_fx`, `/sim/cmd_fz`, `/sim/cmd_torque`

## Wiring

```java
SimAdapter adapter = new SimAdapter(robotId);  // connects to 127.0.0.1:5810+id
// in Robot.robotPeriodic():
adapter.robotPeriodic();
```

Override `decide()` (or read sensor state + publish directly) for your
robot's control logic. The sim's `nt_bridge` translates `cmd_*` topics
into physics; sensor topics are published each 50 Hz tick.

## Build

```
cd adapter/java && gradle build
```

Needs JDK 17+ (`brew install openjdk`) + Gradle (`brew install gradle`).
`ntcore-java` is vendored in `libs/` — the WPILib maven repo cert is
broken on some networks, so the jar is committed directly.

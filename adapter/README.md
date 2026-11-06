# WPILib-side adapter (issue #6)

`SimAdapter` connects robot code to the sim over **real NT4** — the same
protocol a real robot's NetworkTables uses:

- **In** (sim -> robot): `/sim/pose_x`, `/sim/pose_z`, `/sim/vel_x`,
  `/sim/vel_z`, `/sim/contact`
- **Out** (robot -> sim): `/sim/cmd_fx`, `/sim/cmd_fz`, `/sim/cmd_torque`

## Wiring

```java
SimAdapter adapter = new SimAdapter(robotId);  // hosts NT4 server on :5810
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

## Integration (real usage)

`SimAdapter` is a drop-in class for a **real WPILib robot project** — it
does not run standalone, because WPILib JNI natives resolve through the
vendordep system, not loose jars:

1. Copy `SimAdapter.java` into your robot project's `src/main/java/frc/sim/`
2. Add the WPILib vendordep (your robot already has `WPILibNewCommands.json`
   or equivalent — ntcore comes with it)
3. Call `adapter.robotPeriodic()` in `Robot.robotPeriodic()`

The Gradle project here (`adapter/java`) compiles the class against the
real ntcore API — it's a verification scaffold, not a standalone runtime.

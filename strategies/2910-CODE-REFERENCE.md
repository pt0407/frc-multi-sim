# Team 2910 real 2026 code — integration reference

Source: https://github.com/FRCTeam2910/2026CompetitionRobot-Public (Java/WPILib, MIT)
Cloned to /tmp/p2910 during analysis (re-clone if gone).

## What exists
- `AutoFactory.java` — real autos: createLeftNeutralZoneAuto / RightNeutralZone,
  Depot, Trench variants. Ported to `autos/2910-realauto.json` (scores ~25 vs
  ~73 for tuned script — needs per-waypoint shot tuning + better goto routing).
- `FieldConstants.java` — Pose2d waypoints (meters, origin
  kBlueAllianceWallRightSide). Transform used: ox = tx - 8.27, oz = ty - 4.03,
  clamp |z|<=3.2. May need axis re-check.
- Maple-Sim architecture: `SimulatedArena`, `SimulatedRobotState`,
  `MapleSimSwerveDrivetrain`, CTRE `TalonFXSimState`/`CANcoderSimState`/
  `Pigeon2SimState` injection — this is their DESKTOP sim, not our Rust world.
- `SuperStructure.WantedState.AUTO_SCORE_STATIONARY` — real scoring state.

## Bridge plan (not yet built)
2910's code never touches NetworkTables directly for control — it drives
simulated CTRE devices. Two viable paths:
1. Java adapter: implement our IO interfaces (SwerveIOSim etc.) to publish
   ActuatorFrame-equivalent over WS/NT4 to the Rust server. Medium effort,
   keeps their code unmodified.
2. NT4 bridge: run WPILib NetworkTables client (`nt_client` exists in nt_probe)
   that maps their NT outputs to ActuatorFrame and injects SensorFrame as
   simulated device states. Needs a Java-side NT IO shim anyway — same work.

Key mapping needed: swerve module states -> drive force/velocity commands;
intake/shooter WantedState -> intake_active + shot params; pose -> odometry.

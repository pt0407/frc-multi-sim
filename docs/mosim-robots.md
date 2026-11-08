# MoSim robot mechanism constants (decompiled)

Source: `/tmp/mosim-src/Assembly-CSharp/Assembly-CSharp.decompiled.cs`,
namespaces `Prefabs.Games.Rebuilt.Robots.*`. Inline field defaults only —
`[SerializeField]` values without defaults live in the encrypted robot
packs (`ProtectedRobots/*.enc`) and are not recoverable.

## Shared infrastructure

- `PidConstants` defaults: `kP=0.25, kI=0, kD=1e-5, Max=2.25, Isaturation=1`
  (`RobotFramework.decompiled.cs`)
- `PIDController` (fixed-point): default gains scale 6 → p=1.5, i=0, d=0,
  out [-1, 1]
- `Interpolation`/`RebuiltInterpolation`: distance→value lerp table
  (`DistanceValue[]{distance,value}`, sorted + linear interp, clamped at
  ends). Flywheel RPM and hood angle are looked up by **distance to the
  current aim node** — separate maps for alliance-zone shot, pass shot,
  and neutral zone. Aim nodes are scene `AutoAngleNodes` transforms.
- Common tolerances across robots: `aimTolerance≈2°`,
  `hoodAngleTolerance=0.5°`, `hoodAngleLockTolerance=0.1°`,
  `feedReadyDebounceTime=0.06s`, `hoodAngleLimits=[0°,35°]`
- `shootingDriveMultiplier≈0.55` (drive slows while shooting),
  `passingDriveMultiplier 0.5–0.85`, `enableShootOnMove` common
- Flywheel/hood "at speed" model already in `mosim-physics.md`
  (spinUp 1.2 s / spinDown 0.6 s, 8% ball dip, 2% tolerance)

## Per-robot configs

### Simbotics (Simbotics)
Roller groups: intake, indexer, uptake, shooter, hood, final + intake
slide. Joints: intake, kicker (`kickerStowAngle=-10°`), hood.
`intakeBringinDistance=6`, `intakePidMaxDefault=0.55`,
`agitateMaxDivider=2.5`, `agitateSpeed=0.55`,
`shooterRpmDeadband=25`, `hoodRpmReduction=1.5`,
`manualShotRpm=3000`, `manualHoodAngle=5°`, `shootTimeOfFlight=1.2s`.

### CitrusCircuits
Joints: verticalHopper (Y axis), horizontalHopper (Z), intakePivot,
hoodPivot. Roller groups: shooter, hood, floor, uptake, intake.
`intakeRpm=2000`, `intakeBringupAngle=-40°`, `intakeBringupDelay=0.15s`,
`intakeAgitateSpeed=0.5`, `hopperBringdownDelay=0.1s`,
`uptakeRpm=2000`, `floorRpm=2000`, `manualShotRpm=2000`,
`manualHoodAngle=25°`, `hoodRpmReduction=1.5`, `uptakeRpmReduction=1.5`,
`floorRpmReduction=1.5`, `hopperWantsOpen`/`shooterReadyLatched` feed latch.

### CompetitiveConcept (CC)
Joints: feederRollers, intakeJoint, hopperJoint, hoodJoint, climber.
Rollers: intake, indexer, shooter. PIDs for all + climber.
`intakeAngle`, `extendedDistance`, `_deployClimb` — climbing mechanism.

### Overcharge (9483)
Turret + hood + intake + spindexer + feeder + shooter rollers.
`spindexerRpm=2000`, `feederRpm=2000`, `turretErrorTolerance=2°`,
`manualShotRpm=2000`, `manualHoodAngle=25°`,
`turretSteerFeedForwardConstant=3.5`, `groundYawCorrectionGain=0.8`,
`hubTimeOfFlight=1.2s`, `passTimeOfFlight=1.2s`,
`enableTranslationFeedForward`, `swmGroundCorrectionSmoothingTime=0.03s`,
`enablePassWallBias` + `passFieldWallZLimits=[-4.105,4.105]`,
`enableShootOnMove`, `shootingBrakeTime=0.3s`.
Shooting-Wheel-Model (SWM) telemetry: predicted velocity, ground yaw
correction, feed-forward — turret aims with chassis-velocity
compensation.

### Hightide (4414)
Turret + dyeRotor + intake + kickerBar + hood joints.
Rollers: intake, uptake, indexer, shooter.
`feederSpeedMultiplier=2.3`, `uptakeRpm=2000`, `indexerRpm=2000`,
`turretErrorTolerance=2°`, `manualShotRpm=2000`, `manualHoodAngle=25°`,
`turretSteerFeedForwardConstant=6`, `groundYawCorrectionGain=0.8`,
`hubTimeOfFlight=1.2s`, `velocitySmoothingTime=0.1s`,
`ballisticTofWhenTilted`, trench avoidance (`trenchTurretMargin=0.165`,
`trenchShooterRpm=-2000` — reverses shooter near trench!),
tilt compensation (`maxTilt=35°`, `gain=1`,
`hoodZeroExitElevation=88°`, `crossingTilt=12°`, `crossingSpeed=1.75`,
`crossingHold=1s`), kicker deploy (`kickerDeployStartMeters=0.05`,
`kickerDeployDelay=0.15s`), beam-sensor intake detection
(`detectionCooldown=0.15s`).

### ReBlitz
Hopper + intakePivot + hoodPivot joints; rollers: shooter, hood, floor,
uptake, intake, kicker. `HopperTargetInches=-12`,
`HopperLockPositionMeters=-0.299`, `intakeRpm=2000`,
`intakeBringupAngle`, `intakeBringupDelay=0.15s`,
`uptakeRpm=2000`, `floorRpm=2000`,
`hoodAngleLockTolerance=0.1`, `manualShotRpm` unset (prefab),
`hoodRpmReduction=1.5`, `scoringAimTolerance=4°`,
`passingAimTolerance=15°`, `hubNoShootingDist=1.5m`,
`bumpAimPoint=(3.64,1.675)`, `passLandingX=5.91`,
`passLandingMaxZ=3.5`, CAN-range hopper fullness debounce `0.25s`,
`_allianceSign` — pass-arc trajectory planning.

### Blitz
Dual intake pivots (left/right) + hoodPivot; rollers: shooter, hood,
indexer, dead, left/right intake, sideFeeder.
`intakeRpm=2000`, `indexerRPM=2000`, `manualShotRpm=5000` (highest),
`manualHoodAngle=25°`, `hoodRpmReduction=1.25`,
`intakeAutoDeployMode=Both`, `autoDeployVelocityThreshold=0.5`,
`autoDeployDotThreshold=0.5`, `visionLayerName="GamePiece1"` (intake
vision!), `visionArraySize=8`, `visionSideSwitchAdvantage=10`,
`passAimTolerance=7°`, `feedHoodTolerance=2°`,
`feedMaxAimErrorRate=30°/s`, trench avoidance points,
passage-zone snap (`passageZoneMin=(7.6,-0.105)`,
`passageZoneMax=(7.8,0.724)`), pass arc params
(`passArcBaseX=5.91`, `passArcCurveDepth=0.6`, `passArcMaxZ=3.2`,
`passArcHeight=1.938`), `hubObstacleRadius=1.85`,
`hubObstacleHalfSize=(1.5,1.5)`, SWM settings
(`swmDirectionChangeResponse=0.65`, `swmOutlierAccelerationThreshold=80`).

### RatchetRockers (1706)
E-chain assisted: turretsWrapper, intakeJoint, hopperJoint, climber;
left/right indexer+feeder rollers + left/right E-chains (literal
energy-chain runs!). Dual turrets (`leftTurretTransform`,
`rightTurretTransform`), dual interpolation sets (AZ/NZ ×
flywheel/hood/tof per side).
`manualShotRpm=300` (low — likely scaled by roller ratio),
`agitationAngle=30°`, `intakeRpm=2000`, `intakeWobbleDelay=1.25s`,
`turretAngleTolerance=15°` (AZ) / `25°` (NZ),
`hoodAtTargetTolerance=0.3` / `0.8` NZ, `hoodRollerReduction=0.5`,
`enableShootOnMove`, drive multipliers `0.12/0.2` shooting,
`0.6/0.3` passing.

## Realism takeaways for our sim

- **Distance-keyed shot tables are the big one** — every robot lerps
  flywheel RPM and hood angle by distance-to-target instead of fixed
  values. Implemented: `JointSpec.shot_table` + `shot_preset_at`
  (`archetype.rs`), resolved per-tick from distance-to-nearest-hub in
  `World::step`; shooter spin target scales by `flywheel` fraction and
  the hood joint tracks the interpolated angle (command ignored while a
  table is present). 2910 authored with a 5-row table.
- Hood range is universally **0–35°**; aim tolerance ~2°; feed latch
  debounces 60 ms — cheap rules to mirror.
- Roller RPMs cluster at **2000 RPM** intake/uptake/floor/indexer;
  shooter RPM is per-robot (2000–5000, turret robots lower).
- Turret robots compensate aim for chassis velocity (SWM feed-forward,
  ground-yaw correction ~0.8) — our turret is pure position servo.
- Hightide reverses shooter RPM (-2000) in the trench zone —
  mechanism-state-dependent shot behavior exists in the real meta.
- Pass shots are a first-class mode: separate interpolation maps,
  landing-point constraints, wall bias — our `ShotCommand` has no
  pass/score mode split.

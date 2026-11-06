# WPILib-side adapter (issue #6)

Drop `SimAdapter` into your robot project and call `robotPeriodic()`
each loop. The adapter writes `/sim/cmd_fx|fz|torque` — the sim's
`nt_bridge` translates those into physics — and reads
`/sim/pose_x|z`, `/sim/vel_x|z`, `/sim/contact` back.

Build: JDK 17+ + Gradle. The real NT4 integration layer goes in
`getSensorState()` / `setActuator()` — the contract is already there.

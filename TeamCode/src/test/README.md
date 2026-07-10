# TeamCode JVM unit tests

Pure-JVM unit tests for the subsystems, commands, and math utilities. They run on your dev
machine (no robot, no emulator) via Android's `testDebugUnitTest` task.

## Running them

```bash
# JDK matters: Gradle 8.9 + Kotlin 2.0 cannot run on JDK 22+. If you see
#   "Unsupported class file major version 68"
# your default `java` is too new (68 = JDK 24). Point Gradle at a JDK 17-21 --
# Android Studio's bundled JBR is 21 and works:
export JAVA_HOME="$HOME/Applications/Android Studio.app/Contents/jbr/Contents/Home"

./gradlew :TeamCode:testDebugUnitTest            # run all
./gradlew :TeamCode:testDebugUnitTest --tests "*TurretTest"   # one class
./gradlew :TeamCode:testDebugUnitTest --rerun-tasks           # ignore the build cache
```

HTML report: `TeamCode/build/reports/tests/testDebugUnitTest/index.html`.
(Running inside Android Studio uses its JBR automatically, so this Just Works there.)

## How it's built

- **No mocking framework.** Hardware is faked by hand in `testfakes/` (`FakeDcMotorEx`,
  `FakeServo`, `FakeAnalogInput`, `FakeDigitalChannel`). The fakes back only the state the code
  actually uses and `throw NotImplementedError()` for everything else, so an unexpected call fails
  loudly. This keeps the suite robust across JDKs (no byte-buddy/JDK coupling).
- Subsystems that read `Robot.telemetry` are given an empty `MultipleTelemetry()`; command tests
  install faked subsystems into `Robot.Subsystems`.
- **Shared static state:** the suite writes `Robot.telemetry` and the `Turret` companion tunables
  (`kP`, `MIN_ANGLE`, `SHOOT_SPEED`, …). Tests reset these in `@Before`, so run order doesn't matter
  today. A new test that mutates a shared static **must reset it in its own `@Before`**, or it will
  introduce order-dependence that only shows up when the suite is resharded.

## Testing philosophy (important for maintainers)

This is a **regression / characterization** suite: it pins current behavior so future edits can be
validated against it. Two rules:

1. **Contracts, not tuning magic numbers.** Empirical values (regression coefficients, PID gains,
   servo positions, `SCALAR_PER_INCH`, `SHOOT_SPEED`) are asserted only through their *shape*
   (bounds, monotonicity, clamping) or as clearly-labeled characterization values. If you re-tune a
   constant on the robot and a characterization test fails, update the test on purpose.
2. **The CRT/vernier turret decode test proves algebra, not hardware.** `TurretTest` sweeps a
   forward model derived from the tooth counts (137t / 12t / 13t) and checks the decode recovers
   the true angle. It does **not** validate gear-mesh direction or the `ENCODER_*_ZERO_OFFSET_DEG`
   calibration -- those remain on-robot tasks.

## Observations surfaced while writing these (not changed here)

- `Launcher.distanceToScalar` returns 0 for every input because `SCALAR_PER_INCH`/`BASE_SCALAR` are
  still `0.0`. `LaunchByDistance` is therefore inert until those are tuned.
- `Launcher.Regressions` far-field branch (distance > 115) returns the LUT value **without**
  re-clamping to the near-field `POWER_*`/`HOOD_*` bounds, so long-range outputs intentionally sit
  outside those bounds.
- `percentDifference(a, b)` divides by zero when `a + b == 0` (NaN when both are 0). Current callers
  handle this safely, but the helper itself is unguarded.
- **Probable bug (latent, currently masked):** `BaseTemplate.targetGoalPose` defaults to
  `Vector(72.0, 72.0)`, but pedro's `Vector(Double, Double)` constructor is **polar**
  `(magnitude, theta-radians)` -- so the default aim point is roughly `(-70, 19)`, not the
  field-center `(72, 72)` that was clearly intended. It's masked today: `DriverControlled` is the
  only OpMode that turns on `goalLock`, and it calls `setTargetPose(goalPose)` in `initialize()`
  first, so the bogus default only feeds telemetry and never drives the robot. It becomes a live
  drivetrain-aiming bug the moment any OpMode engages `goalLock` without first calling
  `setTargetPose`. Intended-cartesian fix: `Vector(Pose(72.0, 72.0))`.

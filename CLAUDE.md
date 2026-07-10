# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Refactoring Guidelines
- IMPORTANT: Always run relevant tests after any code changes
- Prefer incremental changes over large rewrites
- When extracting methods, preserve original function signatures as wrappers initially
- Document any behavior changes in commit messages

## Hard Rules
- Always run tests before reporting a task as complete

## What this is

An FTC (FIRST Tech Challenge) robot control repo for the 2025-26 "Decode" season. Two Gradle modules:
- `FtcRobotController` — vendor SDK app (do not modify unless absolutely necessary; it's the stock FTC app shell).
- `TeamCode` — all team code, under `org.firstinspires.ftc.teamcode`. This is where nearly all work happens.

There is no CI. Behavior is validated by a JVM unit-test suite (`TeamCode/src/test/`, run via `./gradlew :TeamCode:testDebugUnitTest` — see `TeamCode/src/test/README.md`) and by deploying to the physical Control/Driver Hub.

## Build commands

Run from the repo root (uses the Gradle wrapper):
- `./gradlew :TeamCode:assembleDebug` — compile and build the APK (fastest way to check that everything compiles).
- `./gradlew :TeamCode:compileDebugSources` — compile only, faster feedback loop when checking for Kotlin/Java errors.
- `./gradlew clean` — clean build outputs.
- `./gradlew :TeamCode:installDebug` — build and install onto a connected/ADB-linked Driver Hub.

JVM unit tests live in `TeamCode/src/test/` (`./gradlew :TeamCode:testDebugUnitTest`; requires JDK 17–21 — Android Studio's JBR works). `RPMTest.kt` in `opmode/debug` is an on-robot debug OpMode, not a JUnit test.

## Architecture

### Read/Update/Write loop (`ISubsystem`)

Every subsystem (`hardware/subsystem/*.kt`) and `Robot` itself implements `ISubsystem` (`hardware/ISubsystem.kt`), which enforces a strict lifecycle each control loop iteration, run in this order from the OpMode's `loop()`:

1. **`reset()`** — re-initialize hardware/state (called once, equivalent to init).
2. **`read()`** — pull all sensor/hardware state into fields. Never read hardware directly outside this phase.
3. **`update()`** — pure computation based on the fields read in step 2 (PID, target calculation, telemetry). Never touch hardware here.
4. **`write()`** — push computed state out to motors/servos. The only place hardware is written.

This ordering exists so that a given loop iteration never mixes stale and fresh hardware state. When adding subsystem logic, put it in the correct phase — don't read hardware inside `update()`/`write()`, and don't compute non-trivial logic inside `read()`/`write()`.

`Robot` (`hardware/Robot.kt`) is a singleton object that owns all hardware handles (`Motors`, `Servos`, `DigitalDevices` nested objects), the `Subsystems` registry, the Pedro Pathing `Follower`, and the FTCLib `CommandScheduler`. `Robot.init()` wires hardware map entries (named in `hardware/Names.kt`) to subsystem instances and registers them with the scheduler. `Robot.read()/update()/write()` fan out to every registered subsystem each loop.

### OpMode templates

- `opmode/template/BaseTemplate.kt` — abstract base for all OpModes. Owns the FTCLib command scheduler tick, gamepad wrappers, loop-time telemetry, and turret/drivetrain goal-lock (`face()`/PIDFController) logic. Subclasses implement `initialize()` (once, in `init()`) and `cycle()` (once per loop, before scheduler run).
- `opmode/template/AutoTemplate.kt` — extends `BaseTemplate` for autonomous OpModes; sets `Globals.AUTO = true` and a starting `Pose` for the Pedro Pathing follower.
- TeleOp OpModes (e.g. `opmode/DriverControlled.kt`) extend `BaseTemplate` directly; Autonomous OpModes (`opmode/auto/*.kt`) extend `AutoTemplate`.

### Commands (FTCLib command-based)

`command/CommandTemplate.kt` is the shared base (`CommandBase` + `Command`) for all commands, taking subsystem `ISubsystem` requirements as varargs. Commands live under `command/<subsystem>/` (e.g. `command/intake/IntakeIn.kt`, `command/launcher/FeedLauncherArtifacts.kt`, `command/turret/AimAtGoal.kt`) and are composed into `SequentialCommandGroup`/`ParallelCommandGroup`/`ConditionalCommand` trees, both in TeleOp button bindings and in autonomous routines.

### Autonomous routines and paths

Autonomous OpModes (`opmode/auto/*.kt`) build a big `SequentialCommandGroup`/`ParallelCommandGroup` tree combining `PedroCommand` (wraps a Pedro Pathing `PathChain`, in `command/auto/PedroCommand.kt`) with intake/launcher commands and `WaitCommand`s. The actual path geometry lives in separate files under `paths/` (Java, generated/tuned via Pedro Pathing tooling, e.g. `paths/BlueClose12Paths.java`), matched 1:1 with an OpMode of the same base name in `opmode/auto/`. Several auto files have `Old` variants kept alongside — treat `Old` files as historical/reference unless told otherwise, don't delete without asking.

Follower configuration (drivetrain constants, localizer, PIDs) is in `pedroPathing/Constants.java`; `pedroPathing/Tuning.java` and `opmode/debug/*` are used for on-field tuning, not competition code.

### Global/shared state

`hardware/Globals.kt` holds cross-OpMode state that must persist across init/loop boundaries or between autonomous and teleop (alliance color `isRed`, `AUTO` flag, field poses, heading offset). `hardware/Zones.kt` and `hardware/Names.kt` are static field/hardware-map constants (via `com.skeletonarmy.marrow` polygon zones and hardware map device names respectively).

## Conventions

- Motor/servo/sensor hardware-map names are centralized in `Names.kt` — never hardcode a device name string elsewhere.
- Subsystem tunable constants (positions, power levels, regression coefficients) live in each subsystem's `companion object`.
- Field positions/poses are in inches using the FTC field coordinate convention (0,0 to 144,144); see `Globals.kt` and `Zones.kt` for reference poses.
- Commented-out code (e.g. turret subsystem, older regressions) is common in this repo — it's typically disabled-but-kept-for-reference hardware/features (e.g. turret is currently physically absent from the robot), not dead code to clean up. When re-enabling, check `Robot.kt`'s `Subsystems`/`Motors` objects for the matching commented declarations.

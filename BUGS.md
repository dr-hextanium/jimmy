# Code Review — Bugs & Things That Won't Work As Expected

**Review date:** 2026-07-16
**Scope:** All of `TeamCode/src/main` (Kotlin + team-owned Java), the JVM test suite, and the
relevant behavior of the two vendored libraries it leans on — **Pedro Pathing `2.0.6`** and
**FTCLib `2.1.1`** — verified by reading their actual decompiled sources, not from memory.
**Baseline:** `./gradlew :TeamCode:testDebugUnitTest` is **green** (JDK 21 / Android Studio JBR) and
`:TeamCode:assembleDebug` compiles, both before and after the fixes in this pass.
**Method:** one manual pass over every file, plus two independent read-only review sub-agents whose
findings were reconciled here. Anything asserted as library behavior below was confirmed in source.

## How to read this

Each finding has a **severity**, **confidence**, a concrete **failure scenario**, and a **fix plan**.
Findings are split into two tiers by how safe they are to change *right now*, with the physical
robot not in front of us:

- **TIER A — safe to fix now.** Behavior-preserving cleanups, or genuine fixes covered (or newly
  covered) by the JVM test suite that cannot make the robot behave worse. **These were applied this
  pass** and the full suite re-run green (see ✅).
- **TIER B — do NOT change blind; verify on the robot.** Real issues living in competition-tested
  glue (the OpMode loop, autonomous timing, the turret decode) or that need a physical/intent
  decision. Blind-changing them could shift behavior tuned on the field. Each has an analysis and a
  recommended change to make **with the robot in front of you**.

> Most important context: the double-`scheduler.run()` loop (B1) and the turret decode (B2/B3) have
> run through scrimmages and elimination matches. They are not "obviously broken" — they are "subtly
> wrong in ways the current tuning has absorbed." Change them deliberately, on the robot, not
> overnight and untested.

---

## TIER A — Safe fixes (APPLIED this pass ✅)

### A1. Launcher hood slams to `0.0` (below its usable range) at TeleOp start ✅ FIXED
**Severity:** Moderate · **Confidence:** High

**Where:** `hardware/subsystem/Launcher.kt` — `targetHoodPosition` defaulted to `0.0` (field init)
and was **not** reset in `reset()`, while `reset()` sets `hood.position = HOOD_HIGH` (0.25) and
`write()` unconditionally does `hood.position = targetHoodPosition` every loop.

**The bug:** `reset()` parks the servo at `HOOD_HIGH` (0.25, the low end of the hood's usable travel
`[0.25 .. 0.905]`), but the very next `write()` overwrites it with `targetHoodPosition == 0.0`. In
TeleOp, `DriverControlled.initialize()` never sets the hood and `cycle()` only sets it once the
driver taps dpad-up/down — so from START until the first dpad tap, every loop commands the hood to
**0.0, which is 0.25 *below* its intended minimum**, likely jamming it into a mechanical hard stop
(stall/buzz). Autos are unaffected (they call `targetHoodByScalar(0.375)` in `initialize()` first).

**Fix applied:** `reset()` now sets `targetHoodPosition = HOOD_HIGH`, mirroring how it already resets
`targetTPS`. Now `write()` agrees with `reset()`'s stated rest position, so the hood holds `HOOD_HIGH`
from START until the driver moves it. A regression test (`reset()`-then-`write()` keeps the hood at
`HOOD_HIGH`) was added to `LauncherTest`.

---

## TIER B — Verify on the robot before changing

### B1. The command scheduler runs twice per loop; the follower updates ~4×/loop in auto
**Severity:** Moderate (efficiency + subtle auto timing) · **Confidence:** High

**Where:** `hardware/Robot.kt` `update()` calls `scheduler.run()` then every subsystem `update()`;
`opmode/template/BaseTemplate.kt` `loop()` calls `Robot.update()` **and then** `Robot.scheduler.run()`
*again* (same double-tick in `init_loop()`).

**Verified mechanics** (FTCLib `CommandScheduler`, Pedro `Follower`): each `run()` polls buttons once
and calls each scheduled command's `execute()` once; `SequentialCommandGroup.execute()` advances one
sub-command per call; `PedroCommand.execute()` == `follower.update()`. So in an auto loop,
`follower.update()` fires from `Robot.read()` (1) + run-A's `PedroCommand.execute()` (1) +
`AutoTemplate.cycle()` (1) + run-B's `PedroCommand.execute()` (1) = **4×/loop**, and the auto command
tree steps through **two** transitions per loop instead of one.

**Why not catastrophic:** all four follower updates in a loop read the same manually-bulk-cached
sensors (cache cleared once at loop top), so they recompute the same pose and re-issue the same
powers; nearly all team commands are instant, so double-execute is a no-op; `WaitCommand` uses
wall-clock. **In TeleOp it's benign** (no `PedroCommand`, so `follower.update()` still runs once, via
`read()`; the extra `run()` just re-polls buttons — edge detection is safe because both polls see the
same gamepad snapshot).

**Failure scenario / why it matters:** 4× follower cost lowers the loop rate (worse control); repeated
`follower.update()` over ~0 dt can feed noisy velocity/derivative estimates into Pedro; and any
*future* non-instant command that integrates/ramps in `execute()` will run at double rate. The autos'
step-through timing is currently calibrated *with* the 2×-per-loop stepping.

**Fix plan (on the robot):** the tick inside `Robot.update()` is already correctly ordered (scheduler
before subsystem `update()`), so **delete the redundant second `Robot.scheduler.run()`** from
`BaseTemplate.loop()` and `init_loop()`. Then re-run every auto on the field and confirm timing still
lands; nudge `WaitCommand` durations if a transition now arrives slightly late. (Optional follow-up:
`follower.update()` is dispatched from three layers — `read()`, `PedroCommand.execute()`,
`AutoTemplate.cycle()` — consider collapsing to one call site once the double-tick is gone.)

### B2. Turret vernier/CRT decode snaps to the wrong revolution past ~7° of encoder noise
**Severity:** Moderate–High if the analog encoders are noisy · **Confidence:** High (simulated)

**Where:** `hardware/subsystem/Turret.kt` `fuseAbsoluteAngle()` → feeds `currentAngle` (read) straight
into `posError` (update) with no outlier/rate guard.

**What it is:** the 12t/13t idler decode is algebraically exact (the JVM self-consistency test proves
`decode(forward(θ)) == θ`), but the coarse stage amplifies the phase-difference error by
`R12/(R12−R13) ≈ 13×` before `round()` picks the 12t encoder's revolution. A phase-diff error over
~13.9° flips that `round()`, producing a **~31.5° instantaneous jump** in `currentAngle`. Simulated
tolerance (±uniform per-encoder noise, across −95…95°):

| per-encoder noise | worst error | wrong-revolution snaps |
|---|---|---|
| ≤ 6° | < 0.53° | 0 |
| 8° | 32.2° | ~1% of samples |
| 10° | 32.4° | ~8% of samples |

**Failure scenario:** if the Melonbotics analog encoders jitter > ~6–7° (noise/ground/marginal ADC),
`currentAngle` occasionally leaps ~31.5°. `kP·posError = 0.038·31.5 ≈ 1.2 → clamped to full power`, so
the turret slews at full power toward a phantom target for one loop — a violent "twitch" that
self-corrects next loop.

**Fix plan (measure first):** in **Turret Encoder Debug**, hold the turret still and read the fused
angle's jitter. If quiet (< a couple °), no change. If noisy, add a cheap guard: reject a new
`currentAngle` implying motion faster than physically possible (`|Δ| > MAX_VELOCITY·dt·margin` → hold
previous), and/or median-filter the raw voltages. Only after seeing the real noise.

**✅ Addressed in code (guard implemented; still verify + tune on robot).** The revolution flip is
structurally prevented: after an initial memoryless `fuseAbsoluteAngle` acquisition, `update()` tracks
the angle by *continuity* (`selectRevolutionByContinuity` picks the 12t revolution nearest the previous
angle — the turret can't cross a ~31.5° seam in one control loop, so the revolution can't legitimately
jump), and the result is smoothed by a `FadingMemoryFilter` (which also supplies a clean velocity,
re-enabling `kD`). A loop stall (`dt > MAX_DT`) falls back to the memoryless vernier so a real move made
while stalled is still recovered. New JVM tests: `TurretTest.tracking_*` (seam-noise survival,
large-dt re-acquire, velocity tracking) + `FadingMemoryFilterTest`. Remaining on-robot work is tuning
`ANGLE_FILTER_TAU`/`ANGLE_FILTER_SPIKE_GATE` — see TODO §3 "Turret angle filtering". (Note: **B3**
gear-mesh sign + zero-offset calibration is still required and unaffected by this change.)

**Further hardening (opt-in, `USE_MOTOR_FUSION`, default OFF).** The drive motor's quadrature encoder
can be fused in via a `ComplementaryFilter` (motor tick delta predicts, absolute corrects). Because
motor ticks accumulate in hardware through a stall, fusion carries the estimate across a `dt > MAX_DT`
gap by *continuity* instead of the memoryless vernier — removing even the brief 13t-flip exposure of
the stall re-acquire above — and yields a clean tachometer velocity for `kD`. It is OFF by default and
introduces a **second** independent sign to validate (`MOTOR_ANGLE_SIGN`, runaway-class if wrong, same
hazard family as **B3**), guarded by default-OFF + an on-robot sign check + an auto-revert health
monitor. Tests: `ComplementaryFilterTest` + `TurretTest.fusion_*`. Enable per TODO §3 "Turret
motor-encoder fusion".

### B3. Turret gear-mesh sign & encoder zero-offsets unvalidated → wrong sign = runaway
**Severity:** Critical until calibrated · **Confidence:** High (already known)

**Where:** `Turret.kt` `fuseAbsoluteAngle()` assumes both idler ratios positive (`:257-258`);
`ENCODER_12T/13T_ZERO_OFFSET_DEG`. Already in `TODO.md §3`; repeated so the turret picture is whole:
if the idlers physically turn opposite to the assumed convention, the decode returns `−θ`, so
`posError` has the wrong sign → **positive feedback → the turret runs away from target under power.**
The self-consistency tests *cannot* catch this (they build the forward model from the same `+ratio`
convention, so a sign flip passes every test). **Do not trust the turret for closed-loop motion until
the mesh sign and both zero offsets are set on the real turret** — including that it runs its PID
during auto INIT (B6).

### B4. `AUTO_RECOVERY_POSITION` is never written — TeleOp starts mislocalized every match
**Severity:** Moderate · **Confidence:** High that it's unwired; Medium that it's a defect vs. an
intended manual-reset workflow

**Where:** `hardware/Globals.kt` declares `AUTO_RECOVERY_POSITION` (null); `DriverControlled.kt`
reads `Robot.follower.pose = AUTO_RECOVERY_POSITION ?: Pose(72,72,0)` and nulls it in `stop()`.
Grep across all of `TeamCode/src/main` shows **no assignment of a non-null value anywhere.**

**Failure scenario:** the `?:` fallback always fires, so every TeleOp begins with the localizer
believing the robot is dead-center at heading 0 — regardless of where auto left it. Every
pose-dependent feature (goal-lock `face()`, `distanceToGoal()`, turret shoot-on-the-move) is wrong
until the driver hits `LEFT_STICK_BUTTON` to snap to `resetPose`. The named "recovery" hand-off is
dead code.

**Fix plan (decide the workflow):** if you want the auto→TeleOp pose hand-off, write the robot's
final pose at the end of each auto (`Globals.AUTO_RECOVERY_POSITION = follower.pose`) — but only if
your auto localization is trustworthy end-of-run, else TeleOp inherits auto's drift. If the manual
reset button is the intended workflow, delete the unused field and the `?:` and just default to a
known pose. Left unchanged pending your call (it changes match-start localization).

### B5. `RedClose12Old` is the OpMode literally named "Red Close 12" on the Driver Station
**Severity:** High (operational — wrong auto could be selected in a match) · **Confidence:** High

**Where:** `opmode/auto/RedClose12Old.kt` → `@Autonomous(name = "Red Close 12")`, vs current
`RedClose12ElimsGate.kt` → `"Red Close 12 Elims Gate"`. The class named `…Old` shows up as plain
**"Red Close 12"** in the picker — the natural pick under pressure, but it runs the *old* routine.
(Blue's old variant is correctly named "Blue Close 12 Old", so only Red has this trap.)

**Fix plan (needs your decision — don't delete/rename blind):** tell me which Red auto is the live
one, then either delete `RedClose12Old` + `paths/RedClose12.java`, or rename its `@Autonomous(name=…)`
to include "Old". Already noted in `TODO.md`'s "decide separately" list; this raises its priority.

### B6. The turret drives its PID during autonomous `init_loop` (before START)
**Severity:** Low–Moderate (calibration-safety interaction) · **Confidence:** High

**Where:** `BaseTemplate.init_loop()` runs the full `read()/update()/write()` pipeline when
`Globals.AUTO`; `Turret.update()`/`write()` therefore hold-at-0° during INIT.

**Failure scenario:** with offsets uncalibrated (B3), `currentAngle` can read far from 0°, so during
INIT the turret can drive at up to full power toward a phantom 0° and hit a hard stop / wind cables.
Benign once calibrated. **Fix plan (optional):** gate the turret motor write on a "started" flag if
you don't want it energized pre-match — but only after B3.

### B7. `InstantCommand({ stop() })` at the end of every auto does not actually stop the OpMode
**Severity:** Minor · **Confidence:** Medium (standard iterative-OpMode semantics)

**Where:** `BlueClose12.kt`, `Blue15Close.kt`, `RedClose12ElimsGate.kt`, `RedClose12Old.kt` end with
`InstantCommand({ stop() })`. Neither template overrides the framework's `stop()` lifecycle hook, so
calling it directly runs an (empty) method — it does **not** request OpMode termination.

**Failure scenario:** after the sequence "ends," `loop()` keeps running: the follower holds the last
path end and the flywheel keeps spinning at the last `ManuallyLaunch` target (never zeroed in auto)
until the 30 s timer. Wastes battery / wears the flywheel; the intended halt is a no-op.

**Fix plan:** if you actually want to halt, use `requestOpModeStop()`; if you want to zero actuators
at auto end, add `StopLauncher()`/`StopIntake()` to the tail. Decide whether stopping early (leaving
the robot idle until TeleOp) is even desirable before changing — currently it just holds.

### B8. TeleOp flywheel start power (0.71) disagrees with the displayed/adjustable value (0.64)
**Severity:** Minor · **Confidence:** High

**Where:** `BaseTemplate.start()` schedules `ManuallyLaunch { 0.71 }`; `DriverControlled.kt` has
`launcherPower = 0.64` and shows it as "launching scalar". The first dpad trim recomputes from 0.64.

**Failure scenario:** at START the flywheel spins to 0.71 but telemetry reads 0.64; the driver's first
nudge drops it from 0.71 to ~0.65 (a step *down* when they pressed to go up), and a shot right after
the first trim is at the wrong speed.

**Fix plan (tuning decision):** pick one source of truth — either start at `launcherPower` (0.64) or
set `launcherPower = 0.71` to match the start command — so telemetry, start speed, and the first trim
all agree. Which value is "right" is a shooting-tuning call, so left for you.

---

## Design notes — real, but do NOT "fix" naively

- **No command declares subsystem requirements, and that is load-bearing.** `CommandTemplate` takes
  `vararg requirements` but every command passes none, so `getRequirements()` is always empty. This
  is currently *required*: `IntakeWithGateClosed`/`FeedLauncherArtifacts` are `ParallelCommandGroup`s
  whose members both touch Intake, and FTCLib's `ParallelCommandGroup` **throws
  `IllegalArgumentException` at construction if two members share a requirement** (verified in source).
  So adding "proper" requirements would crash OpMode init. The cost is that no command interrupts
  another and default-commands are impossible — fine today (all commands instant), fragile the moment
  someone adds a long-running command expecting mutual exclusion. Leave as-is; just know it.
- **`Launcher.read()` is empty while `averageTPS` reads `left/right.velocity` inside getters/`update()`.**
  Violates the read→update→write discipline CLAUDE.md states ("never read hardware in `update()`").
  Harmless on the FTC SDK with manual bulk caching (the values are cached, cleared once per loop), and
  refactoring it would ripple through the tests that poke `fakeVelocity` then read `averageTPS`
  directly. Documented, not changed.

## Non-issues examined and cleared (so you don't re-chase them)

- **`follower.setTeleOpDrive(...)` at the end of `loop()` during auto** — inert. `setTeleOpDrive`
  only stashes vectors; `update()` applies them **only when `manualDrive == true`**, set solely by
  `startTeleopDrive()`, which auto never calls. Path following is unaffected.
- **`Robot.pose.heading = …` in the reset binding (`DriverControlled`)** — a no-op, because Pedro's
  `Pose` is immutable (`setHeading` returns a *new* Pose that Kotlin discards). The real relocalize is
  `follower.pose = resetPose`. Covered under the intent question in B-note below.
- **`Robot.init()` registers subsystems then `reset()` nulls the scheduler singleton** — latent only;
  subsystems are updated explicitly (not via `periodic()`) and there are no default commands, so
  nothing depends on the registration today. (Tidy eventually: register after the reset.)
- **Vector cartesian/polar mixups** — none remain. Goal poses use `Vector(Pose)` (cartesian); the
  `Vector(1, θ)` uses in `Tuning.java` are intentional polar unit vectors.
- **`measuredVelocity` spiking at a fused-angle wrap** — cleared. `fuseAbsoluteAngle` is continuous
  across every 12t seam within its ±205° unambiguous range (the revolution counter increments exactly
  as the raw angle drops 360), so its derivative doesn't spike in normal operation — only on a
  misdecode (B2).
- **`percentDifference(0,0)` → NaN in `Launcher.write()`** — intended and tested; the `> 0.005` guard
  treats NaN as "no change." Not a bug.
- **`BaseTemplate.face()` "degrees"-named locals hold radians** — misleading names but mathematically
  consistent (atan2/heading/normalize are all radians; only telemetry converts). Pure-P controller,
  no windup. Not a bug.
- **Control core** — `TrapezoidalProfile` (a faithful WPILib port), `ProjectileSolver`, `ShooterModel`,
  the launcher velocity loop, `InterpLUT`, `map`/`percentDifference`: reviewed closely against their
  tests by both a manual pass and an independent sub-agent; **no correctness defects found.** This
  code is solid.

### Intent question — RESOLVED & implemented (2026-07-16)
You clarified: **right stick = heading, left stick = position to base zone.** Implemented in
`DriverControlled`:
- **RIGHT_STICK_BUTTON** — heading reset (`globalHeadingOffset = current heading`), unchanged.
- **LEFT_STICK_BUTTON** — relocalize to the base zone: `follower.pose = baseZonePose` (position +
  the known base-zone heading) and clear the heading offset. The dead `Robot.pose.heading = …`
  no-op line was removed, and the intended base-zone heading now actually applies via the pose.

New `Globals.RED_BASE_POSE = Pose(38.5, 33.5, 0°)` / `BLUE_BASE_POSE = Pose(105.5, 33.5, 180°)`
(x/y mirror the base-zone reference points in `Zones.kt`; heading is the base-zone facing).
**Confirm these x/y/heading against the real field** — added to `TODO.md §5`.

## Housekeeping (cosmetic; intentionally NOT applied to avoid noisy diffs over tuned code)

- **`pedroPathing/ArtifactPlanning.kt` is broken and unused.** Its Levenberg-Marquardt call is fed an
  all-zero Jacobian (`Array2DRowRealMatrix(3, 12)`), so the optimizer can't move off the initial guess
  — it returns garbage. Nothing references `getPedroPathCode`, and it's the only reason the
  `org.hipparchus:hipparchus-optim` dependency exists. Recommend deleting the file + the dependency
  (your call — may be WIP you want to keep).
- **`utility/InterpLUT.kt` is unused in production** (test-only). Harmless; keep or drop.
- **Mixed tabs/spaces** across several files (`Robot.kt`, `BaseTemplate.kt`, `Intake.kt`, `Names.kt`,
  …). Purely cosmetic (Kotlin is whitespace-insensitive). A repo-wide reformat would bury the real
  changes in noise, so it's intentionally left alone.

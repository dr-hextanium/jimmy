# TODO — New-Robot Bring-Up Checklist

Every value in this file is currently a **placeholder or an assumption from the old robot**. Nothing
here is a code bug — these are the physical/calibration constants the code deliberately leaves for
you to fill in once the new robot exists. Work top-to-bottom: config names first (nothing runs
without them), then directions, then measurements, then on-robot calibration, then tuning.

All the tools referenced live under `opmode/debug/` and show up on the Driver Station under the
**"Debug"** group. They are surgical (each touches only the hardware it needs, most are dead-man /
open-loop) so they are safe to run on an untuned robot.

Legend: 🔴 = robot will not work / will move wrong until set · 🟡 = shots/aim will be off until
calibrated · 🟢 = tune for performance, safe defaults already in place.

---

## 0. Driver Hub Robot Configuration — device names 🔴

The strings below are hard-coded in `hardware/Names.kt` (and the drivetrain/odometry names in
`pedroPathing/Constants.java`). Your **Robot Configuration on the Control Hub must use these exact
names**, OR change the constants here to match your config. Any mismatch = `hardwareMap` crash on
init. Do NOT hardcode names anywhere else — `Names.kt` is the single source of truth.

Config type | Device | Config name | Defined at
---|---|---|---
DcMotor | Intake | `intake` | `Names.kt:6`
DcMotor | Turret | `turret` | `Names.kt:10`
DcMotor | Launcher left | `sl` | `Names.kt:14`
DcMotor | Launcher right | `sr` | `Names.kt:15`
DcMotor | Drive front-right | `fr` | `Names.kt:19` / `Constants.java:35`
DcMotor | Drive front-left | `fl` | `Names.kt:20` / `Constants.java:38`
DcMotor | Drive back-right | `br` | `Names.kt:21` / `Constants.java:36`
DcMotor | Drive back-left | `bl` | `Names.kt:22` / `Constants.java:37`
Servo | Intake gate | `gate` | `Names.kt:28`
Servo | Launcher hood | `hood` | `Names.kt:32`
Analog input | Turret 12T encoder | `te12` | `Names.kt:38`
Analog input | Turret 13T encoder | `te13` | `Names.kt:39`
I2C (Pinpoint) | Odometry computer | `pinpoint` | `Constants.java:48`

- [ ] If you don't yet know which physical motor is on which port/name, run **"Motor Mapper"**. It
      live-enumerates every motor in the config (including names *not* in `Names.kt`, flagged
      `UNMAPPED`), and lets you spin one at a time (hold RT/LT, dead-man, capped at 0.6) to see which
      one moves — plus its port + hub. Use it to reconcile the config names/ports above.
- [ ] Every device above exists in the Control Hub config with the matching name and port.
- [ ] Run **"Robot Status Debug"** — it calls `Robot.init(...)`. If it inits without a crash, all
      names/types resolve. (It only reads; nothing is driven.)

---

## 1. Motor & encoder DIRECTIONS 🔴

Directions are set in code; whether they're right depends on how the new robot is wired and geared.
Wrong directions are the #1 cause of "it runs away on enable."

### Drivetrain (Pedro) — `pedroPathing/Constants.java:39-42`
Current: `leftFront=REVERSE, leftRear=REVERSE, rightFront=FORWARD, rightRear=FORWARD`.
- [ ] Run **"Motor Direction Debugger"** (triangle=FR, square=FL, circle=BR, cross=BL). Confirm each
      wheel drives the robot **forward** when its button is held. Flip any that go backward.

### Odometry pods (Pinpoint) — `Constants.java:50-51`
Current: `forwardEncoderDirection=REVERSED, strafeEncoderDirection=FORWARD`.
- [ ] Push the robot forward → the pose `x`/`y` must increase in the expected direction on **"Robot
      Status Debug"**. Push left/strafe → confirm the other axis. Flip a pod's direction if the sign
      is wrong. Also re-measure the pod offsets `forwardPodY (-4.75)` / `strafePodX (-6.00)` (inches)
      for the new chassis (`Constants.java:45-46`) and confirm the pod type on `:49`
      (`goBILDA_4_BAR_POD`).

### Launcher motors — `hardware/subsystem/Launcher.kt:76-77`
Current: `right=REVERSE, left=FORWARD`. Both must spin the single flywheel the **same** way.
- [ ] Run **"Launcher RPM (open loop)"** — dial up a little power and confirm both wheels drive a
      ball *out*, and that the two reported velocities have the **same sign**. Flip one if they fight.

### Turret motor — `hardware/subsystem/Turret.kt:136` (`FORWARD`) + gear-mesh sign, see §3.

### Intake motor — `hardware/subsystem/Intake.kt:27` (`REVERSE`)
- [ ] Run **"Intake Debug"** (right trigger = in) and confirm it pulls artifacts *in*.

---

## 2. Physical MEASUREMENTS 🟡

These feed the kinematic shooter (`control/ShooterModel.kt`). They replace the old empirical
power/hood regressions with real geometry, so they must be measured, not guessed.

- [ ] **`FLYWHEEL_DIAMETER_MM`** — `ShooterModel.kt:33` (default `72.0`). Measure the flywheel OD.
- [ ] **`COUNTER_ROLLER_DIAMETER_MM`** — `ShooterModel.kt:34` (default `28.0`). Measure the back
      counter-roller OD.
- [ ] **`COUNTER_ROLLER_GEAR_RATIO`** — `ShooterModel.kt:37` (default `2.0`, i.e. 40t→20t). Confirm
      the actual gearing between flywheel and counter-roller.
- [ ] **`LAUNCHER_TICKS_PER_REV`** — `ShooterModel.kt:40` (default `28.0`, bare-motor CPR). Set to
      your flywheel motor's encoder counts/rev from its datasheet.
- [ ] **`TARGET_HEIGHT_DELTA_M`** — `ShooterModel.kt:46` (default `0.5`). Height of the goal center
      **above the ball's launch point**, in metres. Measure on the real robot + real field.

---

## 3. On-robot CALIBRATION 🟡

Values that can only be found by running the physical robot. All default to safe placeholders (same
pattern as the turret encoder offsets).

### Turret absolute-encoder zero offsets 🔴 (turret won't know where it is)
`Turret.kt:433-434` — `ENCODER_12T_ZERO_OFFSET_DEG`, `ENCODER_13T_ZERO_OFFSET_DEG` (default `0.0`).
- [ ] Run **"Turret Encoder Debug"**. Move the turret to its true mechanical **zero**, read the two
      "raw deg" values, and set the offsets to those numbers.
- [ ] **Gear-mesh sign:** in the same tool, rotate the turret in your **positive** direction and
      confirm the *fused* angle **increases**. If it decreases, the sign assumption in
      `fuseAbsoluteAngle` is wrong (ratios at `Turret.kt:427-428`) — see the note at `Turret.kt:388-390`.
      Do not trust the turret for motion until the fused angle tracks correctly through the full
      `MIN_ANGLE..MAX_ANGLE` sweep.
- [ ] Confirm `ENCODER_MAX_VOLTAGE` (`Turret.kt:425`, default `3.3`) matches your analog encoders.
- [ ] Confirm `MOTOR_TICKS_PER_REV` (`Turret.kt:417`, `145.1`) and `TURRET_GEAR_RATIO`
      (`Turret.kt:420`, `137/15`) match the actual motor and gear counts.
- [ ] Confirm the idler gear ratios `ENCODER_12T_GEAR_RATIO` (`137/12`) and `ENCODER_13T_GEAR_RATIO`
      (`137/13`) at `Turret.kt:427-428` match the rebuilt turret's tooth counts. The vernier/CRT
      decode in `fuseAbsoluteAngle` relies on these two being **coprime**; if either idler or the
      turret gear changed, the whole absolute-angle fusion must be re-derived, not just re-tuned.

### Turret angle filtering 🟡 (encoders are noisy; defaults are safe, tune for steadiness)
`Turret.kt:440-441` — `ANGLE_FILTER_TAU` (`0.02` s), `ANGLE_FILTER_SPIKE_GATE` (`10.0` deg). Both are
`@Configurable`, so live-tune from the dashboard. The fused angle is now continuity-tracked (immune to
the ~31.5° revolution flip the raw vernier can suffer under noise) and smoothed by a `FadingMemoryFilter`.
- [ ] In **"Turret Encoder Debug"**, at rest compare `measured angle (pre-filter)` vs `FUSED angle
      (filtered)`. Raise `ANGLE_FILTER_TAU` until the filtered angle is steady; stop before jog
      tracking feels laggy (the turret goal-locks a *moving* target).
- [ ] Set `ANGLE_FILTER_SPIKE_GATE` just above the worst clean per-loop angle step (≈ `MAX_VELOCITY` ×
      loop time) so it clips only gross spikes, never real motion.
- [ ] Sweep the full travel including revolution seams (~every 31.5°) and confirm the FUSED angle never
      steps ~31.5° at speed. (Supersedes the old "decode noise margin" bug item — see §Bugs.)

### Turret motor-encoder fusion 🟢 (opt-in; OFF by default — the absolute-only path above is the default)
`Turret.kt:447-450` — `USE_MOTOR_FUSION` (`false`), `MOTOR_ANGLE_SIGN` (`1.0`), `MOTOR_FUSION_TAU`
(`0.10` s), `MOTOR_FUSION_GATE` (`15.0` deg). All `@Configurable`. Fuses the drive motor's quadrature
encoder (clean, high-res, stall-proof) with the absolute decode via a `ComplementaryFilter`, for a
cleaner velocity, near-zero-lag position, and stronger revolution disambiguation. **A wrong
`MOTOR_ANGLE_SIGN` is runaway-class**, so enable it ONLY after the sign check below.
- [ ] **Confirm the sign FIRST.** In **"Turret Encoder Debug"** (fusion still OFF), jog **positive** and
      confirm the `motor-implied angle (deg)` readout increases together with the FUSED angle. If it
      decreases, flip `MOTOR_ANGLE_SIGN` (`Turret.kt:448`) to `-1.0`.
- [ ] Confirm the drive-train scale is right (`MOTOR_TICKS_PER_REV` / `TURRET_GEAR_RATIO`, above) — a
      scale error is self-correcting but keep it close.
- [ ] Only then set `USE_MOTOR_FUSION = true`. Sweep the full travel across seams and confirm the FUSED
      angle stays continuous (no ~31.5° jump) and the `fusion healthy?` readout stays true.
- [ ] Tune `MOTOR_FUSION_TAU` (larger = steadier/trusts motor more, smaller = corrects to absolute
      faster) and `MOTOR_FUSION_GATE` (must sit **above** the turret's gear backlash and **below** one
      seam ≈ 31.5°). Then revisit `kD` — the tachometer velocity is cleaner than the filtered absolute.

### Turret travel limits 🔴 (prevents winding up cables / hitting hard stops)
`Turret.kt:457-458` — `MAX_ANGLE` (`90.0`), `MIN_ANGLE` (`-90.0`).
- [ ] Set to the real mechanical travel of the turret (degrees from zero). `setTargetAngle` clamps
      to these.

### Flywheel slip efficiency
`ShooterModel.kt:43` — `SLIP_EFFICIENCY` (default `0.85`). Single knob folding all compression/slip
loss; ball exit speed scales linearly with it.
- [ ] Run **"Launcher RPM (open loop)"** to a known steady TPS, measure the real ball exit speed
      (chronograph, or back it out from measured range at a known hood angle), then set
      `SLIP_EFFICIENCY` so `exitSpeedFromTps(measuredTPS)` matches reality. Tune this **after** the
      geometry in §2 is correct.

### Hood angle ↔ servo calibration 🟡 ⚠️ **CALIBRATE IN TWO PLACES**
The hood has two independent servo-position calibrations that **must stay consistent**, or the
manual-aim path and the kinematic-aim path will silently disagree:

1. Kinematic model — `ShooterModel.kt:49-54`:
   - `HOOD_MIN_ANGLE_RAD` / `HOOD_MAX_ANGLE_RAD` (default 30°/60°) = the real achievable launch-angle
     range of the hood.
   - `SERVO_AT_MIN_ANGLE` (`0.25`) / `SERVO_AT_MAX_ANGLE` (`0.905`) = the servo positions at those
     two angles.
2. Manual/driver path — `Launcher.kt:121-122`: `HOOD_LOW` (`0.905`) / `HOOD_HIGH` (`0.25`).

- [ ] Run **"Servo Debugger"** (select the `hood` servo). Find the servo positions at the hood's
      min and max mechanical angles, and measure those angles.
- [ ] Set `SERVO_AT_MIN_ANGLE`/`SERVO_AT_MAX_ANGLE` **and** `HOOD_HIGH`/`HOOD_LOW` from the same
      capture (today: HIGH=0.25=min-angle, LOW=0.905=max-angle — keep them equal to the ShooterModel
      endpoints).
- [ ] Set `HOOD_MIN_ANGLE_RAD`/`HOOD_MAX_ANGLE_RAD` to the measured angles.

### Gate servo positions
`Intake.kt:66-67` — `GATE_OPEN_POSITION` (`0.700`) / `GATE_CLOSED_POSITION` (`0.445`).
- [ ] Run **"Servo Debugger"** (select `gate`), capture the open and closed positions, set them here.

### Flywheel speed ceiling
`ShooterModel.kt:57` — `MAX_TPS` (default `2500.0`). The required flywheel speed is clamped to this;
`Launcher.MAX_TPS` re-uses it as the manual-scalar ceiling (`Launcher.kt:127`).
- [ ] From **"Launcher RPM (open loop)"**, note the max steady TPS the flywheel actually holds at
      full power and set `MAX_TPS` at/under that.

---

## 4. Control-loop TUNING 🟢

Safe starting gains are in place; tune for performance once directions + calibration are done.

### Turret feedforward + PID — `Turret.kt:463-475`
**Automated (recommended): run "Turret Auto-Tune"** (`opmode/debug/TurretAutoTune.kt`). It
characterizes the plant open-loop (SysId-style) and prints ready-to-paste `kStatic`, `kV`, `kA`,
`kP`, `kD` — no hand-tuning. Procedure:
- [ ] Jog the turret to its mechanical **centre** by eye (triggers, capped, dead-man), press **A**.
- [ ] **Hold RIGHT BUMPER** to run (release / press **B** stops instantly). It sign-probes, then
      ping-pongs a steady-state sweep, then runs from-rest dynamic steps. A hard motor-tick limit
      (`HARD_LIMIT_DEG = 70`, from the centred start) is an independent backstop — centre carefully so
      the ±70° band stays inside the real ±90° travel.
- [ ] At **DONE**, copy the five gains into the companion, or press **Y** to push them live to the
      dashboard first and test-drive. It only uses the drive-motor encoder, so it works **before** the
      absolute-encoder calibration in §3 is finished.
- [ ] `kD` is applied live only when `USE_MOTOR_FUSION` is on (the tune uses the clean motor tach; the
      absolute-only path's `measuredVelocity` is noisier, so the shipped `kD=0` is kept otherwise).
- [ ] Watch the printed diagnostics: a large `kStatic +/-` asymmetry flags a mechanical bind; low fit
      R² or a low `min battery (V)` means rerun. Adjust `SETTLING_TIME_S` (default `0.2`s) for a
      stiffer/softer loop and rerun.

**Manual fallback** (if you'd rather tune by hand):
- [ ] `MAX_VELOCITY` (`700.0` deg/s), `MAX_ACCELERATION` (`3600.0` deg/s²) — motion-profile limits.
      Start conservative, raise until it tracks without stalling/overshoot. (Auto-Tune does **not**
      set these — they cap the profile, not the plant model.)
- [ ] `kV` (`0.0012` ≈ 1/MAX_VELOCITY), `kA` (`0.0`), `kP` (`0.038`), `kStatic` (`0.02`). Tune
      feedforward (`kV`, then `kStatic`) first, then trim with `kP`.
- [ ] `kD` (`0.0`) — velocity error damping. It was off because the raw encoder was too noisy to
      differentiate; velocity now comes from the `FadingMemoryFilter` (`measuredVelocity`), or the motor
      tachometer when motor fusion is on (cleaner), so after tuning the filter you can raise `kD` until
      motion is critically damped (no overshoot ring).
- [ ] `ANGLE_TOLERANCE_DEGREES` (`Turret.kt:475`, `0.2`) — "at target" band for `isAtTarget()`.

### Launcher velocity loop
Feedforward-first (`kS + kV·targetTPS + kP·error`, clamped 0..1). Do **not** make it
proportional-dominant.

**Automated (recommended): run "Launcher Auto-Tune"** (`opmode/debug/LauncherAutoTune.kt`). It
characterizes the flywheel open-loop and prints ready-to-paste `kS`, `kV`, `kP` **and** a recommended
`SPINUP_CURRENT_LIMIT_A` at the knee of the spin-up-time-vs-current curve. Procedure:
- [ ] Clear the flywheel area (it **will spin to near full speed**). **Hold RIGHT BUMPER** to run
      (release / press **B** stops instantly). Phase A sweeps steady-state powers; Phase B sweeps the
      current limit and times the spin-up at each.
- [ ] Run it on a **representative (freshish) battery** — stall current scales with pack voltage.
- [ ] At **DONE**, copy `kS`/`kV`/`kP`/`SPINUP_CURRENT_LIMIT_A` into the companion, or press **Y** to
      apply them live (they persist into the next TeleOp). Watch the diagnostics: FF-fit R², `tau`, the
      time-vs-limit table, and the "running current at ref" (the recommendation is floored above it so
      the limit can't throttle the wheel at speed). A "no clear knee" note means widen the sweep.

**Manual fallback:**
- [ ] `kV` (`0.0004` ≈ 1/MAX_TPS) — set so `kV·targetTPS` alone roughly holds the target speed.
- [ ] `kS` (`0.0`) — static offset; `kP` (`0.0003`) — small trim only.
- [ ] `AT_SPEED_TOLERANCE` (`0.05` = 5%) and `MIN_TPS` (`100.0`).
- [ ] Current alert is set to `15.0 A` in `Launcher.kt` — adjust for your motors if needed.

### Launcher current-limited spin-up 🟢 (opt-in; OFF by default) — `Launcher.kt`
`SPINUP_CURRENT_LIMIT_A` (default `0.0` = **disabled** → the plain feedforward+P loop, unchanged) and
`SPINUP_RECOVERY_PER_SEC` (`6.0`). When set, a `CurrentLimiter` caps power so per-motor current stays
near the budget — fastest spin-up within a current limit (`min(FF+P, currentCap)`; only bites during
spin-up, hands back to the velocity loop at speed).
- [ ] Set `SPINUP_CURRENT_LIMIT_A` from the "Launcher Auto-Tune" knee recommendation. **It must sit
      above the steady-state current needed to hold the target speed** (the tuner floors it there); a
      value below that will throttle the wheel at speed and stall shots.
- [ ] The enforced limit is *soft*: the one-loop stall spike and the constant-current hunt run a bit
      above the setpoint. Compare the tuner's printed peak vs the setpoint if peak draw matters.
- [ ] Confirm the drive motors report per-port current (`getCurrent`) on the hub; a bad reading makes
      the limiter inert (falls back to the plain loop) by design.

### Pedro Pathing follower — `pedroPathing/Constants.java`
- [ ] `mass(11.34)` (`:28`) — set to the **new robot's mass in kg**.
- [ ] PIDF coefficients (`:22-25`): `translational (0.08…)`, `heading (1.0…)`,
      `drive (0.03…)`, `secondaryHeading (3.5…)`, plus `headingPIDFSwitch(0.157)` (`:26`). Retune with
      the Pedro tuning OpModes (`pedroPathing/Tuning.java`).
- [ ] Two secondary-PIDF toggles are intentionally left commented at `Constants.java:19,21`
      (`useSecondaryTranslationalPIDF`, `useSecondaryDrivePIDF`) — **enable these while tuning** the
      new drivetrain if you want a two-zone PIDF (they're kept on purpose, not dead code).
- [ ] `pathConstraints` (`:30`) and `maxPower(1.0)` (`:33`).

---

## 5. Field poses & autonomous starting positions 🟡

`hardware/Globals.kt` (FTC field coords, inches, 0–144):
- [ ] `RED_GOAL_POSE` = (144, 144), `BLUE_GOAL_POSE` = (0, 144) (`Globals.kt:21,24`) — confirm against
      the real Decode goal locations. These drive turret aim and shooter distance.
- [ ] `BLUE_RESET_POSE` / `RED_RESET_POSE` (`Globals.kt:27,30`) — relocalization poses; confirm.
- [ ] `RED_BASE_POSE` / `BLUE_BASE_POSE` (`Globals.kt`) — base-zone relocalize poses used by the
      **LEFT_STICK** button (position + base-zone heading). Confirm x/y/heading on the real field;
      x/y currently mirror the base-zone reference points in `Zones.kt`, heading = 0°(red)/180°(blue).

Autonomous start poses:
- [ ] `Robot.kt:125` sets a default `follower.setStartingPose(Pose(0.0, 0.0, 0.0))`. Each auto in
      `opmode/auto/*` (via `AutoTemplate`) should set its **real** starting pose on the field; verify
      each routine you plan to run starts where the robot is physically placed.

---

## 6. Final sanity pass

- [ ] **"Robot Status Debug"** with the robot on the field: pose, distance-to-goal, turret fused
      angle, launcher TPS/at-speed, hood/gate positions all read sane.
- [ ] **"Intake Debug"**: intake pulls artifacts in on the right trigger, ejects on the left, and
      the gate opens/closes (verify motor direction and gate travel).
- [ ] Full JVM test suite still green: `./gradlew :TeamCode:testDebugUnitTest` (JDK 17–21 / the
      embedded JBR — **not** the default JDK 24).

---

> **Note:** the subsystem `companion object`s hold a few more constants not listed above
> (`POWER_UPDATE_THRESHOLD`, `MAX_DT`, intake `POWER_INTAKE`/`POWER_REVERSE`, `MIN_TPS`, etc.). Those
> are algorithm defaults, **not** robot-specific — leave them unless a specific problem points at one.
> The lists above cover every value that depends on the physical robot.

---

## 7. Code-review findings & fixes (2026-07-16) — see [`BUGS.md`](BUGS.md)

A full bug-hunt pass over `TeamCode/src/main` + the JVM tests. Full write-up (severities, failure
scenarios, per-item fix plans, and the non-issues that were cleared) lives in `BUGS.md`. The control
core (`control/`, and the launcher/turret/intake math) came back **clean**. One safe fix was applied
and tested; the rest are competition-tested glue/timing that should be changed **on the robot**, not
blind.

### Applied this pass ✅ (fixed + JVM-tested, suite green)
- [x] **Launcher hood no longer slams to `0.0` at TeleOp start.** `Launcher.reset()` now seeds
      `targetHoodPosition = HOOD_HIGH`, so the first `write()` parks the hood at its rest position
      instead of driving it past the usable-travel low end. Regression test added. (BUGS.md A1)

### Verify on the robot, then fix 🟡 (do NOT change blind)
- [x] **Scheduler runs twice per loop** → `follower.update()` ~4×/loop in auto. **Applied** in the
      loop-time pass (see §8): scheduler and follower now tick **once per loop**. ⚠️ still needs the
      on-field re-check — re-run every auto and confirm path tracking/segment transitions. (BUGS.md B1)
- [x] **Turret decode noise margin.** The rate/outlier guard this asked for now exists: the fused
      angle is continuity-tracked (can't take the ~31.5° revolution flip) and smoothed by a
      `FadingMemoryFilter`. Remaining work is *tuning*, not a fix — see "Turret angle filtering" in §3.
      (BUGS.md B2/B3)
- [ ] **`AUTO_RECOVERY_POSITION` is never written**, so TeleOp always starts localized at field
      center `(72, 72, 0)`. Decide: wire the auto→TeleOp pose hand-off, or drop the field. (BUGS.md B4)
- [ ] **`RedClose12Old` shows as "Red Close 12"** on the Driver Station while the live routine is
      "Red Close 12 Elims Gate" — pick the canonical Red auto, then rename/delete. (BUGS.md B5; see
      also "decide separately" below)
- [ ] **Turret runs its PID during auto `init_loop`** (pre-START). Benign once §3 is calibrated;
      gate the motor write on a "started" flag if you don't want it energized in INIT. (BUGS.md B6)
- [ ] **`InstantCommand({ stop() })` at auto end doesn't stop the OpMode**; the flywheel keeps
      spinning to the 30 s timer. Use `requestOpModeStop()` / zero the actuators for a real halt.
      (BUGS.md B7)
- [ ] **TeleOp flywheel start (`0.71`) ≠ shown scalar (`0.64`)**, so the first dpad trim steps the
      speed *down*. Pick one source of truth. (BUGS.md B8)
- [ ] **Relocalize button heading intent** — the `LEFT_STICK_BUTTON` reset ends up using the
      reset-pose heading (143°/37°), not `actualHeadingAtBaseZone` (0/π) as a dead line implied.
      Decide what the button means physically and make it explicit. (BUGS.md, intent note)

### Design notes / cosmetic (do NOT "fix" naively — see BUGS.md)
- Commands intentionally declare **no** subsystem requirements. This is load-bearing: parallel groups
  whose members both touch Intake (`IntakeWithGateClosed`, `FeedLauncherArtifacts`) would otherwise
  throw at construction. Do not add requirements.
- `pedroPathing/ArtifactPlanning.kt` is broken (all-zero Jacobian) and unused — candidate for deletion
  along with the `org.hipparchus:hipparchus-optim` dependency. Mixed tabs/spaces left as-is to keep
  the diff reviewable.

---

### Not addressed here (decide separately)
- The four `Old` variant files (`opmode/auto/BlueClose12Old.kt`, `RedClose12Old.kt`,
  `paths/BlueClose12OldPaths.java`, `paths/BlueClose12PathsOld.java`) are whole historical files, not
  commented-out code, so this cleanup left them in place. Delete them if they're truly obsolete —
  your call. ⚠️ **`RedClose12Old` is annotated `@Autonomous(name = "Red Close 12")`** — so on the
  Driver Station it appears as the plain "Red Close 12" and can be selected in a match by mistake
  instead of the live "Red Close 12 Elims Gate". Resolve the name before it bites you (BUGS.md B5).
- Battery-voltage monitoring was removed from `Robot.kt` (it was fully dead — sampled nowhere). Re-add
  a `VoltageSensor` read in `Robot.read()` if you want brownout/voltage-comp telemetry on the new robot.
- The `limelight` device grab was removed from `Robot.kt`. Re-add it if the new robot has vision.

---

## 8. Loop-time optimization pass (2026-07-20)

Reduced per-loop work and jitter. **JVM suite green; every change is behavior-preserving in code, but
the auto/telemetry changes must be confirmed on the robot.** Note: the hard I/O floor (2 Lynx hubs'
bulk reads + the Pinpoint I2C read) puts the realistic sustained loop ceiling at **~120 Hz, not 200 Hz**
— use the new meter to read the true number on this robot.

### Applied ✅ (code + JVM tests)
- **Loop-time meter** — `BaseTemplate.logLoopTime()` now shows `avg … ms (… Hz) | worst … ms` (worst-case
  exposes GC/telemetry spikes). Always on the driver station, independent of the debug flag below.
- **One `scheduler.run()` and one `follower.update()` per loop** (was 2× / up to 4× in auto).
- **Teleop drive moved out of the shared loop** into `DriverControlled.cycle()`, so autos no longer run
  `face()` + `setTeleOpDrive()`.
- **Tier-3 micro-opts** (bit-identical, tested): cached turret `TrapezoidalProfile`, launcher velocity
  sampled once in `read()`, `ShooterModel.horizontalExitSpeed…` de-duplicated, cached `Subsystems.all()`.

### New setup knob 🟢 — `Globals.DEBUG_TELEMETRY` (`Globals.kt`, default **false**)
- Matches: **leave OFF.** Telemetry goes to the driver station only (no FTC Dashboard/Panels sink) and
  transmits at 100 ms; diagnostic lines are suppressed. The loop meter still shows.
- Tuning/pit: **set it ON from the dashboard BEFORE init** to wire the Dashboard/Panels sink back in
  (20 ms transmit) and stream all the diagnostic lines the tuning OpModes and subsystems emit.

### Must verify on the robot 🟡
- [ ] Re-run **every autonomous** end-to-end — the single-`follower.update()` change alters the auto
      motion path (once-per-loop is Pedro's intended cadence, but confirm tracking + segment hand-offs).
- [ ] TeleOp: field-centric drive + turret goal-lock still behave (drive relocated to `cycle()`).
- [ ] Read the loop meter (avg + worst) in TeleOp and in auto-during-path; confirm auto dropped toward
      the ~120 Hz floor.
- [ ] With `DEBUG_TELEMETRY = false`, confirm the Dashboard shows nothing but the driver station still
      shows the loop meter; flip it ON and confirm the Dashboard graphs return.

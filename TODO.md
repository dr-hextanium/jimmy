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
Digital device | Bottom beam-break | `bbb` | `Names.kt:38`
Digital device | Middle beam-break | `mbb` | `Names.kt:39`
Digital device | Top beam-break | `tbb` | `Names.kt:40`
Analog input | Turret 12T encoder | `te12` | `Names.kt:46`
Analog input | Turret 13T encoder | `te13` | `Names.kt:47`
I2C (Pinpoint) | Odometry computer | `pinpoint` | `Constants.java:48`

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

### Turret motor — `hardware/subsystem/Turret.kt:76` (`FORWARD`) + gear-mesh sign, see §3.

### Intake motor — `hardware/subsystem/Intake.kt:27` (`REVERSE`)
- [ ] Run **"Intake + Beam Break Debug"** (right trigger = in) and confirm it pulls artifacts *in*.

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
`Turret.kt:263-264` — `ENCODER_12T_ZERO_OFFSET_DEG`, `ENCODER_13T_ZERO_OFFSET_DEG` (default `0.0`).
- [ ] Run **"Turret Encoder Debug"**. Move the turret to its true mechanical **zero**, read the two
      "raw deg" values, and set the offsets to those numbers.
- [ ] **Gear-mesh sign:** in the same tool, rotate the turret in your **positive** direction and
      confirm the *fused* angle **increases**. If it decreases, the sign assumption in
      `fuseAbsoluteAngle` is wrong (ratios at `Turret.kt:257-258`) — see the note at `Turret.kt:233-235`.
      Do not trust the turret for motion until the fused angle tracks correctly through the full
      `MIN_ANGLE..MAX_ANGLE` sweep.
- [ ] Confirm `ENCODER_MAX_VOLTAGE` (`Turret.kt:255`, default `3.3`) matches your analog encoders.
- [ ] Confirm `MOTOR_TICKS_PER_REV` (`Turret.kt:248`, `145.1`) and `TURRET_GEAR_RATIO`
      (`Turret.kt:251`, `137/15`) match the actual motor and gear counts.
- [ ] Confirm the idler gear ratios `ENCODER_12T_GEAR_RATIO` (`137/12`) and `ENCODER_13T_GEAR_RATIO`
      (`137/13`) at `Turret.kt:257-258` match the rebuilt turret's tooth counts. The vernier/CRT
      decode in `fuseAbsoluteAngle` relies on these two being **coprime**; if either idler or the
      turret gear changed, the whole absolute-angle fusion must be re-derived, not just re-tuned.

### Turret travel limits 🔴 (prevents winding up cables / hitting hard stops)
`Turret.kt:266-267` — `MAX_ANGLE` (`90.0`), `MIN_ANGLE` (`-90.0`).
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

### Turret feedforward + PID — `Turret.kt:272-282`
- [ ] `MAX_VELOCITY` (`700.0` deg/s), `MAX_ACCELERATION` (`3600.0` deg/s²) — motion-profile limits.
      Start conservative, raise until it tracks without stalling/overshoot.
- [ ] `kV` (`0.0012` ≈ 1/MAX_VELOCITY), `kA` (`0.0`), `kP` (`0.038`), `kD` (`0.0`, encoder is noisy),
      `kStatic` (`0.02`). Tune feedforward (`kV`, then `kStatic`) first, then trim with `kP`.
- [ ] `ANGLE_TOLERANCE_DEGREES` (`Turret.kt:282`, `0.2`) — "at target" band for `isAtTarget()`.

### Launcher velocity loop — `Launcher.kt:132-134`
Feedforward-first (`kS + kV·targetTPS + kP·error`, clamped 0..1). Do **not** make it
proportional-dominant.
- [ ] `kV` (`0.0004` ≈ 1/MAX_TPS) — set so `kV·targetTPS` alone roughly holds the target speed.
- [ ] `kS` (`0.0`) — static offset; `kP` (`0.0003`) — small trim only.
- [ ] `AT_SPEED_TOLERANCE` (`Launcher.kt:129`, `0.05` = 5%) and `MIN_TPS` (`:124`, `100.0`).
- [ ] Current alert is set to `15.0 A` in `Launcher.kt:84` — adjust for your motors if needed.

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

Autonomous start poses:
- [ ] `Robot.kt:125` sets a default `follower.setStartingPose(Pose(0.0, 0.0, 0.0))`. Each auto in
      `opmode/auto/*` (via `AutoTemplate`) should set its **real** starting pose on the field; verify
      each routine you plan to run starts where the robot is physically placed.

---

## 6. Final sanity pass

- [ ] **"Robot Status Debug"** with the robot on the field: pose, distance-to-goal, turret fused
      angle, launcher TPS/at-speed, beam-breaks, hood/gate positions all read sane.
- [ ] **"Intake + Beam Break Debug"**: each beam-break flips "artifact present" only when actually
      blocked (verify wiring/polarity of `bbb`/`mbb`/`tbb`).
- [ ] Full JVM test suite still green: `./gradlew :TeamCode:testDebugUnitTest` (JDK 17–21 / the
      embedded JBR — **not** the default JDK 24).

---

> **Note:** the subsystem `companion object`s hold a few more constants not listed above
> (`POWER_UPDATE_THRESHOLD`, `MAX_DT`, intake `POWER_INTAKE`/`POWER_REVERSE`, `MIN_TPS`, etc.). Those
> are algorithm defaults, **not** robot-specific — leave them unless a specific problem points at one.
> The lists above cover every value that depends on the physical robot.

---

### Not addressed here (decide separately)
- The four `Old` variant files (`opmode/auto/BlueClose12Old.kt`, `RedClose12Old.kt`,
  `paths/BlueClose12OldPaths.java`, `paths/BlueClose12PathsOld.java`) are whole historical files, not
  commented-out code, so this cleanup left them in place. Delete them if they're truly obsolete —
  your call.
- Battery-voltage monitoring was removed from `Robot.kt` (it was fully dead — sampled nowhere). Re-add
  a `VoltageSensor` read in `Robot.read()` if you want brownout/voltage-comp telemetry on the new robot.
- The `limelight` device grab was removed from `Robot.kt`. Re-add it if the new robot has vision.

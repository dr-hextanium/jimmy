package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.control.ComplementaryFilter
import org.firstinspires.ftc.teamcode.control.FadingMemoryFilter
import org.firstinspires.ftc.teamcode.control.ShooterModel
import org.firstinspires.ftc.teamcode.control.TimeSource
import org.firstinspires.ftc.teamcode.control.TrapezoidalProfile
import org.firstinspires.ftc.teamcode.hardware.Globals
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.Robot
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.round
import kotlin.math.sign

/**
 * Turret position comes from two absolute analog encoders meshed onto the 137t turret
 * gear via 12t and 13t idler gears (sensing only -- neither idler drives the turret).
 * A single one of these aliases every ~31.5 degrees of turret rotation, so [fuseAbsoluteAngle]
 * combines both (a vernier/nonius decode exploiting that 12 and 13 are coprime) to recover
 * one unambiguous absolute angle. See that function for the derivation.
 *
 * Control is a trapezoidal-profiled feedforward+PID law. Each loop a [TrapezoidalProfile] advances
 * a smooth setpoint toward [targetAngle] under velocity/acceleration limits (so both large step
 * commands and continuous goal-lock tracking stay bounded), and the motor is driven from that
 * setpoint with velocity/acceleration feedforward plus position feedback. When aiming at the goal,
 * [face] leads a moving target using the real chassis velocity and the shooter's own computed
 * ball speed for time-of-flight, so aim and shot share one physics model.
 *
 * The analog encoders are noisy, and the vernier decode's revolution pick amplifies that noise ~13x
 * before rounding -- so a bad pick would snap the reported angle by a full ~31.5 deg seam. Two guards
 * handle this (see [update]): after an initial memoryless [fuseAbsoluteAngle] acquisition, the angle
 * is tracked by *continuity* ([selectRevolutionByContinuity] -- the turret can't cross a seam in one
 * loop, so the revolution can't legitimately jump), and the result is smoothed by a
 * [FadingMemoryFilter] that also yields a clean velocity for the kD term.
 *
 * Optionally ([USE_MOTOR_FUSION], default off), the drive motor's own high-resolution quadrature
 * encoder is fused in via a [ComplementaryFilter]: the motor tick delta (measured, low-noise, and
 * accumulated in hardware even through loop stalls) predicts motion and the absolute decode corrects
 * the drift, giving near-zero-lag position, a clean tachometer velocity, and stronger revolution
 * disambiguation. It is opt-in because a wrong [MOTOR_ANGLE_SIGN] is runaway-class (the fused angle
 * would chase a phantom); confirm the sign on-robot in "Turret Encoder Debug" before enabling. A
 * health monitor auto-reverts to the absolute-only path if the motor and absolute persistently
 * disagree in sign.
 */
@Configurable
class Turret(
    val motor: DcMotorEx,
    val encoder12: AnalogInput,
    val encoder13: AnalogInput,
    private val timeSource: TimeSource = TimeSource.SYSTEM,
) : ISubsystem {
    // Publicly readable, internally written. Exposed (private set) for observability/telemetry
    // and unit tests; external callers still change the target only through setTargetAngle().
    var currentAngle: Double = 0.0
        private set
    var targetAngle: Double = 0.0
        private set
    var motorPower: Double = 0.0
        private set

    // Profiled setpoint the control law actually tracks (rate/accel limited toward targetAngle).
    var profiledPosition: Double = 0.0
        private set
    var profiledVelocity: Double = 0.0
        private set

    private var lastWritePower: Double = 0.0

    // Motor-implied turret angle (deg) from the drive motor's own quadrature encoder, relative to the
    // reset zero and sign-corrected. Diagnostic/telemetry only (compare its DELTA direction against the
    // fused angle to confirm MOTOR_ANGLE_SIGN); the live fusion path uses [rawTicks] directly.
    var motorImpliedAngle: Double = 0.0
        private set

    // Raw idler-encoder voltages latched in read(); decoded + filtered in update() (needs dt).
    private var rawVoltage12: Double = 0.0
    private var rawVoltage13: Double = 0.0

    // Drive-motor encoder state latched in read(). Tick counts accumulate in hardware even across a
    // loop stall, so tick DIFFERENCES are a true (stall-proof) displacement; velocity is the tach.
    private var rawTicks: Int = 0
    private var rawMotorVelocity: Double = 0.0
    private var lastTicks: Int = 0

    // Smooths the fused angle and estimates its rate. Seeded from the memoryless vernier decode on
    // acquisition, then advanced by continuity-tracked measurements. Active when motor fusion is OFF.
    private val angleFilter = FadingMemoryFilter()

    // Fuses the motor tick delta (prediction) with the absolute decode (correction). Active when motor
    // fusion is ON + healthy.
    private val complementary = ComplementaryFilter()

    // Motor-fusion health: latched false (for the session) if the motor and absolute persistently
    // disagree in sign (e.g. a wrong MOTOR_ANGLE_SIGN), reverting to the absolute-only path.
    var motorFusionHealthy: Boolean = true
        private set
    private var fusionSignDisagreeStreak: Int = 0
    // Which estimator ran last loop, so a mid-session switch can hand off the estimate cleanly.
    private var wasUsingFusion: Boolean = false

    // Pre-filter measured angle (vernier on acquisition, continuity while tracking). Public-read as a
    // diagnostic: compare its jitter against the filtered [currentAngle] when tuning ANGLE_FILTER_TAU.
    var measuredAngle: Double = 0.0
        private set

    // Measured turret rate (deg/s) from [angleFilter]; drives the optional kD term. Public-read for
    // telemetry/tests.
    var measuredVelocity: Double = 0.0
        private set

    // False until the first vernier acquisition; also re-armed on a loop stall (see update()).
    var locked: Boolean = false
        private set

    private var lastTime: Double = 0.0
    private var profileInitialized: Boolean = false

    // Cached reference governor. MAX_VELOCITY/MAX_ACCELERATION are @Configurable live-tunables, so the
    // profile is rebuilt only when either changes -- otherwise the hot loop would allocate a
    // TrapezoidalProfile (plus run its two require() checks) every tick. calculate() is a pure function
    // of (dt, current, goal, maxV, maxA) with no instance state, so a reused instance with the same
    // limits is bit-identical to a freshly constructed one.
    private var cachedProfile: TrapezoidalProfile? = null
    private var cachedMaxVelocity: Double = Double.NaN
    private var cachedMaxAcceleration: Double = Double.NaN

    var aimAtGoal = false
    var offset = 0.0

    override fun reset() {
        aimAtGoal = false
        offset = 0.0

        motor.direction = DcMotorSimple.Direction.FORWARD

        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        targetAngle = 0.0
        currentAngle = 0.0
        motorPower = 0.0
        lastWritePower = 0.0

        profiledPosition = 0.0
        profiledVelocity = 0.0
        measuredVelocity = 0.0
        lastTime = timeSource.seconds()
        profileInitialized = false
        locked = false

        lastTicks = 0 // matches STOP_AND_RESET_ENCODER above
        motorFusionHealthy = true
        fusionSignDisagreeStreak = 0
        wasUsingFusion = false

        cachedProfile = null // rebuilt lazily from the current MAX_VELOCITY/MAX_ACCELERATION
    }

    /**
     * The reference-governor [TrapezoidalProfile], rebuilt only when [MAX_VELOCITY]/[MAX_ACCELERATION]
     * (both live-tunable) change. Each limit is read once here so a mid-loop dashboard write can't tear
     * the (maxV, maxA) pair, and the require() checks in the constructor run only on an actual change.
     */
    private fun profile(): TrapezoidalProfile {
        val maxV = MAX_VELOCITY
        val maxA = MAX_ACCELERATION
        var p = cachedProfile
        if (p == null || maxV != cachedMaxVelocity || maxA != cachedMaxAcceleration) {
            p = TrapezoidalProfile(maxV, maxA)
            cachedProfile = p
            cachedMaxVelocity = maxV
            cachedMaxAcceleration = maxA
        }
        return p
    }

    override fun read() {
        // Pull hardware only -- decode + filtering happen in update(), which owns dt.
        rawTicks = motor.currentPosition
        rawMotorVelocity = motor.velocity // ticks/s; valid in RUN_WITHOUT_ENCODER (see Launcher)
        motorImpliedAngle = rawTicks * DEG_PER_TICK * MOTOR_ANGLE_SIGN
        rawVoltage12 = encoder12.voltage
        rawVoltage13 = encoder13.voltage
    }

    override fun update() {
        if (aimAtGoal) {
            val goalPose = if (Globals.isRed ?: true) Globals.RED_GOAL_POSE else Globals.BLUE_GOAL_POSE
            face(goalPose, Robot.follower.pose, Robot.follower.velocity)
        }

        val enc12Deg = wrapTo360(voltageToDegrees(rawVoltage12) - ENCODER_12T_ZERO_OFFSET_DEG)
        val enc13Deg = wrapTo360(voltageToDegrees(rawVoltage13) - ENCODER_13T_ZERO_OFFSET_DEG)

        val now = timeSource.seconds()
        // On the first tick after a reset, collapse the elapsed time to zero so a long init->loop gap
        // can't register as one huge dt (which would re-acquire and lurch the profile).
        if (!profileInitialized) lastTime = now
        // rawElapsed drives the motor-fusion path (tick deltas are valid across a stall); dt is the
        // profile's clamped step (a big gap is treated as a hold, never integrated).
        val rawElapsed = now - lastTime
        lastTime = now
        var dt = rawElapsed
        if (dt <= 0.0 || dt > MAX_DT) dt = 0.0

        // If the active estimator changed since last loop (a dashboard toggle of USE_MOTOR_FUSION, or
        // the health monitor latching fusion off), hand the current estimate to the newly-active filter
        // so its continuity prior is correct.
        val effectiveFusion = USE_MOTOR_FUSION && motorFusionHealthy
        if (locked && effectiveFusion != wasUsingFusion) {
            if (effectiveFusion) { complementary.reset(currentAngle); lastTicks = rawTicks }
            else angleFilter.reset(currentAngle)
        }
        wasUsingFusion = effectiveFusion

        if (effectiveFusion) {
            updateAngleWithMotorFusion(enc12Deg, enc13Deg, rawElapsed)
        } else {
            updateAngleAbsoluteOnly(enc12Deg, enc13Deg, dt)
        }

        // On the first tick after a reset, seed the profile where the turret actually is so it never
        // lurches toward a stale setpoint.
        if (!profileInitialized) {
            profiledPosition = currentAngle
            profiledVelocity = 0.0
            profileInitialized = true
        }

        var profileAccel = 0.0
        if (dt > 0.0) {
            val profile = profile()
            val next = profile.calculate(
                dt,
                TrapezoidalProfile.State(profiledPosition, profiledVelocity),
                TrapezoidalProfile.State(targetAngle, 0.0),
            )
            profileAccel = (next.velocity - profiledVelocity) / dt
            profiledPosition = next.position
            profiledVelocity = next.velocity
        }

        val posError = profiledPosition - currentAngle
        val raw = kV * profiledVelocity +
            kA * profileAccel +
            kP * posError +
            kD * (profiledVelocity - measuredVelocity)
        // Static-friction feedforward only when we're actually commanding effort, to avoid chatter
        // about zero when the turret is settled on target.
        val staticFF = if (abs(raw) > 1e-6) kStatic * sign(raw) else 0.0

        motorPower = Range.clip(raw + staticFF, -1.0, 1.0)
    }

    /**
     * Absolute-only angle estimate (motor fusion OFF): the shipped, tested path. dt == 0 (first tick
     * or a stall) re-acquires from the memoryless vernier and re-seeds the filter; otherwise track by
     * continuity and smooth with the [FadingMemoryFilter].
     */
    private fun updateAngleAbsoluteOnly(enc12Deg: Double, enc13Deg: Double, dt: Double) {
        if (!locked || dt == 0.0) {
            measuredAngle = fuseAbsoluteAngle(enc12Deg, enc13Deg)
            angleFilter.reset(measuredAngle)
            locked = true
        } else {
            measuredAngle = selectRevolutionByContinuity(enc12Deg, angleFilter.position)
            angleFilter.update(dt, measuredAngle, ANGLE_FILTER_TAU, ANGLE_FILTER_SPIKE_GATE)
        }
        currentAngle = angleFilter.position
        measuredVelocity = angleFilter.velocity
    }

    /**
     * Motor-fused angle estimate (motor fusion ON). Cold start (`!locked`) acquires from the memoryless
     * vernier -- the only motor-independent ground truth (the turret may have been back-driven while
     * disabled). Every subsequent loop, INCLUDING a large-dt stall, is carried by the motor: the tick
     * delta (measured, stall-proof) predicts, and the absolute decode corrects the drift. So the noisy
     * 13t encoder is only ever touched at cold start, not on stalls.
     */
    private fun updateAngleWithMotorFusion(enc12Deg: Double, enc13Deg: Double, rawElapsed: Double) {
        if (!locked) {
            measuredAngle = fuseAbsoluteAngle(enc12Deg, enc13Deg)
            complementary.reset(measuredAngle)
            lastTicks = rawTicks // measure the next delta from the acquire point (no double-count)
            locked = true
        } else {
            // Clamp against a physical bound (scaled by the RAW elapsed, so a legitimate stall
            // displacement isn't clipped while a tick glitch/overflow can't alias across a ~31.5 deg
            // seam and corrupt the revolution pick).
            val maxMotion = MAX_VELOCITY * rawElapsed * MOTION_CLAMP_MARGIN
            val motionDelta = ((rawTicks - lastTicks) * DEG_PER_TICK * MOTOR_ANGLE_SIGN)
                .coerceIn(-maxMotion, maxMotion)
            val priorPos = complementary.position
            val predicted = priorPos + motionDelta
            measuredAngle = selectRevolutionByContinuity(enc12Deg, predicted) // motor-predicted prior
            // dt-aware blend so behaviour is loop-rate invariant (larger tau trusts the motor more).
            val alpha = if (rawElapsed > 0.0) 1.0 - exp(-rawElapsed / MOTOR_FUSION_TAU) else 0.0
            complementary.update(rawElapsed, motionDelta, measuredAngle, alpha, MOTOR_FUSION_GATE)
            lastTicks = rawTicks
            monitorFusionHealth(motionDelta, measuredAngle - priorPos)
        }
        currentAngle = complementary.position
        measuredVelocity = rawMotorVelocity * DEG_PER_TICK * MOTOR_ANGLE_SIGN // clean tachometer rate
    }

    /**
     * Wrong-sign safety net. When both the motor motion and the absolute's implied motion are
     * meaningful, they must share a sign; a persistent disagreement means the motor is fighting the
     * truth (most likely a wrong [MOTOR_ANGLE_SIGN]), so latch fusion off for the session and let the
     * estimator switch in [update] hand back to the absolute-only path. Prevents runaway from a
     * mis-calibrated sign that slipped past the on-robot check.
     */
    private fun monitorFusionHealth(motionDelta: Double, absoluteImpliedDelta: Double) {
        if (abs(motionDelta) < MOTOR_HEALTH_MIN_DELTA_DEG ||
            abs(absoluteImpliedDelta) < MOTOR_HEALTH_MIN_DELTA_DEG
        ) return

        if (sign(motionDelta) != sign(absoluteImpliedDelta)) {
            fusionSignDisagreeStreak++
            if (fusionSignDisagreeStreak >= MOTOR_HEALTH_MAX_DISAGREE) motorFusionHealthy = false
        } else {
            fusionSignDisagreeStreak = 0
        }
    }

    override fun write() {
        if (abs(motorPower - lastWritePower) > POWER_UPDATE_THRESHOLD) {
            motor.power = motorPower
            lastWritePower = motorPower
        }
    }

    fun setTargetAngle(degrees: Double) {
        targetAngle = Range.clip(degrees, MIN_ANGLE, MAX_ANGLE)
    }

    fun getAngle(): Double {
        return currentAngle
    }

    fun isAtTarget(): Boolean {
        return abs(targetAngle - currentAngle) < ANGLE_TOLERANCE_DEGREES
    }

    fun distance(targetPose: Vector, robotPose: Pose): Double {
        return hypot(targetPose.xComponent - robotPose.x, targetPose.yComponent - robotPose.y)
    }

    /**
     * Aim the turret at [targetPose], leading a moving target (shoot-on-the-move). The lead uses
     * the field-frame chassis velocity and the ball's horizontal exit speed from [ShooterModel]
     * (the same physics the launcher uses) for time-of-flight, so the turret and shooter can never
     * disagree about how fast the ball leaves. With zero [robotVelocity] this reduces to a straight
     * aim at the goal.
     */
    fun face(targetPose: Vector, robotPose: Pose, robotVelocity: Vector) {
        val distance = distance(targetPose, robotPose)
        val ballSpeed = ShooterModel.horizontalExitSpeedInchesPerSec(distance)
        val timeOfFlight = if (ballSpeed > 1e-6) distance / ballSpeed else 0.0

        val virtualGoalX = targetPose.xComponent - (robotVelocity.xComponent * timeOfFlight)
        val virtualGoalY = targetPose.yComponent - (robotVelocity.yComponent * timeOfFlight)

        val angleToTargetFromCenter = atan2(
            virtualGoalY - robotPose.y,
            virtualGoalX - robotPose.x
        )

        val globalTargetDegrees = Math.toDegrees(angleToTargetFromCenter)
        val robotHeadingDegrees = Math.toDegrees(robotPose.heading)

        val robotAngleDiff = normalizeAngle(globalTargetDegrees - robotHeadingDegrees + offset)

        setTargetAngle(robotAngleDiff)

        Robot.debug("robot heading (deg)", robotHeadingDegrees)
        Robot.debug("Corrected Global Target (deg)", globalTargetDegrees)
        Robot.debug("Final Turret Target (deg)", robotAngleDiff)
    }

    private fun normalizeAngle(degrees: Double): Double {
        var angle = degrees
        while (angle > 180) angle -= 360
        while (angle <= -180) angle += 360
        return angle
    }

    private fun wrapTo360(degrees: Double): Double {
        var angle = degrees % 360.0
        if (angle < 0) angle += 360.0
        return angle
    }

    private fun voltageToDegrees(voltage: Double): Double = (voltage / ENCODER_MAX_VOLTAGE) * 360.0

    /**
     * Vernier/nonius decode: the 12t and 13t idler encoders each alias many times per turret
     * revolution (137/12 =~ 11.42 and 137/13 =~ 10.54 idler revs per turret rev), but since
     * 12 and 13 are coprime, their phase *difference* only completes one cycle every
     * 360 / (ENCODER_12T_GEAR_RATIO - ENCODER_13T_GEAR_RATIO) =~ 410 degrees of turret
     * rotation -- comfortably covering MIN_ANGLE..MAX_ANGLE. That difference gives a coarse,
     * unambiguous angle; we then pick the matching revolution of the (higher-resolution) 12t
     * encoder and solve for the precise angle from it directly.
     *
     * NOT YET VALIDATED ON HARDWARE: gear-mesh direction (sign of the ratios below) and the
     * ENCODER_12T_ZERO_OFFSET_DEG/ENCODER_13T_ZERO_OFFSET_DEG calibration constants need to be
     * confirmed/tuned on the real turret before trusting this for motion.
     */
    private fun fuseAbsoluteAngle(encoder12Deg: Double, encoder13Deg: Double): Double {
        val phaseDiff = normalizeAngle(encoder12Deg - encoder13Deg)
        val coarseAngle = phaseDiff / (ENCODER_12T_GEAR_RATIO - ENCODER_13T_GEAR_RATIO)

        val nearestRevolution = round(((coarseAngle * ENCODER_12T_GEAR_RATIO) - encoder12Deg) / 360.0)

        return (encoder12Deg + 360.0 * nearestRevolution) / ENCODER_12T_GEAR_RATIO
    }

    /**
     * Tracking counterpart to [fuseAbsoluteAngle]: given a fresh 12t reading and the last known
     * [priorAngle], pick the 12t revolution whose decoded angle sits nearest [priorAngle]. The 12t
     * encoder aliases every ~31.5 deg of turret travel, so one reading is ambiguous by whole seams;
     * the turret physically cannot move a full seam between control loops, so the revolution nearest
     * the previous angle is always the right one. Unlike [fuseAbsoluteAngle] this ignores the noisy
     * 13t encoder entirely, so it can't suffer the ~13x-amplified revolution flip -- at the cost of
     * needing a valid prior (established by an initial [fuseAbsoluteAngle] acquisition).
     */
    private fun selectRevolutionByContinuity(encoder12Deg: Double, priorAngle: Double): Double {
        val nearestRevolution = round((priorAngle * ENCODER_12T_GEAR_RATIO - encoder12Deg) / 360.0)
        return (encoder12Deg + 360.0 * nearestRevolution) / ENCODER_12T_GEAR_RATIO
    }

    companion object {
        // goBILDA 1150 RPM (5.2:1) Yellow Jacket motor: 145.1 encoder ticks per output-shaft rev.
        private const val MOTOR_TICKS_PER_REV = 145.1

        // 137t turret gear driven by a 15t motor pinion.
        private const val TURRET_GEAR_RATIO = 137.0 / 15.0
        private const val TICKS_PER_TURRET_REV = MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO
        // Public so the TurretAutoTune characterization OpMode shares this exact conversion (its
        // motor-tick safety limit and velocity readings must match the live subsystem's geometry).
        const val DEG_PER_TICK = 360.0 / TICKS_PER_TURRET_REV // ~0.272 deg/turret-tick

        // Absolute analog position encoders (Melonbotics, 0-3.3V per revolution).
        const val ENCODER_MAX_VOLTAGE = 3.3

        private const val ENCODER_12T_GEAR_RATIO = 137.0 / 12.0 // idler revs per turret rev
        private const val ENCODER_13T_GEAR_RATIO = 137.0 / 13.0

        // Raw encoder angle (deg) when the turret sits at its true zero position. Must be
        // calibrated on-robot: home the turret, read the raw encoder voltages, convert with
        // voltageToDegrees(), and set these to the results.
        var ENCODER_12T_ZERO_OFFSET_DEG = 0.0
        var ENCODER_13T_ZERO_OFFSET_DEG = 0.0

        // Fused-angle smoothing (FadingMemoryFilter). ANGLE_FILTER_TAU is the smoothing time constant
        // in seconds: larger = steadier position but more tracking lag (the turret goal-locks a MOVING
        // target, so keep it small). ANGLE_FILTER_SPIKE_GATE (deg) rejects noise spikes -- real turret
        // motion is only a few deg per loop, so anything larger is noise. Both are on-robot tunables.
        var ANGLE_FILTER_TAU = 0.02          // s
        var ANGLE_FILTER_SPIKE_GATE = 10.0   // deg

        // Drive-motor encoder fusion (ComplementaryFilter). OPT-IN: a wrong MOTOR_ANGLE_SIGN is
        // runaway-class, so this defaults OFF and must be enabled on-robot only AFTER confirming the
        // sign in "Turret Encoder Debug" (jog positive -> both the fused-absolute angle and the motor
        // prediction must increase). When on, motor ticks predict motion and the absolute corrects it.
        var USE_MOTOR_FUSION = false
        var MOTOR_ANGLE_SIGN = 1.0           // +1/-1: motor-tick direction vs. turret positive
        var MOTOR_FUSION_TAU = 0.10          // s crossover; larger = trust the motor more (slower correct)
        var MOTOR_FUSION_GATE = 15.0         // deg; must sit ABOVE gear backlash and BELOW one seam (~31.5)
        var MOTION_CLAMP_MARGIN = 2.0        // motionDelta clamp = MAX_VELOCITY * elapsed * this
        // Health monitor: latch fusion off after this many consecutive loops where the motor and the
        // absolute disagree in sign (only counted when both moved more than MIN_DELTA in the loop).
        var MOTOR_HEALTH_MIN_DELTA_DEG = 0.5
        var MOTOR_HEALTH_MAX_DISAGREE = 12

        // ~+/-180 physical travel; run +/-150 in code to keep margin off the hard stops.
        var MAX_ANGLE = 150.0
        var MIN_ANGLE = -150.0

        // Motion-profile limits. Default max velocity is roughly the 1150 RPM motor's free speed
        // through the 137:15 reduction (~755 deg/s); acceleration is a generous starting guess.
        // Both are on-robot tunables.
        var MAX_VELOCITY = 700.0      // deg/s
        var MAX_ACCELERATION = 3600.0 // deg/s^2

        // Profiled feedforward + feedback gains (on-robot tunables).
        var kP = 0.038      // power per deg of position error
        var kV = 0.0012     // power per deg/s of profiled velocity (~1 / MAX_VELOCITY)
        var kA = 0.0        // power per deg/s^2 of profiled acceleration
        var kD = 0.0        // power per deg/s of velocity error (off by default; velocity comes from the
                            // FadingMemoryFilter, or the motor tachometer when USE_MOTOR_FUSION is on
                            // -- cleaner -- so enable + tune on robot for extra damping)
        var kStatic = 0.02  // static-friction feedforward

        var ANGLE_TOLERANCE_DEGREES = 0.2
        var POWER_UPDATE_THRESHOLD = 0.01

        // Largest loop dt (s) we'll integrate the profile over; longer gaps are treated as a hold.
        private const val MAX_DT = 0.1
    }
}

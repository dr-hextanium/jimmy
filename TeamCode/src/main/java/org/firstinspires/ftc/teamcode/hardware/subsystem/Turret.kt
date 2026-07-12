package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.control.ShooterModel
import org.firstinspires.ftc.teamcode.control.TimeSource
import org.firstinspires.ftc.teamcode.control.TrapezoidalProfile
import org.firstinspires.ftc.teamcode.hardware.Globals
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.Robot
import kotlin.math.abs
import kotlin.math.atan2
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
 */
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

    // Motor's own relative encoder isn't used for position (the absolute encoders are the
    // source of truth), kept only as a diagnostic cross-check against the fused angle.
    private var motorImpliedAngle: Double = 0.0

    // Measured turret rate (deg/s), differentiated from the fused angle for the optional kD term.
    private var measuredVelocity: Double = 0.0
    private var lastAngle: Double = 0.0
    private var lastTime: Double = 0.0
    private var profileInitialized: Boolean = false

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
        lastAngle = 0.0
        lastTime = timeSource.seconds()
        profileInitialized = false
    }

    override fun read() {
        motorImpliedAngle = (motor.currentPosition / TICKS_PER_TURRET_REV) * 360.0

        val encoder12Deg = wrapTo360(voltageToDegrees(encoder12.voltage) - ENCODER_12T_ZERO_OFFSET_DEG)
        val encoder13Deg = wrapTo360(voltageToDegrees(encoder13.voltage) - ENCODER_13T_ZERO_OFFSET_DEG)

        currentAngle = fuseAbsoluteAngle(encoder12Deg, encoder13Deg)
    }

    override fun update() {
        if (aimAtGoal) {
            val goalPose = if (Globals.isRed ?: true) Globals.RED_GOAL_POSE else Globals.BLUE_GOAL_POSE
            face(goalPose, Robot.follower.pose, Robot.follower.velocity)
        }

        // On the first tick after a reset, start the profile where the turret actually is so it
        // never lurches toward a stale setpoint, and skip integrating this (unknown-length) step.
        if (!profileInitialized) {
            profiledPosition = currentAngle
            profiledVelocity = 0.0
            lastAngle = currentAngle
            lastTime = timeSource.seconds()
            profileInitialized = true
        }

        val now = timeSource.seconds()
        var dt = now - lastTime
        lastTime = now
        // Ignore non-positive or absurdly large steps (paused loop, first frame): hold, don't lurch.
        if (dt <= 0.0 || dt > MAX_DT) dt = 0.0

        measuredVelocity = if (dt > 0.0) (currentAngle - lastAngle) / dt else measuredVelocity
        lastAngle = currentAngle

        var profileAccel = 0.0
        if (dt > 0.0) {
            val profile = TrapezoidalProfile(MAX_VELOCITY, MAX_ACCELERATION)
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

        Robot.telemetry.addData("robot heading (deg)", robotHeadingDegrees)
        Robot.telemetry.addData("Corrected Global Target (deg)", globalTargetDegrees)
        Robot.telemetry.addData("Final Turret Target (deg)", robotAngleDiff)
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

    companion object {
        // goBILDA 1150 RPM (5.2:1) Yellow Jacket motor: 145.1 encoder ticks per output-shaft rev.
        private const val MOTOR_TICKS_PER_REV = 145.1

        // 137t turret gear driven by a 15t motor pinion.
        private const val TURRET_GEAR_RATIO = 137.0 / 15.0
        private const val TICKS_PER_TURRET_REV = MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO

        // Absolute analog position encoders (Melonbotics, 0-3.3V per revolution).
        const val ENCODER_MAX_VOLTAGE = 3.3

        private const val ENCODER_12T_GEAR_RATIO = 137.0 / 12.0 // idler revs per turret rev
        private const val ENCODER_13T_GEAR_RATIO = 137.0 / 13.0

        // Raw encoder angle (deg) when the turret sits at its true zero position. Must be
        // calibrated on-robot: home the turret, read the raw encoder voltages, convert with
        // voltageToDegrees(), and set these to the results.
        var ENCODER_12T_ZERO_OFFSET_DEG = 0.0
        var ENCODER_13T_ZERO_OFFSET_DEG = 0.0

        var MAX_ANGLE = 90.0
        var MIN_ANGLE = -90.0

        // Motion-profile limits. Default max velocity is roughly the 1150 RPM motor's free speed
        // through the 137:15 reduction (~755 deg/s); acceleration is a generous starting guess.
        // Both are on-robot tunables.
        var MAX_VELOCITY = 700.0      // deg/s
        var MAX_ACCELERATION = 3600.0 // deg/s^2

        // Profiled feedforward + feedback gains (on-robot tunables).
        var kP = 0.038      // power per deg of position error
        var kV = 0.0012     // power per deg/s of profiled velocity (~1 / MAX_VELOCITY)
        var kA = 0.0        // power per deg/s^2 of profiled acceleration
        var kD = 0.0        // power per deg/s of velocity error (off by default; encoder is noisy)
        var kStatic = 0.02  // static-friction feedforward

        var ANGLE_TOLERANCE_DEGREES = 0.2
        var POWER_UPDATE_THRESHOLD = 0.01

        // Largest loop dt (s) we'll integrate the profile over; longer gaps are treated as a hold.
        private const val MAX_DT = 0.1
    }
}

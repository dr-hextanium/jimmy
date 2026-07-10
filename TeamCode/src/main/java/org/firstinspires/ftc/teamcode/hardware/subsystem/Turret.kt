package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range
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
 */
class Turret(val motor: DcMotorEx, val encoder12: AnalogInput, val encoder13: AnalogInput) : ISubsystem {
    // Publicly readable, internally written. Exposed (private set) for observability/telemetry
    // and unit tests; external callers still change the target only through setTargetAngle().
    var currentAngle: Double = 0.0
        private set
    var targetAngle: Double = 0.0
        private set
    var motorPower: Double = 0.0
        private set
    private var lastWritePower: Double = 0.0

    // Motor's own relative encoder isn't used for position (the absolute encoders are the
    // source of truth), kept only as a diagnostic cross-check against the fused angle.
    private var motorImpliedAngle: Double = 0.0

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
            // Robot velocity left at zero for now (no lead correction yet) -- feeding a real
            // velocity vector here is a tuning follow-up, not part of re-enabling the turret.
            face(goalPose, Robot.follower.pose, Vector())
        }

        val error = targetAngle - currentAngle

        if (abs(error) < ANGLE_TOLERANCE_DEGREES) {
            motorPower = 0.0
        } else {
            var output = error * kP

            output += sign(error) * kStatic

            motorPower = Range.clip(output, -1.0, 1.0)
        }
    }

    override fun write() {
    //        Robot.telemetry.addData("current turret angle", currentAngle)
    //        Robot.telemetry.addData("target turret angle", getAngle())
    //        Robot.telemetry.addData("turret motor power", motorPower)
    //        Robot.telemetry.addData("aiming at goal", aimAtGoal)
    //        Robot.telemetry.addData("turret offset", offset)
    //        Robot.telemetry.addData("turret at target", isAtTarget())

        if (abs(motorPower - lastWritePower) > POWER_UPDATE_THRESHOLD) {
            motor.power = motorPower
            lastWritePower = motorPower
        }
    }

    fun setTargetAngle(degrees: Double) {
        targetAngle = Range.clip(degrees, MIN_ANGLE, MAX_ANGLE)
    }

//    fun setTargetAngle(degrees: Double) {
//        targetAngle = if (degrees in MIN_ANGLE..MAX_ANGLE) {
//            // If the angle is within the valid range, use it.
//            degrees
//        } else {
//            // If the angle is out of range, default to the zero position.
//            0.0
//        }
//    }

    fun getAngle(): Double {
        return currentAngle
    }

    fun isAtTarget(): Boolean {
        return abs(targetAngle - currentAngle) < ANGLE_TOLERANCE_DEGREES
    }

    fun distance(targetPose: Vector, robotPose: Pose): Double {
        return hypot(targetPose.xComponent - robotPose.x, targetPose.yComponent - robotPose.y)
    }

    fun face(targetPose: Vector, robotPose: Pose, robotVelocity: Vector) {
        val distance = distance(targetPose, robotPose)
        val timeOfFlight = distance / SHOOT_SPEED

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
        private const val TICKS_PER_DEGREE = TICKS_PER_TURRET_REV / 360.0

        // Absolute analog position encoders (Melonbotics, 0-3.3V per revolution).
        const val ENCODER_MAX_VOLTAGE = 3.3

        private const val ENCODER_12T_GEAR_RATIO = 137.0 / 12.0 // idler revs per turret rev
        private const val ENCODER_13T_GEAR_RATIO = 137.0 / 13.0

        // Raw encoder angle (deg) when the turret sits at its true zero position. Must be
        // calibrated on-robot: home the turret, read the raw encoder voltages, convert with
        // voltageToDegrees(), and set these to the results.
        var ENCODER_12T_ZERO_OFFSET_DEG = 0.0
        var ENCODER_13T_ZERO_OFFSET_DEG = 0.0

        var SHOOT_SPEED = 180.0

        var MAX_ANGLE = 90.0
        var MIN_ANGLE = -90.0

        var kP = 0.038
        var kStatic = 0.02

        var ANGLE_TOLERANCE_DEGREES = 0.2
        var POWER_UPDATE_THRESHOLD = 0.01
    }
}
package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.Robot
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sign

class Turret(val motor: DcMotorEx) : ISubsystem {
    private var currentAngle: Double = 0.0
    private var targetAngle: Double = 0.0
    private var motorPower: Double = 0.0
    private var lastWritePower: Double = 0.0

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
        val currentTicks = motor.currentPosition
        currentAngle = (currentTicks / TICKS_PER_TURRET_REV) * 360.0
    }

    override fun update() {
//        if (aimAtGoal) {
//            val goalPose = if (Globals.isRed ?: true) Globals.RED_GOAL_POSE else Globals.BLUE_GOAL_POSE
////            face(goalPose, Robot.pose, Robot.follower.velocity)
//            face(goalPose, Robot.follower.pose, Vector())
//        }

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

    companion object {
        private const val MOTOR_TICKS_PER_REV = 384.5
        private const val TURRET_GEAR_RATIO = 135.0 / 24.0
        private const val TICKS_PER_TURRET_REV = MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO
        private const val TICKS_PER_DEGREE = TICKS_PER_TURRET_REV / 360.0 // 6 ticks / deg

        var SHOOT_SPEED = 180.0

        var MAX_ANGLE = 90.0
        var MIN_ANGLE = -90.0

        var kP = 0.038
        var kStatic = 0.02

        var ANGLE_TOLERANCE_DEGREES = 0.2
        var POWER_UPDATE_THRESHOLD = 0.01
    }
}
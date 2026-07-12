package org.firstinspires.ftc.teamcode.opmode.template

import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.pedropathing.control.PIDFCoefficients
import com.pedropathing.control.PIDFController
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions.normalizeAngleSigned
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.command.launcher.ManuallyLaunch
import org.firstinspires.ftc.teamcode.hardware.Globals
import org.firstinspires.ftc.teamcode.hardware.Robot
import kotlin.math.atan2
import kotlin.math.hypot


abstract class BaseTemplate(var initialHeading: Double = 0.0) : OpMode() {
    val primary by lazy { Robot.gamepad1 }
	val secondary by lazy { Robot.gamepad2 }

	var lastTimeStamp = 0.0

    var controller: PIDFController = PIDFController(PIDFCoefficients(1.2, 0.0, 0.0, 0.0))

    var goalLock: Boolean = false
    // Vector(Double, Double) is pedro's POLAR (magnitude, theta) constructor; use Vector(Pose) to
    // get a cartesian field-center default instead of a point ~(-70, 19).
    var targetGoalPose: Vector = Vector(Pose(72.0, 72.0))

    fun setTargetPose(vector: Vector) { targetGoalPose = vector }

	private fun logLoopTime() {
		val now = System.nanoTime().toDouble()
		telemetry.addData("loop time (hz)", 1e9 / (now - lastTimeStamp))
		lastTimeStamp = now
	}

	override fun init() {
        telemetry.msTransmissionInterval = 10

        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2)

        initialize()


		telemetry.addLine("In the initialization phase; start after at least 1 second.")
	}

	override fun start() {
        resetRuntime()

        if (!Globals.AUTO) {
            Robot.scheduler.schedule(
                ManuallyLaunch { 0.71 },
            )

            Robot.follower.startTeleopDrive()
		}
	}

	override fun init_loop() {
		if (!Globals.AUTO) return

		Robot.hubs.forEach { it.clearBulkCache() }

		Robot.read()
		Robot.update()
		Robot.scheduler.run()
		Robot.write()

		logLoopTime()

		telemetry.update()
	}

    fun distance(targetPose: Vector, robotPose: Pose): Double {
        return hypot(targetPose.xComponent - robotPose.x, targetPose.yComponent - robotPose.y)
    }

    fun face(targetPose: Vector, robotPose: Pose) {
        val angleToTargetFromCenter = atan2(
            targetPose.yComponent - robotPose.y,
            targetPose.xComponent - robotPose.x
        ) // radians

//        val angleToTargetFromCenter = atan2(
//            robotPose.y - targetPose.xComponent - ,
//            targetPose.yComponent - robotPose.x
//        ) // radians

        val globalTargetDegrees = angleToTargetFromCenter
        val robotHeadingDegrees = robotPose.heading

        val robotAngleDiff = normalizeAngleSigned(globalTargetDegrees - robotHeadingDegrees) // assumes radian input

        controller.updateError(robotAngleDiff)

        Robot.telemetry.addData("robot heading", Math.toDegrees(robotHeadingDegrees))
        Robot.telemetry.addData("global heading target", Math.toDegrees(globalTargetDegrees))
        Robot.telemetry.addData("delta angle", robotAngleDiff)
    }

	override fun loop() {
		Robot.hubs.forEach { it.clearBulkCache() }

		Robot.read()
		Robot.update()

		cycle()

		Robot.scheduler.run()

        val robotPose = Robot.follower.pose

//        // get the pose to put into this
//        val angleToTargetFromCenter = atan2(targetGoalPose.yComponent - robotPose.y, targetGoalPose.xComponent - robotPose.x)
//        val robotAngleDiff = normalizeAngleSigned(angleToTargetFromCenter - robotPose.heading)

//        controller.updateError(robotAngleDiff)

        face(targetGoalPose, robotPose)

        val angularAdjustment =
            if (goalLock) {
                controller.run()
            } else {
                (-gamepad1.right_stick_x).toDouble()
            }

        Robot.follower.setTeleOpDrive(
            (-gamepad1.left_stick_y).toDouble(),
            (-gamepad1.left_stick_x).toDouble(),
            angularAdjustment,
            false,
            Globals.globalHeadingOffset
        )

        Robot.write()

        logLoopTime()

        telemetry.update()
    }

	abstract fun initialize()

	abstract fun cycle()

	companion object {
		val CROSS = GamepadKeys.Button.A
		val CIRCLE = GamepadKeys.Button.B
		val TRIANGLE = GamepadKeys.Button.Y
		val SQUARE = GamepadKeys.Button.X
	}
}
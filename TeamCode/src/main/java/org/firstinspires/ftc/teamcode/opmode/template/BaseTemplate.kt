package org.firstinspires.ftc.teamcode.opmode.template

import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.pedropathing.control.PIDFCoefficients
import com.pedropathing.control.PIDFController
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.command.launcher.Manual
import org.firstinspires.ftc.teamcode.hardware.Globals
import org.firstinspires.ftc.teamcode.hardware.Robot


abstract class BaseTemplate : OpMode() {
	val primary by lazy { Robot.gamepad1 }
	val secondary by lazy { Robot.gamepad2 }

	var lastTimeStamp = 0.0

    var controller: PIDFController = PIDFController(PIDFCoefficients(0.5, 0.0, 0.0, 0.0))
    var goalLock: Boolean = false

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
                Manual { 0.71 },
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

	override fun loop() {
		Robot.hubs.forEach { it.clearBulkCache() }

		Robot.read()
		Robot.update()

		cycle()

		Robot.scheduler.run()

        val targetPose = Pose(12.0, 138.0)
        val robotPose = Robot.follower.pose

        // get the pose to put into this



//        val angleToTargetFromCenter =
//            atan2(targetPose.y - robotPose.y, targetPose.x - robotPose.x)
//        val robotAngleDiff: Double =
//            normalizeAngle(angleToTargetFromCenter - robotPose.heading)
//
//        controller.updateError(robotAngleDiff)

//        val angularAdjustment =
//            if (false) {
//                controller.run()
//            } else {
//                (-gamepad1.right_stick_x).toDouble()
//            }

        Robot.follower.setTeleOpDrive(
            (-gamepad1.left_stick_y).toDouble(),
            (-gamepad1.left_stick_x).toDouble(),
            (-gamepad1.right_stick_x).toDouble(),
            false
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
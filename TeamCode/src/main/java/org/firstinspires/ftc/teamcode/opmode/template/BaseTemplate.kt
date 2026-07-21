package org.firstinspires.ftc.teamcode.opmode.template

import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.pedropathing.control.PIDFCoefficients
import com.pedropathing.control.PIDFController
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions.normalizeAngleSigned
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.hardware.Globals
import org.firstinspires.ftc.teamcode.hardware.Robot
import kotlin.math.atan2
import kotlin.math.hypot


abstract class BaseTemplate(var initialHeading: Double = 0.0) : OpMode() {
    val primary by lazy { Robot.gamepad1 }
	val secondary by lazy { Robot.gamepad2 }

	var lastTimeStamp = 0.0
	// Loop-time meter state. The first sample is invalid (no prior timestamp), so it's skipped;
	// thereafter we report a running average (sustained rate) and the worst-case single loop, which
	// is the number that actually exposes GC/telemetry jitter. Reset each match in start().
	private var hasPrevLoop = false
	private var loopSumMs = 0.0
	private var loopCount = 0L
	private var loopMaxMs = 0.0

    var controller: PIDFController = PIDFController(PIDFCoefficients(1.2, 0.0, 0.0, 0.0))

    var goalLock: Boolean = false
    // Vector(Double, Double) is pedro's POLAR (magnitude, theta) constructor; use Vector(Pose) to
    // get a cartesian field-center default instead of a point ~(-70, 19).
    var targetGoalPose: Vector = Vector(Pose(72.0, 72.0))

    fun setTargetPose(vector: Vector) { targetGoalPose = vector }

	// Always on the plain driver-station telemetry so it survives Globals.DEBUG_TELEMETRY = false.
	private fun logLoopTime() {
		val now = System.nanoTime().toDouble()
		if (!hasPrevLoop) {
			lastTimeStamp = now
			hasPrevLoop = true
			return
		}
		val loopMs = (now - lastTimeStamp) / 1e6
		lastTimeStamp = now
		loopSumMs += loopMs
		loopCount++
		if (loopMs > loopMaxMs) loopMaxMs = loopMs

		val avgMs = loopSumMs / loopCount
		telemetry.addData(
			"loop",
			"avg %.2f ms (%.0f Hz) | worst %.2f ms".format(avgMs, 1000.0 / avgMs, loopMaxMs)
		)
	}

	override fun init() {
        // msTransmissionInterval is set once in Robot.init (gated on Globals.DEBUG_TELEMETRY).
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2)

        initialize()


		telemetry.addLine("In the initialization phase; start after at least 1 second.")
	}

	override fun start() {
        resetRuntime()

        // Start each match with a clean loop-time meter (init_loop iterations don't count).
        hasPrevLoop = false
        loopSumMs = 0.0
        loopCount = 0L
        loopMaxMs = 0.0

        if (!Globals.AUTO) {
            // No flywheel auto-prespin at teleop start -- the driver spins it up via gamepad2 dpad.
            // (Re-add `ManuallyLaunch { 0.71 }` here to prespin for competition.)
            Robot.follower.startTeleopDrive()
		}
	}

	override fun init_loop() {
		if (!Globals.AUTO) return

		Robot.hubs.forEach { it.clearBulkCache() }

		Robot.read()
		Robot.update() // internally runs the CommandScheduler
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

        val globalTargetDegrees = angleToTargetFromCenter
        val robotHeadingDegrees = robotPose.heading

        val robotAngleDiff = normalizeAngleSigned(globalTargetDegrees - robotHeadingDegrees) // assumes radian input

        controller.updateError(robotAngleDiff)

        Robot.debug("robot heading", Math.toDegrees(robotHeadingDegrees))
        Robot.debug("global heading target", Math.toDegrees(globalTargetDegrees))
        Robot.debug("delta angle", robotAngleDiff)
    }

	// Mode-agnostic. Robot.update() runs the CommandScheduler once and the follower is advanced
	// exactly once per loop inside Robot.read(); each mode's control law lives in cycle() (teleop
	// drive in DriverControlled.cycle(); autos drive through Pedro), so nothing teleop-specific runs
	// during autonomous.
	override fun loop() {
		Robot.hubs.forEach { it.clearBulkCache() }

		Robot.read()
		Robot.update()

		cycle()

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
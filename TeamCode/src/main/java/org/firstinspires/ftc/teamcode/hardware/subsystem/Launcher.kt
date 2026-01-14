package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.utility.absPercentDifference

class Launcher(val left: DcMotorEx, val right: DcMotorEx) : ISubsystem {
	val motors = listOf(left, right)
	var targetTPS = 0.0

	val averageTPS: Double
		get() = (left.velocity + right.velocity) / 2.0

	val atSpeed: Boolean
		get() = absPercentDifference(averageTPS, targetTPS) <= AT_SPEED_TOLERANCE

	val withinSafetyMargins: Boolean
		get() = absPercentDifference(left.velocity, right.velocity) <= MAXIMUM_DEVIANCE

    val gamepad by lazy { Robot.gamepad1.gamepad }

	val isReady: Boolean = false
		get() {
            val wasReady = field
            val nowReady = atSpeed && withinSafetyMargins

            when {
                !wasReady && nowReady -> signalAtSpeed(gamepad)
                wasReady && !nowReady -> signalWrongSpeed(gamepad)
            }

            return nowReady
        }

	fun distanceToTPS(distance: Double): Double {
		val calculated = (distance * TPS_PER_INCH) + BASE_RPM
		return calculated.coerceIn(0.0, MAX_TPS)
	}

    fun targetTPSByDistance(distance: Double) {
        targetTPS = distanceToTPS(distance)
    }

	fun targetTPSByScalar(scale: Double) {
        targetTPS = scaleToTPS(scale)
    }

	override fun reset() {
		targetTPS = 0.0

		right.direction = REVERSE
		left.direction = REVERSE

		motors.forEach {
			it.zeroPowerBehavior = FLOAT
			it.power = 0.0
			it.velocity = 0.0

			it.setCurrentAlert(15.0, CurrentUnit.AMPS)

			it.mode = RunMode.RUN_USING_ENCODER
			it.mode = RunMode.STOP_AND_RESET_ENCODER
		}
	}

	override fun read() {  }

	override fun update() {
		Robot.telemetry.addData("left velocity", left.velocity)
		Robot.telemetry.addData("right velocity", right.velocity)
	}

	override fun write() {
        if (!atSpeed) motors.forEach { it.velocity = targetTPS }
    }
    
    // scale is [0, 1]
    fun scaleToTPS(scale: Double) = scale * MAX_TPS

    fun signalAtSpeed(gamepad: Gamepad) {
        gamepad.runLedEffect(AT_SPEED_LED_EFFECT)
        gamepad.runRumbleEffect(AT_SPEED_RUMBLE_EFFECT)
    }

    fun signalWrongSpeed(gamepad: Gamepad) {
        gamepad.runLedEffect(WRONG_SPEED_LED_EFFECT)
        gamepad.runRumbleEffect(WRONG_SPEED_RUMBLE_EFFECT)
    }

	companion object {
        const val MAX_TPS = 2300.0

		// the maximum amount that the left and right motors can deviate at any given time
		const val MAXIMUM_DEVIANCE = 0.1 // 10%

		// the tolerance to certify a AT_SPEED state
		const val AT_SPEED_TOLERANCE = 0.05 // 5%

		const val BASE_RPM = 0.0
		const val TPS_PER_INCH = 0.0

        val AT_SPEED_LED_EFFECT = Gamepad.LedEffect.Builder()
            .addStep(0.0, 1.0, 0.0, 100)
            .addStep(0.0, 0.0, 0.0, 100)

            .addStep(0.0, 1.0, 0.0, 100)
            .addStep(0.0, 0.0, 0.0, 100)

            .addStep(0.0, 1.0, 0.0, 100)
            .addStep(0.0, 0.0, 0.0, 100)
            .addStep(0.0, 1.0, 0.0, 1000)
            .build()

        val AT_SPEED_RUMBLE_EFFECT = Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 100)
            .addStep(0.0, 0.0, 100)
            .addStep(0.75, 0.75, 100)
            .addStep(0.0, 0.0, 100)
            .addStep(0.5, 0.5, 100)
            .addStep(0.0, 0.0, 100)
            .build()

        val WRONG_SPEED_LED_EFFECT = Gamepad.LedEffect.Builder()
            .addStep(1.0, 0.0, 0.0, 1300)
            .build()

        val WRONG_SPEED_RUMBLE_EFFECT = Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 300)
            .build()
	}
}
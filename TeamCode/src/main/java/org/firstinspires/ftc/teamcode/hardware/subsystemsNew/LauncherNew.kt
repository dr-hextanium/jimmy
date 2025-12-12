package org.firstinspires.ftc.teamcode.hardware.subsystemsNew

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.Launcher
import org.firstinspires.ftc.teamcode.hardware.Launcher.Companion
import org.firstinspires.ftc.teamcode.utility.absPercentDifference

class LauncherNew(val left: DcMotorEx, val right: DcMotorEx) : ISubsystem {
	val motors = listOf(left, right)
	var targetRPM = 0.0

	val averageRPM: Double
		get() = (left.velocity + right.velocity) / 2.0

	val atSpeed: Boolean
		get() = absPercentDifference(averageRPM, targetRPM) <= Launcher.AT_SPEED_TOLERANCE

	val withinSafetyMargins: Boolean
		get() = absPercentDifference(left.velocity, right.velocity) <= Launcher.MAXIMUM_DEVIANCE

	val isReady: Boolean
		get() = atSpeed && withinSafetyMargins

	override fun reset() {
		targetRPM = 0.0

		right.direction = FORWARD
		left.direction = REVERSE

		motors.forEach {
			it.zeroPowerBehavior = FLOAT
			it.power = 0.0
			it.velocity = 0.0

			it.setCurrentAlert(15.0, CurrentUnit.AMPS)

			it.mode = RunMode.STOP_AND_RESET_ENCODER
			it.mode = RunMode.RUN_USING_ENCODER
		}
	}

	override fun read() {
		TODO("Not yet implemented")
	}

	override fun update() {
		TODO("Not yet implemented")
	}

	override fun write() {
		TODO("Not yet implemented")
	}

	enum class State {
		STOPPED,
		SPINNING_UP,
		AT_SPEED
	}

	companion object {
		const val MAX_RPM = 0.0 // ticks per second

		// the maximum amount that the left and right motors can deviate at any given time
		const val MAXIMUM_DEVIANCE = 0.1 // 10%

		// the tolerance to certify a AT_SPEED state
		const val AT_SPEED_TOLERANCE = 0.05 // 5%

		const val BASE_RPM = 0.0
		const val RPM_PER_INCH = 0.0
	}
}
package org.firstinspires.ftc.teamcode.hardware.subsystemsNew

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.utility.absPercentDifference

class LauncherNew(val left: DcMotorEx, val right: DcMotorEx) : ISubsystem {
	val motors = listOf(left, right)
	var targetRPM = 0.0
	var state = State.STOPPED

	val averageRPM: Double
		get() = (left.velocity + right.velocity) / 2.0

	val atSpeed: Boolean
		get() = absPercentDifference(averageRPM, targetRPM) <= AT_SPEED_TOLERANCE

	val withinSafetyMargins: Boolean
		get() = absPercentDifference(left.velocity, right.velocity) <= MAXIMUM_DEVIANCE

	val isReady: Boolean
		get() = atSpeed && withinSafetyMargins

	// distance: inches
	fun distanceToRPM(distance: Double): Double {
		val calculated = (distance * RPM_PER_INCH) + BASE_RPM
		return calculated.coerceIn(0.0, MAX_RPM)
	}

	fun setTargetRPM(message: Message, distance: Double = 0.0, rpm: Double = 0.0) {
		when(message) {
			Message.STOP -> {
				targetRPM = 0.0
				state = State.STOPPED
			}
			Message.SPIN_UP -> {
				targetRPM = distanceToRPM(distanceToRPM(distance))
				state = if (isReady) State.AT_SPEED else State.SPINNING_UP
			}
			Message.OVERRIDE_RPM -> {
				targetRPM = rpm

				state = if (isReady) State.AT_SPEED else State.SPINNING_UP
			}
		}

		when(state) {
			State.STOPPED -> {
				motors.forEach { it.velocity = 0.0 }
			}
			State.SPINNING_UP -> {
				motors.forEach { it.velocity = targetRPM }
			}
			State.AT_SPEED -> {
				motors.forEach { it.velocity = targetRPM }
			}
		}
	}

	override fun reset() {
		targetRPM = 0.0
		state = State.STOPPED

		right.direction = FORWARD
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

	override fun write() {  }

	enum class State {
		STOPPED,
		SPINNING_UP,
		AT_SPEED
	}

	enum class Message {
		STOP,
		SPIN_UP,
		OVERRIDE_RPM
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
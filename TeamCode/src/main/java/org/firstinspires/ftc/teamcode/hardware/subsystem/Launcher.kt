package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.utility.absPercentDifference

class Launcher(val left: DcMotorEx, val right: DcMotorEx) : ISubsystem {
	val motors = listOf(left, right)
	var targetRPM = 0.0

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

    fun setTargetRPMByDistance(distance: Double) {
        daddyTargetRPM(distanceToRPM(distance))
    }

	fun daddyTargetRPM(rpm: Double) { targetRPM = rpm }

	override fun reset() {
		targetRPM = 0.0

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
        if (!atSpeed) {
            motors.forEach { it.velocity = targetRPM * RPM_TO_TPS }
        }
    }

	companion object {
        const val RPM_TO_TPS = (1.0 / 60.0) * (28.0 / 1.0) * (12.0 / 17.0)
        const val MAX_RPM = 2300.0

		// the maximum amount that the left and right motors can deviate at any given time
		const val MAXIMUM_DEVIANCE = 0.1 // 10%

		// the tolerance to certify a AT_SPEED state
		const val AT_SPEED_TOLERANCE = 0.05 // 5%

		const val BASE_RPM = 0.0
		const val RPM_PER_INCH = 0.0
	}
}
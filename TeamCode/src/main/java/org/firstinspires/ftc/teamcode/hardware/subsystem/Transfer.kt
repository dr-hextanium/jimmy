package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.Robot

class Transfer(val motor: DcMotorEx) : ISubsystem {
	var power = 0.0

	override fun reset() {
		motor.direction = FORWARD
		motor.zeroPowerBehavior = BRAKE
		motor.power = 0.0
		motor.mode = RunMode.RUN_WITHOUT_ENCODER
	}

	override fun read() {  }

	override fun update() {
		Robot.telemetry.addData("transfer power", power)
	}

	override fun write() {
		motor.power = power
	}

	companion object {
		const val FEED_POWER = 1.0
		const val REVERSE_POWER = -1.0
	}
}
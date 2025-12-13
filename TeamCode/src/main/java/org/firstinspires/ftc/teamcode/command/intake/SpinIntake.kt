package org.firstinspires.ftc.teamcode.command.intake

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.hardware.subsystemsNew.IntakeNew

class SpinIntake : CommandTemplate() {
	override fun initialize() { Robot.Subsystems.intake.power = IntakeNew.POWER_INTAKE }

	override fun execute() {  }

	override fun isFinished(): Boolean = true

}
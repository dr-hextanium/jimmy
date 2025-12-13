package org.firstinspires.ftc.teamcode.command.intake

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.hardware.subsystem.Intake

class ReverseIntake : CommandTemplate() {
	override fun initialize() { Robot.Subsystems.intake.power = Intake.POWER_REVERSE }

	override fun execute() {  }

	override fun isFinished(): Boolean = true

}
package org.firstinspires.ftc.teamcode.command.intake

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.hardware.subsystem.Intake

class IntakeIn(val power: Double = Intake.POWER_INTAKE) : CommandTemplate() {
	override fun initialize() {
        Robot.Subsystems.intake.power = power
    }

	override fun execute() {  }

	override fun isFinished(): Boolean = true

}
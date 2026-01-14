package org.firstinspires.ftc.teamcode.command.intake

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.hardware.subsystem.Intake

class SpinIntake : CommandTemplate() {
	override fun initialize() {
        Robot.Subsystems.intake.power = Intake.POWER_INTAKE
        Robot.Subsystems.transfer.power = -0.4
    }

	override fun execute() {  }

	override fun isFinished(): Boolean = true

}
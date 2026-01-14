package org.firstinspires.ftc.teamcode.command.intake

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot

class StopIntake : CommandTemplate() {
	override fun initialize() {
        Robot.Subsystems.intake.power = 0.0
        Robot.Subsystems.transfer.power = 0.0
    }

	override fun execute() {  }

	override fun isFinished(): Boolean = true
}
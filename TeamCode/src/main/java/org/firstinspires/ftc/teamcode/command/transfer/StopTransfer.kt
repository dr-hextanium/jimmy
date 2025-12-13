package org.firstinspires.ftc.teamcode.command.transfer

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot

class StopTransfer : CommandTemplate() {
	override fun initialize() { Robot.Subsystems.transfer.power = 0.0 }

	override fun execute() {  }

	override fun isFinished(): Boolean = true
}
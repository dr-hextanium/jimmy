package org.firstinspires.ftc.teamcode.command.transfer

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.hardware.subsystem.Transfer

class ReverseTransfer : CommandTemplate() {
	override fun initialize() { Robot.Subsystems.transfer.power = Transfer.REVERSE_POWER }

	override fun execute() {  }

	override fun isFinished(): Boolean = true
}
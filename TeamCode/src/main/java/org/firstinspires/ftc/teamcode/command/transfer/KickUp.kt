package org.firstinspires.ftc.teamcode.command.transfer

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.hardware.subsystem.Transfer

class KickUp : CommandTemplate() {
	override fun initialize() { Robot.Subsystems.transfer.power = Transfer.FEED_POWER }

	override fun execute() {  }

	override fun isFinished(): Boolean = true
}
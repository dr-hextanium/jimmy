package org.firstinspires.ftc.teamcode.command.Transfer

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.hardware.subsystemsNew.IntakeNew
import org.firstinspires.ftc.teamcode.hardware.subsystemsNew.TransferNew

class KickUp : CommandTemplate() {
	override fun initialize() { Robot.Subsystems.transfer.power = TransferNew.POWER_FEED }

	override fun execute() {  }

	override fun isFinished(): Boolean = true
}
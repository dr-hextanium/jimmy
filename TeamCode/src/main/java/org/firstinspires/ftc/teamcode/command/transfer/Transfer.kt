package org.firstinspires.ftc.teamcode.command.transfer

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.hardware.subsystem.Transfer

class Transfer(val power: () -> Double = { Transfer.FEED_POWER }) : CommandTemplate() {
	override fun initialize() { Robot.Subsystems.transfer.power = power() }

	override fun execute() {  }

	override fun isFinished(): Boolean = true
}
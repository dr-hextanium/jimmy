package org.firstinspires.ftc.teamcode.command.Launcher

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot

class SetLauncherRPMManual(private val message: Message, private val rpm: Double) : CommandTemplate() {
	override fun initialize() { Robot.Subsystems.launcher.setTargetRPM() }

	override fun execute() {
		TODO("Not yet implemented")
	}

	override fun isFinished(): Boolean {
		TODO("Not yet implemented")
	}
}
package org.firstinspires.ftc.teamcode.opmode

import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.command.launcher.Manual
import org.firstinspires.ftc.teamcode.command.launcher.StopLauncher
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.opmode.template.BaseTemplate

@TeleOp
class FindRPM : BaseTemplate() {
	override fun initialize() {
		GamepadButton(primary, GamepadKeys.Button.X)
			.whenPressed(
				Manual(100000000000.0)
			)

		GamepadButton(primary, CIRCLE)
			.whenPressed(
				StopLauncher()
			)
	}

	override fun cycle() {
		telemetry.addData("left ticks per second", Robot.Subsystems.launcher.left.velocity)
		telemetry.addData("right ticks per second", Robot.Subsystems.launcher.right.velocity)
	}
}
package org.firstinspires.ftc.teamcode.opmode.debug

import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.command.intake.SpinIntake
import org.firstinspires.ftc.teamcode.command.intake.StopIntake
import org.firstinspires.ftc.teamcode.command.launcher.Manual
import org.firstinspires.ftc.teamcode.command.launcher.StopLauncher
import org.firstinspires.ftc.teamcode.command.transfer.Transfer
import org.firstinspires.ftc.teamcode.command.transfer.StopTransfer
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.opmode.template.BaseTemplate
import org.firstinspires.ftc.teamcode.wrapper.GamepadTrigger

@TeleOp
@Disabled
class FindRPM : BaseTemplate() {
	override fun initialize() {
		GamepadButton(primary, GamepadKeys.Button.X)
			.whenPressed(
				Manual({ 1.0 })
			)

		GamepadButton(primary, CIRCLE)
			.whenPressed(
				StopLauncher()
			)

		GamepadTrigger(primary, 0.5, GamepadKeys.Trigger.LEFT_TRIGGER)
			.whenActive(
				SpinIntake()
			)
			.whenInactive(
				StopIntake()
			)

		GamepadTrigger(primary, 0.5, GamepadKeys.Trigger.RIGHT_TRIGGER)
			.whenActive(
				Manual({ 1.0 })
			)
			.whenInactive(
				StopLauncher()
			)

		GamepadButton(primary, GamepadKeys.Button.Y)
			.whenPressed(
				SequentialCommandGroup(
					Transfer(),
					WaitCommand(750),
					StopTransfer()
				)
			)
	}

	override fun cycle() {
		telemetry.addData("left ticks per second", Robot.Subsystems.launcher.left.velocity)
		telemetry.addData("right ticks per second", Robot.Subsystems.launcher.right.velocity)
	}
}
package org.firstinspires.ftc.teamcode.opmode

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.command.intake.SpinIntake
import org.firstinspires.ftc.teamcode.command.intake.StopIntake
import org.firstinspires.ftc.teamcode.command.launcher.Manual
import org.firstinspires.ftc.teamcode.command.launcher.StopLauncher
import org.firstinspires.ftc.teamcode.command.transfer.Transfer
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.opmode.template.BaseTemplate
import org.firstinspires.ftc.teamcode.wrapper.GamepadTrigger

@TeleOp
class DriverControlled : BaseTemplate() {
    var launcherPower = 0.0
    var increment = 0.01

	override fun initialize() {
		GamepadTrigger(primary, 0.3, GamepadKeys.Trigger.LEFT_TRIGGER)
			.whenActive(
                ParallelCommandGroup(
                    SpinIntake(),
                    Transfer { -0.5 }
                )
			)
			.whenInactive(
				StopIntake()
			)

        GamepadTrigger(primary, 0.3, GamepadKeys.Trigger.RIGHT_TRIGGER)
            .whenActive(
                ParallelCommandGroup(
                    Transfer(),
                    SpinIntake()
                )
            )
            .whenInactive(
                StopIntake()
            )

        GamepadButton(primary, GamepadKeys.Button.X)
            .whenPressed(
                StopLauncher()
                )

        GamepadButton(primary, GamepadKeys.Button.Y)
            .whenPressed(
                Manual { launcherPower }
            )

        GamepadButton(primary, GamepadKeys.Button.B)
            .whenPressed(
                Manual { 1.0 }
            )

        GamepadButton(primary, GamepadKeys.Button.RIGHT_STICK_BUTTON)
            .whenPressed(InstantCommand({
                Robot.follower.pose = Pose(Robot.pose.x, Robot.pose.y, 0.0)
            }))

        GamepadButton(primary, GamepadKeys.Button.LEFT_STICK_BUTTON)
            .whenPressed(InstantCommand({
                goalLock = !goalLock
            }))
	}

	override fun cycle() {
        launcherPower += when {
            gamepad1.dpad_right -> increment
            gamepad1.dpad_left -> -increment
            else -> 0.0
        }

        launcherPower = launcherPower.coerceIn(0.0, 1.0)

        telemetry.addData("average tps", Robot.Subsystems.launcher.averageTPS)
        telemetry.addData("ready", Robot.Subsystems.launcher.isReady)
        Robot.telemetry.addData("launcher power", launcherPower)
        Robot.telemetry.addData("heading", Math.toDegrees(Robot.follower.heading))
	}
}
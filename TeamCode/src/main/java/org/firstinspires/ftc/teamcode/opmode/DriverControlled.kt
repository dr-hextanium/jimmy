package org.firstinspires.ftc.teamcode.opmode

import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.command.intake.SpinIntake
import org.firstinspires.ftc.teamcode.command.intake.StopIntake
import org.firstinspires.ftc.teamcode.command.launcher.Manual
import org.firstinspires.ftc.teamcode.command.transfer.Transfer
import org.firstinspires.ftc.teamcode.hardware.Globals
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.hardware.subsystem.Launcher
import org.firstinspires.ftc.teamcode.opmode.template.BaseTemplate
import org.firstinspires.ftc.teamcode.wrapper.GamepadTrigger

@TeleOp
class DriverControlled : BaseTemplate() {
    override fun initialize() {
        Globals.AUTO = false

        // --- 1. PRIMARY DRIVER CONTROLS (INTAKE) ---
        GamepadTrigger(primary, 0.3, GamepadKeys.Trigger.RIGHT_TRIGGER)
            .whenActive(ParallelCommandGroup(Transfer(), SpinIntake()))
            .whenInactive(ParallelCommandGroup(StopIntake(), Transfer { 0.0 }))

        GamepadTrigger(primary, 0.3, GamepadKeys.Trigger.LEFT_TRIGGER)
            .whenActive(ParallelCommandGroup(SpinIntake(), Transfer { -1.0 }))
            .whenInactive(ParallelCommandGroup(StopIntake(), Transfer { 0.0 }))

        // --- 2. JAM CLEARING (New) ---
        // Runs ONLY the transfer backwards. Essential if a ball jams at the flywheel entry.
        GamepadButton(primary, GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(Transfer { -1.0 })
            .whenReleased(Transfer { 0.0 })

//        // --- 3. SHOOTING MACROS ---
//        GamepadButton(primary, GamepadKeys.Button.X)
//            .whenPressed(InstantCommand({ Robot.follower.pose = Pose(55.0, 7.5, Robot.pose.heading) }))

        GamepadButton(primary, GamepadKeys.Button.Y)
            .whenPressed(Manual { 0.71 })

        GamepadButton(primary, GamepadKeys.Button.B)
            .whenPressed(Manual { 0.93 })
//            .whenPressed(Manual { 0.875 })

        // --- 4. MANUAL OVERRIDES (New) ---

        // PRE-SPIN: Hold to rev up flywheel while driving to position
        // When released, it drops back to IDLE (1000 RPM)
        GamepadButton(primary, GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(Manual { 1.0 })
            .whenReleased(Manual { Launcher.IDLE_TPS / Launcher.MAX_TPS })

        // MANUAL FEED: Tap to push one ball into the flywheel manually
        // Useful if the sensor logic fails or you just want a single shot
        GamepadButton(primary, GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(Transfer { 1.0 })
            .whenReleased(Transfer { 0.0 })

        // --- 5. SAFETY / ABORT (Modified) ---
        // Stops flywheel AND Cancels any running TransferSequence macro
        GamepadButton(primary, GamepadKeys.Button.A)
            .whenPressed(InstantCommand({
                CommandScheduler.getInstance().cancelAll() // Stop the macro!
                Robot.Subsystems.launcher.targetTPSByScalar(0.0) // Kill motors
                Robot.Subsystems.intake.power = 0.0
                Robot.Subsystems.transfer.power = 0.0
            }))

        // --- 6. UTILITY ---
        GamepadButton(primary, GamepadKeys.Button.RIGHT_STICK_BUTTON)
            .whenPressed(InstantCommand({
                Robot.follower.pose = Pose(Robot.pose.x, Robot.pose.y, 0.0)
            }))
    }

    override fun cycle() {
        // Only show necessary telemetry
        val hubCurrent = Robot.hubs.sumOf { it.getCurrent(CurrentUnit.AMPS) }
        println(hubCurrent)
        Robot.telemetry.addData("hub current", hubCurrent)
        Robot.telemetry.addData("average tps", Robot.Subsystems.launcher.averageTPS)
        Robot.telemetry.addData("shooter ready?", Robot.Subsystems.launcher.isReady)
        Robot.telemetry.addData("heading", Math.toDegrees(Robot.follower.heading))
    }
}
package org.firstinspires.ftc.teamcode.opmode

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.command.intake.CloseGate
import org.firstinspires.ftc.teamcode.command.intake.IntakeIn
import org.firstinspires.ftc.teamcode.command.intake.IntakeOut
import org.firstinspires.ftc.teamcode.command.intake.OpenGate
import org.firstinspires.ftc.teamcode.command.intake.StopIntake
import org.firstinspires.ftc.teamcode.command.turret.AimAtGoal
import org.firstinspires.ftc.teamcode.command.turret.PointTowards
import org.firstinspires.ftc.teamcode.command.turret.StopAimingAtGoal
import org.firstinspires.ftc.teamcode.hardware.Globals
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.hardware.Robot.Subsystems.launcher
import org.firstinspires.ftc.teamcode.opmode.template.BaseTemplate
import org.firstinspires.ftc.teamcode.wrapper.GamepadTrigger

@TeleOp
class DriverControlledRed : DriverControlled(isRed = true)

@TeleOp
class DriverControlledBlue : DriverControlled(isRed = false)

open class DriverControlled(val isRed: Boolean) : BaseTemplate() {
    var launcherPower = 0.0
    var increment = 0.01


    override fun initialize() {
        Globals.AUTO = false
        Globals.isRed = isRed

        Robot.follower.pose = Globals.AUTO_RECOVERY_POSITION ?: Pose(0.0, 0.0, 0.0)

        val resetPose = if (Globals.isRed ?: true) Globals.RED_RESET_POSE else Globals.BLUE_RESET_POSE

        GamepadTrigger(primary, 0.3, GamepadKeys.Trigger.LEFT_TRIGGER)
            .whenActive(ParallelCommandGroup(IntakeIn(), CloseGate()))
            .whenInactive(StopIntake())

        GamepadTrigger(primary, 0.3, GamepadKeys.Trigger.RIGHT_TRIGGER)
            .whenActive(ParallelCommandGroup(IntakeIn(), OpenGate()))
            .whenInactive(ParallelCommandGroup(StopIntake(), CloseGate()))

        GamepadButton(primary, GamepadKeys.Button.RIGHT_STICK_BUTTON)
            .whenPressed(InstantCommand({
                Robot.follower.pose = Pose(Robot.pose.x, Robot.pose.y, 0.0)
            }))

        GamepadButton(primary, GamepadKeys.Button.LEFT_STICK_BUTTON)
            .whenPressed(InstantCommand({
                Robot.follower.pose = Pose(resetPose.xComponent, resetPose.yComponent, Robot.pose.heading)
            }))

        GamepadButton(primary, GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(IntakeOut())
            .whenReleased(StopIntake())

        GamepadButton(primary, SQUARE)
            .whenPressed(ParallelCommandGroup(
                StopAimingAtGoal(),
                PointTowards(0.0)
            ))

        GamepadButton(primary, CIRCLE)
            .whenPressed(AimAtGoal())

        GamepadButton(primary, GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(InstantCommand({ Robot.Subsystems.turret.offset += 10.0 }))

        GamepadButton(primary, GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(InstantCommand({ Robot.Subsystems.turret.offset -= 10.0 }))
    }

    override fun cycle() {
        val goalPose = if (Globals.isRed ?: true) Globals.RED_GOAL_POSE else Globals.BLUE_GOAL_POSE

        launcherPower += when {
            gamepad1.dpad_right -> increment
            gamepad1.dpad_left -> -increment
            else -> 0.0
        }.coerceIn(0.0, 1.0)

//        launcher.targetTPSByScalar(launcher.distanceToScalar((Vector(Robot.pose) - goalPose).magnitude))

        launcher.targetTPSByScalar(launcherPower)

        Robot.telemetry.addData("robot pose", Robot.follower.pose)

        if (Robot.Subsystems.turret.aimAtGoal) {
            Robot.Subsystems.turret.face(goalPose, Robot.follower.pose, Vector())
        }

        Robot.telemetry.addData("average tps", launcher.averageTPS)
        Robot.telemetry.addData("target tps?", launcher.targetTPS)
        Robot.telemetry.addData("shooter ready?", launcher.isReady)
        Robot.telemetry.addData("distance from goal", Robot.Subsystems.turret.distance(goalPose, Robot.follower.pose))
        Robot.telemetry.addData("heading", Math.toDegrees(Robot.follower.heading))
    }

    override fun stop() {
        Globals.isRed = null
        Globals.AUTO_RECOVERY_POSITION = null

        super.stop()
    }
}
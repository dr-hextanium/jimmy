package org.firstinspires.ftc.teamcode.opmode

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.command.intake.CloseGate
import org.firstinspires.ftc.teamcode.command.intake.IntakeIn
import org.firstinspires.ftc.teamcode.command.intake.IntakeOut
import org.firstinspires.ftc.teamcode.command.intake.OpenGate
import org.firstinspires.ftc.teamcode.command.intake.StopIntake
import org.firstinspires.ftc.teamcode.command.launcher.ManualHood
import org.firstinspires.ftc.teamcode.command.launcher.ManuallyLaunch
import org.firstinspires.ftc.teamcode.command.turret.AimAtGoal
import org.firstinspires.ftc.teamcode.command.turret.PointTowards
import org.firstinspires.ftc.teamcode.command.turret.StopAimingAtGoal
import org.firstinspires.ftc.teamcode.hardware.Globals
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.hardware.Zones
import org.firstinspires.ftc.teamcode.opmode.template.BaseTemplate
import org.firstinspires.ftc.teamcode.wrapper.GamepadTrigger
import kotlin.math.PI
import kotlin.math.hypot
import kotlin.math.roundToInt

@TeleOp
class DriverControlledRed : DriverControlled(isRed = true, 0.0)

@TeleOp
class DriverControlledBlue : DriverControlled(isRed = false, PI)

open class DriverControlled(val isRed: Boolean, initialHeading: Double) : BaseTemplate(initialHeading) {
    var launcherPower = 0.64
    var launcherIncrement = 0.01
    var hoodPosition = 0.2
    var servoIncrement = 0.005

    // Derive from the alliance constructor param, NOT Globals.isRed: these initializers run at
    // construction, before initialize() sets Globals.isRed (and stop() nulls it), so reading the
    // global here would always fall through to the RED defaults.
    var baseZonePose = if (isRed) Globals.RED_BASE_POSE else Globals.BLUE_BASE_POSE
    var goalZone = if (isRed) Zones.RED_GOAL_ZONE else Zones.BLUE_GOAL_ZONE
    var goalPose = if (isRed) Globals.RED_GOAL_POSE else Globals.BLUE_GOAL_POSE

    fun distanceToGoal(): Double = hypot(goalPose.xComponent - Robot.follower.pose.x, goalPose.yComponent - Robot.follower.pose.y)

    override fun initialize() {
        Globals.AUTO = false
        Globals.isRed = isRed

        Robot.follower.pose = Globals.AUTO_RECOVERY_POSITION ?: Pose(72.0, 72.0, 0.0)

        GamepadButton(primary, SQUARE)
            .whenPressed(InstantCommand({
                goalLock = !goalLock

                if (goalLock) {
                    Robot.scheduler.schedule(AimAtGoal())
                } else {
                    Robot.scheduler.schedule(SequentialCommandGroup(StopAimingAtGoal(), PointTowards(0.0)))
                }
            }))

        GamepadTrigger(primary, 0.3, GamepadKeys.Trigger.LEFT_TRIGGER)
            .whenActive(IntakeIn())
            .whenInactive(StopIntake())

        GamepadTrigger(primary, 0.5, GamepadKeys.Trigger.RIGHT_TRIGGER)
            .whenActive(OpenGate())
            .whenInactive(CloseGate())

        // RIGHT stick button = HEADING reset: zero the field-centric heading offset to the current
        // facing, so "up" on the drive stick means the way the robot is pointing right now.
        GamepadButton(primary, GamepadKeys.Button.RIGHT_STICK_BUTTON)
            .whenPressed(InstantCommand({ Globals.globalHeadingOffset = Robot.follower.pose.heading }))

        // LEFT stick button = POSITION relocalize to the base zone: snap the localizer to the base-
        // zone pose (its known x/y and the heading the robot faces there) and clear the heading
        // offset so field-centric drive is referenced to the field frame again.
        GamepadButton(primary, GamepadKeys.Button.LEFT_STICK_BUTTON)
            .whenPressed(
                InstantCommand({
                    Globals.globalHeadingOffset = 0.0
                    Robot.follower.pose = baseZonePose
                })
            )

        GamepadButton(primary, GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(IntakeOut())
            .whenReleased(StopIntake())

        setTargetPose(goalPose)

        GamepadButton(primary, GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(InstantCommand({ Robot.Subsystems.turret.offset += 10.0 }))

        GamepadButton(primary, GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(InstantCommand({ Robot.Subsystems.turret.offset -= 10.0 }))
    }

    override fun cycle() {
        // Field-centric teleop drive + turret/heading goal-lock. Lives here (not in BaseTemplate.loop())
        // so it never runs during autonomous, which drives through Pedro. setTeleOpDrive stores the
        // movement vectors; they take effect on the next loop's follower.update() (in Robot.read()).
        // Turret goal-lock (SQUARE) aims the TURRET only; the chassis stays fully driver-controlled,
        // so the robot no longer auto-rotates toward the goal.
        val angularAdjustment = (-gamepad1.right_stick_x).toDouble()

        Robot.follower.setTeleOpDrive(
            (-gamepad1.left_stick_y).toDouble(),
            (-gamepad1.left_stick_x).toDouble(),
            angularAdjustment,
            false,
            Globals.globalHeadingOffset
        )

        val launcherPowerChange = when {
            gamepad2.dpad_right -> launcherIncrement
            gamepad2.dpad_left -> -launcherIncrement
            else -> 0.0
        }

        val hoodPositionChange = when {
            gamepad2.dpad_up -> servoIncrement
            gamepad2.dpad_down -> -servoIncrement
            else -> 0.0
        }

        // Only (re)schedule when the driver actually changes something -- these are
        // "instant" commands (all work happens in initialize()), so scheduling them every
        // loop regardless of change would just be needless churn through the scheduler.
        if (launcherPowerChange != 0.0) {
            launcherPower = (launcherPower + launcherPowerChange).coerceIn(-1.0, 1.0)
            Robot.scheduler.schedule(ManuallyLaunch { launcherPower })
        }

        if (hoodPositionChange != 0.0) {
            hoodPosition = (hoodPosition + hoodPositionChange).coerceIn(0.0, 1.0)
            Robot.scheduler.schedule(ManualHood { hoodPosition })
        }

        if (Globals.DEBUG_TELEMETRY) {
            Robot.telemetry.addData("robot pose", Robot.follower.pose.let { "x: ${it.x}, y: ${it.y}, h: ${Math.toDegrees(it.heading).roundToInt()}" })

            Robot.telemetry.addData("launching scalar", launcherPower)
            Robot.telemetry.addData("hood scalar", hoodPosition)

            Robot.telemetry.addData("aiming at goal?", goalLock)

            Robot.telemetry.addData("distance to goal", distanceToGoal())
            Robot.telemetry.addData("target goal pose", targetGoalPose)
        }
    }

    override fun stop() {
        Globals.isRed = null
        Globals.AUTO_RECOVERY_POSITION = null
        Globals.globalHeadingOffset = 0.0

        super.stop()
    }
}
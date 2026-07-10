package org.firstinspires.ftc.teamcode.opmode

//import org.firstinspires.ftc.teamcode.command.turret.AimAtGoal
//import org.firstinspires.ftc.teamcode.command.turret.StopAimingAtGoal
import com.arcrobotics.ftclib.command.InstantCommand
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
import org.firstinspires.ftc.teamcode.hardware.Globals
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.hardware.Robot.Subsystems.launcher
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

    var resetPose = if (Globals.isRed ?: true) Globals.RED_RESET_POSE else Globals.BLUE_RESET_POSE
    var goalZone = if (Globals.isRed ?: true) Zones.RED_GOAL_ZONE else Zones.BLUE_GOAL_ZONE
    var goalPose = if (Globals.isRed ?: true) Globals.RED_GOAL_POSE else Globals.BLUE_GOAL_POSE
    var actualHeadingAtBaseZone = if (Globals.isRed ?: true) 0.0 else PI

    var i = 0

    var shooterEnabled = false

    fun distanceToGoal(): Double = hypot(goalPose.xComponent - Robot.follower.pose.x, goalPose.yComponent - Robot.follower.pose.y)

    override fun initialize() {
        Globals.AUTO = false
        Globals.isRed = isRed

        Robot.follower.pose = Globals.AUTO_RECOVERY_POSITION ?: Pose(72.0, 72.0, 0.0)

        GamepadButton(primary, SQUARE)
            .whenPressed(InstantCommand({
                goalLock = !goalLock
            }))

        GamepadButton(primary, TRIANGLE)
            .whenPressed(InstantCommand({
                shooterEnabled = !shooterEnabled
            }))

        GamepadButton(primary, CIRCLE)
            .whenPressed(InstantCommand({
                val v = Vector(0.0, 0.0)

                val potentialTargets = listOf(
                    Pair(0.0, 0.0), Pair(0.0, 144.0), Pair(144.0, 0.0), Pair(144.0, 144.0),
                    Pair(0.0, 0.0), Pair(0.0, -144.0), Pair(-144.0, 0.0), Pair(-144.0, -144.0)
                )

                if (i >= (potentialTargets.size - 1)) {
                    i = 0
                } else {
                    i++
                }

                val pair = potentialTargets[i]

                v.setOrthogonalComponents(pair.first, pair.second)

                setTargetPose(v)
            }))

        GamepadTrigger(primary, 0.3, GamepadKeys.Trigger.LEFT_TRIGGER)
            .whenActive(IntakeIn())
            .whenInactive(StopIntake())

        GamepadTrigger(primary, 0.5, GamepadKeys.Trigger.RIGHT_TRIGGER)
            .whenActive(OpenGate())
            .whenInactive(CloseGate())

        GamepadButton(primary, GamepadKeys.Button.RIGHT_STICK_BUTTON)
            .whenPressed(InstantCommand({ Globals.globalHeadingOffset = Robot.follower.pose.heading }))

        GamepadButton(primary, GamepadKeys.Button.LEFT_STICK_BUTTON)
            .whenPressed(
                InstantCommand({
                    Robot.pose.heading = actualHeadingAtBaseZone
                    Globals.globalHeadingOffset = 0.0

                    Robot.follower.pose = resetPose
                })
            )

        GamepadButton(primary, GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(IntakeOut())
            .whenReleased(StopIntake())

        setTargetPose(goalPose)

//        GamepadButton(primary, SQUARE)
//            .whenPressed(ParallelCommandGroup(
//                StopAimingAtGoal(),
//                PointTowards(0.0)
//            ))
//
//        GamepadButton(primary, CIRCLE)
//            .whenPressed(AimAtGoal())

//        GamepadButton(primary, GamepadKeys.Button.DPAD_LEFT)
//            .whenPressed(InstantCommand({ Robot.Subsystems.turret.offset += 10.0 }))
//
//        GamepadButton(primary, GamepadKeys.Button.DPAD_RIGHT)
//            .whenPressed(InstantCommand({ Robot.Subsystems.turret.offset -= 10.0 }))

//        Robot.follower.pose = Pose(Robot.follower.pose.x, Robot.follower.pose.y, initialHeading)
    }

    override fun cycle() {
//        setTargetPose(goalPose)

        launcherPower += when {
            gamepad2.dpad_right -> launcherIncrement
            gamepad2.dpad_left -> -launcherIncrement
            else -> 0.0
        }

        hoodPosition += when {
            gamepad2.dpad_up -> servoIncrement
            gamepad2.dpad_down -> -servoIncrement
            else -> 0.0
        }

        // launcher.distanceToScalar((Vector(Robot.pose) - goalPose).magnitude)

        // 1.0, 0.225
        //
        // 130.0 0.89 0.185 119 deg
        // 122.2 0.82 0.03 122 deg

        launcherPower = launcherPower.coerceIn(-1.0, 1.0)
        hoodPosition = hoodPosition.coerceIn(0.0, 1.0)

        launcher.targetTPSByScalar(launcherPower)
        launcher.targetHoodByScalar(hoodPosition)

//        if (shooterEnabled) {
//            launcher.targetTPSByScalar(powerRegression(distanceToGoal()))
//        } else {
//            launcher.targetTPSByScalar(0.0)
//        }
//
//        launcher.targetHoodByScalar(hoodRegression(distanceToGoal()))

//        launcher.targetTPSByScalar(Launcher.LookupTables.powerInterpLUT[distanceToGoal()])
//        launcher.targetTPSByScalar(Launcher.LookupTables.powerInterpLUT[distanceToGoal()])

//        launcher.targetTPSByScalar(0.0)
//        launcher.targetHoodByScalar(Launcher.LookupTables.hoodInterpLUT[distanceToGoal()])

//        launcher.currentPower = launcherPower

        Robot.telemetry.addData("robot pose", Robot.follower.pose.let { "x: ${it.x}, y: ${it.y}, h: ${Math.toDegrees(it.heading).roundToInt()}" })

        Robot.telemetry.addData("launching scalar", launcherPower)
        Robot.telemetry.addData("hood scalar", hoodPosition)

        Robot.telemetry.addData("aiming at goal?", goalLock)
        Robot.telemetry.addData("in shooting zone?", Robot.inShootingZone)
        Robot.telemetry.addData("shooter enabled?", shooterEnabled)

//        if (Robot.Subsystems.turret.aimAtGoal) {
//            Robot.Subsystems.turret.face(goalPose, Robot.follower.pose, Vector())
//        }

        Robot.telemetry.addData("distance to goal", distanceToGoal())
        Robot.telemetry.addData("index", i)
        Robot.telemetry.addData("target goal pose", targetGoalPose)

//        Robot.telemetry.addData("average tps", launcher.averageTPS)
//        Robot.telemetry.addData("target tps?", launcher.targetTPS)
//        Robot.telemetry.addData("shooter ready?", launcher.isReady)
//        Robot.telemetry.addData("heading", Math.toDegrees(Robot.follower.heading))
//        Robot.telemetry.addData("heading offset", Globals.globalHeadingOffset)
    }

    override fun stop() {
        Globals.isRed = null
        Globals.AUTO_RECOVERY_POSITION = null
        Globals.globalHeadingOffset = 0.0

        super.stop()
    }
}
package org.firstinspires.ftc.teamcode.opmode.debug

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.Globals
import org.firstinspires.ftc.teamcode.hardware.Robot
import kotlin.math.hypot

/**
 * Read-only whole-robot status dashboard for a quick pre-match / configuration health check:
 * pose, distance to each goal, turret fused angle, launcher velocities + at-speed, and hood/gate
 * positions.
 *
 * Low risk: after init it only calls Robot.read() (localizer + sensor reads); it never calls
 * Robot.update()/write(), so no motor or servo is driven from the loop.
 */
@TeleOp(name = "Robot Status Debug", group = "Debug")
class RobotStatusDebug : OpMode() {
    override fun init() {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2)
    }

    override fun loop() {
        Robot.hubs.forEach { it.clearBulkCache() }
        Robot.read() // localizer + subsystem reads only -- no actuator writes

        val pose = Robot.follower.pose
        telemetry.addData("pose x", "%.1f", pose.x)
        telemetry.addData("pose y", "%.1f", pose.y)
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.heading))
        telemetry.addData("dist to RED goal", "%.1f", distanceTo(pose, Globals.RED_GOAL_POSE))
        telemetry.addData("dist to BLUE goal", "%.1f", distanceTo(pose, Globals.BLUE_GOAL_POSE))

        telemetry.addLine("--- turret ---")
        telemetry.addData("fused angle (deg)", "%.2f", Robot.Subsystems.turret.currentAngle)

        telemetry.addLine("--- launcher ---")
        telemetry.addData("left vel (tps)", "%.0f", Robot.Motors.Launcher.leftMotor.velocity)
        telemetry.addData("right vel (tps)", "%.0f", Robot.Motors.Launcher.rightMotor.velocity)
        telemetry.addData("avg tps", "%.0f", Robot.Subsystems.launcher.averageTPS)
        telemetry.addData("at speed?", Robot.Subsystems.launcher.isReady)

        telemetry.addLine("--- servos ---")
        telemetry.addData("hood pos", "%.3f", Robot.Servos.Launcher.hood.position)
        telemetry.addData("gate pos", "%.3f", Robot.Servos.Intake.gate.position)
    }

    private fun distanceTo(pose: Pose, goal: Vector): Double =
        hypot(goal.xComponent - pose.x, goal.yComponent - pose.y)
}

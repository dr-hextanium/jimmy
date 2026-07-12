package org.firstinspires.ftc.teamcode.command.launcher

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot

/**
 * Set the launcher's flywheel speed and hood from the kinematic shooter model for a target at the
 * given field [distance] (inches). One-shot: it just applies the aiming solution and finishes.
 */
class LaunchByDistance(val distance: Double) : CommandTemplate() {
    override fun initialize() {
        Robot.Subsystems.launcher.aimAtDistance(distance)
    }

    override fun execute() {}

    override fun isFinished() = true
}

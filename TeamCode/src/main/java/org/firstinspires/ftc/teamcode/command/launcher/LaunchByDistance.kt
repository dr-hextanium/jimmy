package org.firstinspires.ftc.teamcode.command.launcher

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot

class LaunchByDistance(val distance: Double) : CommandTemplate() {
    override fun initialize() {
        val launcher = Robot.Subsystems.launcher

        launcher.targetTPSByScalar(launcher.distanceToScalar(distance))
    }

    override fun execute() {}

    override fun isFinished() = true
}
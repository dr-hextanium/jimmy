package org.firstinspires.ftc.teamcode.command.launcher

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot

class SpinUp(val distance: Double) : CommandTemplate() {
    override fun initialize() {
        Robot.Subsystems.launcher.setTargetRPMByDistance(distance)
    }

    override fun execute() {}

    override fun isFinished() = true
}
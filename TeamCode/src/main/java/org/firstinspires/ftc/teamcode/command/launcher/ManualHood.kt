package org.firstinspires.ftc.teamcode.command.launcher

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot

class ManualHood(val scalar: () -> Double) : CommandTemplate() {
    override fun initialize() {
        Robot.Subsystems.launcher.targetHoodByScalar(scalar())
    }

    override fun execute() {}

    override fun isFinished() = true
}
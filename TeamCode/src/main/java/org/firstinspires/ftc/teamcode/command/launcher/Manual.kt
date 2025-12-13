package org.firstinspires.ftc.teamcode.command.launcher

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot

open class Manual(val rpm: Double) : CommandTemplate() {
    override fun initialize() {
        Robot.Subsystems.launcher.setTargetRPM(rpm)
    }

    override fun execute() {

    }

    override fun isFinished() = true
}
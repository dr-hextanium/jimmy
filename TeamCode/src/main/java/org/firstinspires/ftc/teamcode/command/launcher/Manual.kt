package org.firstinspires.ftc.teamcode.command.launcher

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot

open class Manual(val scalar: () -> Double) : CommandTemplate() {
    override fun initialize() {
        Robot.Subsystems.launcher.targetTPSByScalar(scalar())
    }

    override fun execute() {

    }

//    override fun isFinished() = Robot.Subsystems.launcher.isReady
override fun isFinished() = true
}
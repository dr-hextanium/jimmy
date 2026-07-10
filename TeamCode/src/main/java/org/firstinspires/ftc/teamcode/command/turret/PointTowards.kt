package org.firstinspires.ftc.teamcode.command.turret

import org.firstinspires.ftc.teamcode.command.CommandTemplate

class PointTowards(val degrees: Double) : CommandTemplate() {
    override fun initialize() {
//        Robot.Subsystems.turret.setTargetAngle(degrees)
    }

    override fun execute() {  }

    override fun isFinished(): Boolean = true
}
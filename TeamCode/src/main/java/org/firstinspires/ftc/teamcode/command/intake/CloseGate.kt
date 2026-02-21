package org.firstinspires.ftc.teamcode.command.intake

import org.firstinspires.ftc.teamcode.command.CommandTemplate
import org.firstinspires.ftc.teamcode.hardware.Robot

class CloseGate : CommandTemplate() {
    override fun initialize() {
        Robot.Subsystems.intake.closeGate()
    }

    override fun execute() {  }

    override fun isFinished(): Boolean = true
}
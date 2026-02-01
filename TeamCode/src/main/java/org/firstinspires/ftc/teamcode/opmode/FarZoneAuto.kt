package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.command.transfer.TransferSequence
import org.firstinspires.ftc.teamcode.hardware.Globals.FAR
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.opmode.template.BaseTemplate

@Autonomous
class FarZoneAuto : BaseTemplate() {
    override fun initialize() {

    }

    override fun start() {
        resetRuntime()

        Robot.scheduler.schedule(
            TransferSequence(FAR)
        )

        Robot.follower.startTeleopDrive()
    }

    override fun cycle() {
    }
}
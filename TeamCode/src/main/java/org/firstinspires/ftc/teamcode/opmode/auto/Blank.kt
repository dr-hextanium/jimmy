package org.firstinspires.ftc.teamcode.opmode.auto

import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.opmode.template.AutoTemplate

@Autonomous(name = "Blank Auto")
class BlankAuto : AutoTemplate(Pose()) {
    val command by lazy { }

    override fun initialize() {
        command
        follower.setStartingPose(start)
    }

    override fun start() {
        super.start()

        Robot.scheduler.schedule(
            SequentialCommandGroup(

            ),
        )
    }
}
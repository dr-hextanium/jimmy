package org.firstinspires.ftc.teamcode.opmode.auto

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.command.auto.PedroCommand
import org.firstinspires.ftc.teamcode.command.intake.SpinIntake
import org.firstinspires.ftc.teamcode.command.intake.StopIntake
import org.firstinspires.ftc.teamcode.command.launcher.Manual
import org.firstinspires.ftc.teamcode.command.transfer.Transfer
import org.firstinspires.ftc.teamcode.command.transfer.TransferSequence
import org.firstinspires.ftc.teamcode.hardware.Globals.BETTER_FAR
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.opmode.template.AutoTemplate
import org.firstinspires.ftc.teamcode.paths.BlueFar3
import kotlin.math.PI

@Autonomous(name = "Blue Far Zone 3")
class BlueFar3Auto : AutoTemplate(Pose(55.0, 7.5, PI / 2)) {
    override fun start() {
        super.start()

        Robot.scheduler.schedule(
            SequentialCommandGroup(
                Manual { 1.0 },
                PedroCommand(
                    BlueFar3(follower).Path1, follower
                ),
                TransferSequence(BETTER_FAR),
                WaitCommand(1000),
                PedroCommand(
                    BlueFar3(follower).Path2, follower
                ),
                ParallelCommandGroup(SpinIntake(), Transfer { -1.0 }),
                PedroCommand(
                    BlueFar3(follower).Path3, follower
                ),
                ParallelCommandGroup(StopIntake(), Transfer { 0.0 }),
                PedroCommand(
                    BlueFar3(follower).Path4, follower
                ),
                PedroCommand(
                    BlueFar3(follower).Path5, follower
                ),
                WaitCommand(1000),
                TransferSequence(BETTER_FAR),
            ),
        )
    }

}
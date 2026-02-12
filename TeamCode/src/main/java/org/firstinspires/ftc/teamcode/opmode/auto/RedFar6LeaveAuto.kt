package org.firstinspires.ftc.teamcode.opmode.auto

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.command.auto.PedroCommand
import org.firstinspires.ftc.teamcode.command.intake.SpinIntake
import org.firstinspires.ftc.teamcode.command.launcher.Manual
import org.firstinspires.ftc.teamcode.command.transfer.Transfer
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.opmode.template.AutoTemplate
import org.firstinspires.ftc.teamcode.paths.RedFar6Leave
import kotlin.math.PI

@Autonomous(name = "Red Far 6 + Leave")
class RedFar6LeaveAuto : AutoTemplate(Pose(89.0, 7.5, PI / 2)) {
    val command by lazy {
        SequentialCommandGroup(
            // score preloads
            Manual { 0.93 },
            PedroCommand(
                RedFar6Leave(follower).LaunchPreloads, follower
            ),
            WaitCommand(200),
            ParallelCommandGroup(
                Transfer { 1.0 },
                SpinIntake()
            ),
            WaitCommand(1500),
            Transfer { -1.0 },

            // score first spike
            PedroCommand(
                RedFar6Leave(follower).IntakeHumanPlayer, follower
            ),
            WaitCommand(1000),
            Transfer { -1.0 },

            PedroCommand(
                RedFar6Leave(follower).GoBackToShoot, follower
            ),
            Transfer { -1.0 },

            PedroCommand(
                RedFar6Leave(follower).ShootHumanPlayer, follower
            ),
            Transfer { -1.0 },

            WaitCommand(1000),
            ParallelCommandGroup(
                Transfer { 1.0 },
                SpinIntake()
            ),
            WaitCommand(2000),

            Transfer { 0.0 },
            Manual { 0.0 },
            PedroCommand(
                RedFar6Leave(follower).Leave, follower
            ),
        )
    }

    override fun initialize() {
        command
        follower.setStartingPose(start)
    }


    override fun start() {
        super.start()

        Robot.scheduler.schedule(
            command
        )
    }

}
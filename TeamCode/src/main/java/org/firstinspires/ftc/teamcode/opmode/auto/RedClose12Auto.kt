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
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.opmode.template.AutoTemplate
import org.firstinspires.ftc.teamcode.paths.RedClose12
import kotlin.math.PI

@Autonomous(name = "Red Close 12")
class RedClose12Auto : AutoTemplate(Pose(112.0, 136.5, PI / 2)) {
    val command by lazy {
        SequentialCommandGroup(
            // score preloads
            Manual { 0.71 },
            PedroCommand(
                RedClose12(follower).ScorePreloads, follower
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
                RedClose12(follower).IntakeSpike1, follower
            ),

            StopIntake(),
            WaitCommand(500),

            PedroCommand(
                RedClose12(follower).ScoreSpike1, follower
            ),
            WaitCommand(200),
            ParallelCommandGroup(
                Transfer { 1.0 },
                SpinIntake()
            ),
            WaitCommand(1500),
            Transfer { -1.0 },

            // score second spike
            PedroCommand(
                RedClose12(follower).AlignSpike2, follower
            ),
            PedroCommand(
                RedClose12(follower).IntakeSpike2, follower
            ),
            StopIntake(),
            WaitCommand(500),
            PedroCommand(
                RedClose12(follower).ScoreSpike2, follower
            ),
            WaitCommand(200),
            SpinIntake(),
            ParallelCommandGroup(
                Transfer { 1.0 },
                SpinIntake()
            ),
            WaitCommand(1500),
            Transfer { -1.0 },

            // score third spike
            PedroCommand(
                RedClose12(follower).PrepAlignSpike3, follower
            ),
            PedroCommand(
                RedClose12(follower).IntakeSpike3, follower
            ),
            WaitCommand(250),
            StopIntake(),
            PedroCommand(
                RedClose12(follower).ScoreSpike3Part1, follower
            ),
            PedroCommand(
                RedClose12(follower).ScoreSpike3Part2, follower
            ),
            WaitCommand(200),
            ParallelCommandGroup(
                Transfer { 1.0 },
                SpinIntake()
            ),
            WaitCommand(1500),
            Transfer { 0.0 },
            Manual { 0.0 },
            PedroCommand(
                RedClose12(follower).Leave, follower
            ),
            StopIntake()
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
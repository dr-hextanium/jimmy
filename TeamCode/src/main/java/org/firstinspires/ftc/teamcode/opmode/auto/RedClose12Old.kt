package org.firstinspires.ftc.teamcode.opmode.auto

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.command.auto.PedroCommand
import org.firstinspires.ftc.teamcode.command.intake.IntakeWithGateClosed
import org.firstinspires.ftc.teamcode.command.launcher.FeedLauncherArtifacts
import org.firstinspires.ftc.teamcode.command.launcher.ManualHood
import org.firstinspires.ftc.teamcode.command.launcher.ManuallyLaunch
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.opmode.template.AutoTemplate
import org.firstinspires.ftc.teamcode.paths.RedClose12
import kotlin.math.PI

@Autonomous(name = "Red Close 12")
class RedClose12Old : AutoTemplate(Pose(112.0, 136.5, PI / 2)) {
    val command by lazy {
        SequentialCommandGroup(
            // score preloads
            ParallelCommandGroup(IntakeWithGateClosed(), ManuallyLaunch { 0.64 }, ManualHood { 0.2 }),

            PedroCommand(
                RedClose12(follower).ScorePreloads, follower
            ),
            WaitCommand(200),
            FeedLauncherArtifacts(),
            WaitCommand(1500),
            IntakeWithGateClosed(),

            // score first spike
            PedroCommand(
                RedClose12(follower).IntakeSpike1, follower
            ),

            WaitCommand(500),

            PedroCommand(
                RedClose12(follower).ScoreSpike1, follower
            ),
            WaitCommand(200),
            FeedLauncherArtifacts(),
            WaitCommand(1500),
            IntakeWithGateClosed(),

            // score second spike
            PedroCommand(
                RedClose12(follower).AlignSpike2, follower
            ),
            PedroCommand(
                RedClose12(follower).IntakeSpike2, follower
            ),
            WaitCommand(500),
            PedroCommand(
                RedClose12(follower).ScoreSpike2, follower
            ),
            WaitCommand(200),
            FeedLauncherArtifacts(),
            WaitCommand(1500),
            IntakeWithGateClosed(),

            // score third spike
            PedroCommand(
                RedClose12(follower).PrepAlignSpike3, follower
            ),
            PedroCommand(
                RedClose12(follower).IntakeSpike3, follower
            ),
            WaitCommand(250),
            PedroCommand(
                RedClose12(follower).ScoreSpike3Part1, follower
            ),
            PedroCommand(
                RedClose12(follower).ScoreSpike3Part2, follower
            ),
            WaitCommand(200),
            FeedLauncherArtifacts(),

            WaitCommand(1500),
            IntakeWithGateClosed(),
            PedroCommand(
                RedClose12(follower).Leave, follower
            ),

            InstantCommand({ stop() })
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
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
import org.firstinspires.ftc.teamcode.paths.BlueClose12PathsOld
import kotlin.math.PI

@Autonomous(name = "Blue Close 12 Old")
class BlueClose12Old : AutoTemplate(Pose(32.0, 136.5, PI / 2)) {
    val command by lazy {
        SequentialCommandGroup(
            // score preloads
            ParallelCommandGroup(IntakeWithGateClosed(), ManuallyLaunch { 0.64 }, ManualHood { 0.2 }),

            PedroCommand(
                BlueClose12PathsOld(follower).ScorePreloads, follower
            ),
            WaitCommand(200),
            FeedLauncherArtifacts(),
            WaitCommand(1500),
            IntakeWithGateClosed(),

            // score first spike
            PedroCommand(
                BlueClose12PathsOld(follower).IntakeSpike1, follower
            ),

            WaitCommand(500),

            PedroCommand(
                BlueClose12PathsOld(follower).ScoreSpike1, follower
            ),
            WaitCommand(200),
            FeedLauncherArtifacts(),
            WaitCommand(1500),
            IntakeWithGateClosed(),

            // score second spike
            PedroCommand(
                BlueClose12PathsOld(follower).AlignSpike2, follower
            ),
            PedroCommand(
                BlueClose12PathsOld(follower).IntakeSpike2, follower
            ),
            WaitCommand(500),
            PedroCommand(
                BlueClose12PathsOld(follower).ScoreSpike2, follower
            ),
            WaitCommand(200),
            FeedLauncherArtifacts(),
            WaitCommand(1500),
            IntakeWithGateClosed(),

            // score third spike
            PedroCommand(
                BlueClose12PathsOld(follower).PrepAlignSpike3, follower
            ),
            PedroCommand(
                BlueClose12PathsOld(follower).IntakeSpike3, follower
            ),
            WaitCommand(250),
            PedroCommand(
                BlueClose12PathsOld(follower).ScoreSpike3Part1, follower
            ),
            PedroCommand(
                BlueClose12PathsOld(follower).ScoreSpike3Part2, follower
            ),
            WaitCommand(200),
            FeedLauncherArtifacts(),
            WaitCommand(1500),
            IntakeWithGateClosed(),

            PedroCommand(
                BlueClose12PathsOld(follower).Leave, follower
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
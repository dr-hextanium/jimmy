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
import org.firstinspires.ftc.teamcode.paths.RedCloseElims12GatePath
import kotlin.math.PI

@Autonomous(name = "Red Close 12 Elims Gate")
class RedClose12ElimsGate : AutoTemplate(Pose(112.0, 136.5, PI / 2)) {
    val command by lazy {
        SequentialCommandGroup(
            // score preloads
            ParallelCommandGroup(IntakeWithGateClosed(), ManuallyLaunch { 0.64 }, ManualHood { 0.2 }),

            PedroCommand(
                RedCloseElims12GatePath(follower).ScorePreloads, follower
            ),
            WaitCommand(200),
            FeedLauncherArtifacts(),
            WaitCommand(1500),
            IntakeWithGateClosed(),

            // score first spike
            PedroCommand(
                RedCloseElims12GatePath(follower).IntakeSpike1, follower
            ),

            // score first spike
            PedroCommand(
                RedCloseElims12GatePath(follower).Path12, follower
            ),
            WaitCommand(500),

            PedroCommand(
                RedCloseElims12GatePath(follower).ScoreSpike1, follower
            ),
            WaitCommand(200),
            FeedLauncherArtifacts(),
            WaitCommand(1500),
            IntakeWithGateClosed(),

            // score second spike
            PedroCommand(
                RedCloseElims12GatePath(follower).AlignSpike2, follower
            ),
            PedroCommand(
                RedCloseElims12GatePath(follower).IntakeSpike2, follower
            ),
            WaitCommand(500),
            PedroCommand(
                RedCloseElims12GatePath(follower).ScoreSpike2, follower
            ),
            WaitCommand(200),
            FeedLauncherArtifacts(),
            WaitCommand(1500),
            IntakeWithGateClosed(),

            // score third spike
            PedroCommand(
                RedCloseElims12GatePath(follower).PrepAlignSpike3, follower
            ),
            PedroCommand(
                RedCloseElims12GatePath(follower).IntakeSpike3, follower
            ),
            WaitCommand(250),
            PedroCommand(
                RedCloseElims12GatePath(follower).ScoreSpike3Part1, follower
            ),
            PedroCommand(
                RedCloseElims12GatePath(follower).ScoreSpike3Part2, follower
            ),
            WaitCommand(200),
            FeedLauncherArtifacts(),

            WaitCommand(1500),
            IntakeWithGateClosed(),
            PedroCommand(
                RedCloseElims12GatePath(follower).Leave, follower
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
package org.firstinspires.ftc.teamcode.opmode.auto

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.command.auto.PedroCommand
import org.firstinspires.ftc.teamcode.command.intake.IntakeWithGateClosed
import org.firstinspires.ftc.teamcode.command.launcher.FeedLauncherArtifacts
import org.firstinspires.ftc.teamcode.command.launcher.ManualHood
import org.firstinspires.ftc.teamcode.command.launcher.ManuallyLaunch
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.opmode.template.AutoTemplate
import org.firstinspires.ftc.teamcode.paths.BlueClose12Paths

@Autonomous(name = "Blue Close 12")
class BlueClose12 : AutoTemplate(Pose(26.0, 130.0, Math.toRadians(143.0))) {
    val command by lazy {
        fun construct(pathChain: PathChain): PedroCommand {
            return PedroCommand(pathChain, follower)
        }

        SequentialCommandGroup(
            // score preloads
            ParallelCommandGroup(IntakeWithGateClosed(), ManuallyLaunch { 0.65 }, ManualHood { 0.194 }),
            construct(BlueClose12Paths(follower).ScorePreloads),
            FeedLauncherArtifacts(),
            WaitCommand(750),

            // intake spike 2
            IntakeWithGateClosed(),
            construct(BlueClose12Paths(follower).IntakeSpike2),
            WaitCommand(750),
            construct(BlueClose12Paths(follower).OpenGate),
            WaitCommand(500),
            construct(BlueClose12Paths(follower).PrepScoreSpike2),
            construct(BlueClose12Paths(follower).ScoreSpike2),
            FeedLauncherArtifacts(),
            WaitCommand(750),

            IntakeWithGateClosed(),
            construct(BlueClose12Paths(follower).IntakeSpike1),
            WaitCommand(750),
            construct(BlueClose12Paths(follower).PrepScoreSpike1),
            construct(BlueClose12Paths(follower).ScoreSpike1),
            FeedLauncherArtifacts(),
            WaitCommand(750),

            IntakeWithGateClosed(),
            construct(BlueClose12Paths(follower).IntakeSpike3),
            WaitCommand(750),
            construct(BlueClose12Paths(follower).PrepScoreSpike3),
            construct(BlueClose12Paths(follower).ScoreSpike3),
            FeedLauncherArtifacts(),
            WaitCommand(750),

            InstantCommand({ stop() })
        )
    }

    override fun initialize() {
        command
        follower.setStartingPose(start)
        Robot.Subsystems.launcher.targetHoodByScalar(0.375)
    }

    override fun start() {
        super.start()

        Robot.scheduler.schedule(
            SequentialCommandGroup(
                command
            ),
        )
    }

    override fun cycle() {
        super.cycle()
    }
}
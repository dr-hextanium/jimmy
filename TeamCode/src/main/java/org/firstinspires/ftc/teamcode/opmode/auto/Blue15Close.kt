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
import org.firstinspires.ftc.teamcode.paths.Blue15ClosePaths

@Autonomous(name = "Blue 15 Close")
class Blue15Close : AutoTemplate(Pose(26.0, 130.0, Math.toRadians(143.0))) {
    val command by lazy {
        fun construct(pathChain: PathChain): PedroCommand {
            return PedroCommand(pathChain, follower)
        }

//        launcher.targetTPSByScalar(powerRegression(distanceToGoal()).coerceIn(POWER_LOWER_BOUND, POWER_UPPER_BOUND))
//        launcher.targetHoodByScalar(hoodRegression(distanceToGoal()).coerceIn(HOOD_LOWER_BOUND, HOOD_UPPER_BOUND))

        SequentialCommandGroup(
            // score preloads
            ParallelCommandGroup(IntakeWithGateClosed(), ManuallyLaunch { 0.65 }, ManualHood { 0.194 }),
            construct(Blue15ClosePaths(follower).ScorePreloads),
            FeedLauncherArtifacts(),
            WaitCommand(750),

            // intake spike 2
            IntakeWithGateClosed(),
            construct(Blue15ClosePaths(follower).IntakeSpike2),

            // score spike 2
            WaitCommand(500),
            construct(Blue15ClosePaths(follower).ScoreSpike2),
            FeedLauncherArtifacts(),
            WaitCommand(750),

            // open gate 1
            IntakeWithGateClosed(),
            construct(Blue15ClosePaths(follower).OpenGate1),
            construct(Blue15ClosePaths(follower).IntakeGate1),

            // score gate 1
            WaitCommand(1500),
            construct(Blue15ClosePaths(follower).ScoreGate1),
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
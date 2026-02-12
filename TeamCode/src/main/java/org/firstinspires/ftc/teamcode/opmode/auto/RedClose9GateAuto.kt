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
import org.firstinspires.ftc.teamcode.paths.RedClose9Gate
import kotlin.math.PI

@Autonomous(name = "Red Close 9 + Open Gate")
class RedClose9GateAuto : AutoTemplate(Pose(112.0, 136.5, PI / 2)) {
    val command by lazy {
        SequentialCommandGroup(
            // score preloads
            Manual { 0.71 },
            PedroCommand(
                RedClose9Gate(follower).ScorePreloads, follower
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
                RedClose9Gate(follower).IntakeSpike1, follower
            ),
            PedroCommand(
                RedClose9Gate(follower).OpenGate, follower
            ),
            PedroCommand(
                RedClose9Gate(follower).ScoreSpike1, follower
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
                RedClose9Gate(follower).AlignSpike2, follower
            ),
            PedroCommand(
                RedClose9Gate(follower).IntakeSpike2, follower
            ),
            PedroCommand(
                RedClose9Gate(follower).ScoreSpike2, follower
            ),
            WaitCommand(200),
            ParallelCommandGroup(
                Transfer { 1.0 },
                SpinIntake()
            ),
            WaitCommand(1500),
            StopIntake(),
            WaitCommand(1500),
            Transfer { 0.0 },
            Manual { 0.0 },
            PedroCommand(
                RedClose9Gate(follower).Leave, follower
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
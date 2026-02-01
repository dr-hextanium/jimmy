package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.command.intake.SpinIntake
import org.firstinspires.ftc.teamcode.command.intake.StopIntake
import org.firstinspires.ftc.teamcode.command.launcher.Manual
import org.firstinspires.ftc.teamcode.hardware.PowerDelayPair
import org.firstinspires.ftc.teamcode.hardware.Robot

class BetterTransferSequence(set: Triple<PowerDelayPair, PowerDelayPair, PowerDelayPair>) : SequentialCommandGroup(
    Manual { set.first.power },
    WaitUntilCommand { Robot.Subsystems.launcher.isReady }
        .withTimeout(2000L),

    ParallelCommandGroup(
        SpinIntake(), // Helps push ball in
        Transfer()    // Feeds ball to flywheel
    ),
)

class ShootThree() : SequentialCommandGroup(
    Manual { 0.84 },
    WaitUntilCommand { Robot.Subsystems.launcher.isReady }
        .withTimeout(2000L),

    ParallelCommandGroup(
        SpinIntake(), // Helps push ball in
        Transfer()    // Feeds ball to flywheel
    ),

    WaitCommand(250),

    ParallelCommandGroup(
        StopIntake(),
        StopTransfer()
    ),

    WaitCommand(500),

    ParallelCommandGroup(
        SpinIntake(), // Helps push ball in
        Transfer()    // Feeds ball to flywheel
    ),

    WaitCommand(250),

    ParallelCommandGroup(
        StopIntake(),
        StopTransfer()
    ),

    WaitCommand(500),

    ParallelCommandGroup(
        SpinIntake(), // Helps push ball in
        Transfer()    // Feeds ball to flywheel
    ),

    WaitCommand(250),

    ParallelCommandGroup(
        StopIntake(),
        StopTransfer()
    ),
)
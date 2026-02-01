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
import org.firstinspires.ftc.teamcode.hardware.subsystem.Launcher

class TransferSequence(set: Triple<PowerDelayPair, PowerDelayPair, PowerDelayPair>) : SequentialCommandGroup() {
    // Tuning Constants (in Milliseconds)
    private val FEED_TIME = 225L     // How long to run rollers to eject ball
    private val SPIN_UP_TIMEOUT = 2000L // Max time to wait for spinup before firing anyway
    private val RECOVERY_TIMEOUT = 1200L // Max time to wait between shots

    init {
        addCommands(
            // ==================== SHOT 1 ====================
            Manual { set.first.power }, // Set Target Speed

            // Wait for sensor "isReady" OR 4 seconds max (prevents hanging)
            WaitUntilCommand { Robot.Subsystems.launcher.isReady }
                .withTimeout(SPIN_UP_TIMEOUT),

            // Fire!
            ParallelCommandGroup(
                SpinIntake(), // Helps push ball in
                Transfer()    // Feeds ball to flywheel
            ),
            WaitCommand(FEED_TIME), // Ensure ball leaves

            // Reset Feeders
            Transfer { -0.5 },
            StopIntake(),

            // ==================== SHOT 2 ====================
            ParallelCommandGroup(
                // Wait user delay AND set new power
                WaitCommand(set.first.delay),
                Manual { set.second.power }
            ),

            // Wait for new speed (acceleration or deceleration)
            WaitUntilCommand { Robot.Subsystems.launcher.isReady }
                .withTimeout(RECOVERY_TIMEOUT),

            ParallelCommandGroup(
                SpinIntake(),
                Transfer()
            ),
            WaitCommand(FEED_TIME),
            Transfer { -0.5 },
            StopIntake(),

            // ==================== SHOT 3 ====================
            ParallelCommandGroup(
                WaitCommand(set.second.delay),
                Manual { set.third.power }
            ),

            WaitUntilCommand { Robot.Subsystems.launcher.isReady }
                .withTimeout(RECOVERY_TIMEOUT),

            ParallelCommandGroup(
                SpinIntake(),
                Transfer()
            ),
            WaitCommand(set.third.delay),

            // ==================== CLEANUP ====================
            StopIntake(),
            Transfer { 0.0 },
            Manual { Launcher.IDLE_TPS / Launcher.MAX_TPS } // bring to idle tps
        )
    }
}
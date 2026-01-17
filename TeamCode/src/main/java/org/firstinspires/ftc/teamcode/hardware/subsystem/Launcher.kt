package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.utility.absPercentDifference
import kotlin.math.abs

class Launcher(val left: DcMotorEx, val right: DcMotorEx) : ISubsystem {
    val motors = listOf(left, right)

    // The "Final" target the user wants
    var targetTPS = 0.0

    // The "Intermediate" target we send to the motor to ramp up slowly
    private var currentRampedTPS = 0.0

    val averageTPS: Double
        get() = (abs(left.velocity) + abs(right.velocity)) / 2.0

    private var wasReady = false

    val atSpeed: Boolean
        get() {
            // Guard: If we aren't trying to spin (or target is very low), we aren't "Ready" to shoot
            if (targetTPS < 100.0) return false
            // We compare actual velocity against the FINAL target, not the ramped target
            return absPercentDifference(averageTPS, targetTPS) <= AT_SPEED_TOLERANCE
        }

    val withinSafetyMargins: Boolean
        get() = absPercentDifference(left.velocity, right.velocity) <= MAXIMUM_DEVIANCE

    val gamepad by lazy { Robot.gamepad1.gamepad }

    val isReady: Boolean
        get() = atSpeed

    fun distanceToTPS(distance: Double): Double {
        val calculated = (distance * TPS_PER_INCH) + BASE_RPM
        return calculated.coerceIn(0.0, MAX_TPS)
    }

    fun targetTPSByDistance(distance: Double) {
        targetTPS = distanceToTPS(distance)
    }

    // This API remains exactly the same
    fun targetTPSByScalar(scale: Double) {
        targetTPS = scaleToTPS(scale)
    }

    override fun reset() {
        wasReady = false
        targetTPS = 0.0
        currentRampedTPS = 0.0 // Reset the ramp logic

        right.direction = REVERSE
        left.direction = REVERSE

        motors.forEach {
            it.zeroPowerBehavior = FLOAT
            it.power = 0.0
            it.velocity = 0.0

            it.setCurrentAlert(15.0, CurrentUnit.AMPS)

            // Correct Order: Reset Encoder FIRST, then set RunMode
            it.mode = RunMode.STOP_AND_RESET_ENCODER
            it.mode = RunMode.RUN_USING_ENCODER
        }
    }

    override fun read() {  }

    override fun update() {
        // --- 1. Slew Rate Limiter (The Belt Saver) ---
        // Calculate how far we are from the target
        val error = targetTPS - currentRampedTPS

        // Limit the change per loop to prevent torque spikes
        // coerceIn ensures we don't jump more than RAMP_RATE in one cycle
        val step = error.coerceIn(-RAMP_RATE_PER_LOOP, RAMP_RATE_PER_LOOP)

        // Apply the step
        currentRampedTPS += step
        // ---------------------------------------------

        // 2. Logic Updates
        val nowReady = isReady

        if (nowReady != wasReady) {
            if (nowReady) {
                signalAtSpeed(gamepad)
            } else {
                signalWrongSpeed(gamepad)
            }
            wasReady = nowReady
        }

        Robot.telemetry.addData("Launcher Target TPS", targetTPS)
        Robot.telemetry.addData("Launcher Ramped TPS", currentRampedTPS)
        Robot.telemetry.addData("Left Vel", left.velocity)
        Robot.telemetry.addData("Right Vel", right.velocity)
    }

    override fun write() {
        // CRITICAL: We write the RAMPED value, not the raw target
        motors.forEach { it.velocity = currentRampedTPS }
    }

    fun scaleToTPS(scale: Double) = scale * MAX_TPS

    fun signalAtSpeed(gamepad: Gamepad) {
        gamepad.runLedEffect(AT_SPEED_LED_EFFECT)
        gamepad.runRumbleEffect(AT_SPEED_RUMBLE_EFFECT)
    }

    fun signalWrongSpeed(gamepad: Gamepad) {
        gamepad.runLedEffect(WRONG_SPEED_LED_EFFECT)
        gamepad.runRumbleEffect(WRONG_SPEED_RUMBLE_EFFECT)
    }

    companion object {
        const val MAX_TPS = 2100.0
        const val IDLE_TPS = 1000.0

        // SAFETY TUNING
        // If belts still skip, LOWER this number (e.g. to 40.0).
        // If it speeds up too slowly, INCREASE this number (e.g. to 100.0).
        // 60.0 adds ~3000 TPS per second (assuming 50Hz loop), taking ~0.7s to reach max speed.
        const val RAMP_RATE_PER_LOOP = 1000.0

        const val MAXIMUM_DEVIANCE = 0.1
        const val AT_SPEED_TOLERANCE = 0.05 // 5% tolerance

        const val BASE_RPM = 0.0
        const val TPS_PER_INCH = 0.0

        val AT_SPEED_LED_EFFECT = Gamepad.LedEffect.Builder()
            .addStep(0.0, 1.0, 0.0, 100)
            .addStep(0.0, 0.0, 0.0, 100)
            .addStep(0.0, 1.0, 0.0, 100)
            .addStep(0.0, 0.0, 0.0, 100)
            .addStep(0.0, 1.0, 0.0, 100)
            .addStep(0.0, 0.0, 0.0, 100)
            .addStep(0.0, 1.0, 0.0, 1000)
            .build()

        val AT_SPEED_RUMBLE_EFFECT = Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 100)
            .addStep(0.0, 0.0, 100)
            .addStep(0.75, 0.75, 100)
            .addStep(0.0, 0.0, 100)
            .addStep(0.5, 0.5, 100)
            .addStep(0.0, 0.0, 100)
            .build()

        val WRONG_SPEED_LED_EFFECT = Gamepad.LedEffect.Builder()
            .addStep(1.0, 0.0, 0.0, 1300)
            .build()

        val WRONG_SPEED_RUMBLE_EFFECT = Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 300)
            .build()

        val SHOT_FIRST_LED_EFFECT = Gamepad.LedEffect.Builder()
            .addStep(1.0, 1.0, 0.0, 50)
            .build()

        val SHOT_SECOND_LED_EFFECT = Gamepad.LedEffect.Builder()
            .addStep(1.0, 1.0, 0.0, 50)
            .addStep(0.0, 0.0, 0.0, 50)
            .addStep(1.0, 1.0, 0.0, 50)
            .build()

        val SHOT_THIRD_LED_EFFECT = Gamepad.LedEffect.Builder()
            .addStep(1.0, 1.0, 0.0, 50)
            .addStep(0.0, 0.0, 0.0, 50)
            .addStep(1.0, 1.0, 0.0, 50)
            .addStep(0.0, 0.0, 0.0, 50)
            .addStep(1.0, 1.0, 0.0, 50)
            .build()
    }
}
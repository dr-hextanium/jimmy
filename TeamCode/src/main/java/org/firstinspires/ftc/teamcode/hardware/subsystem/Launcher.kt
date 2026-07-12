package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.control.ShooterModel
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.utility.absPercentDifference
import org.firstinspires.ftc.teamcode.utility.map
import org.firstinspires.ftc.teamcode.utility.percentDifference
import kotlin.math.abs

/**
 * Flywheel launcher: a single flywheel driven 1:1 by two motors, plus a servo-actuated hood.
 *
 * Speed control is feedforward-first: the flywheel power is `kS + kV*targetTPS + kP*(targetTPS -
 * measuredTPS)`, clamped to [0, 1]. The feedforward `kV*targetTPS` (with kV ~ 1/MAX_TPS) alone
 * holds roughly the right speed, so even at kP = 0 the loop behaves sanely; the small proportional
 * term saturates to full power while spinning up and trims to zero at speed. This is deliberately
 * not a proportional-dominant loop -- a mistuned kP must not be able to leave the flywheel unable
 * to reach speed, since both teleop and every auto drive the flywheel through here.
 *
 * There are two ways to set the target:
 *  - [targetTPSByScalar] / [targetHoodByScalar]: the manual, field-tested 0..1 path (driver trims,
 *    autos' hand-tuned constants).
 *  - [aimAtDistance]: the kinematic path -- [ShooterModel] turns a field distance into the flywheel
 *    speed and hood angle that physically reach the goal.
 */
class Launcher(val left: DcMotorEx, val right: DcMotorEx, val hood: Servo) : ISubsystem {
    val motors = listOf(left, right)

    var currentPower = 0.0

    var targetTPS = 0.0
    var targetHoodPosition = 0.0

    val averageTPS: Double
        get() = (abs(left.velocity) + abs(right.velocity)) / 2.0

    val atSpeed: Boolean
        get() {
            if (targetTPS < MIN_TPS) return false
            return absPercentDifference(averageTPS, targetTPS) <= AT_SPEED_TOLERANCE
        }

    val isReady: Boolean
        get() = atSpeed

    fun scaleToTPS(scale: Double) = scale * MAX_TPS

    fun targetTPSByScalar(scale: Double) {
        targetTPS = scaleToTPS(scale)
    }

    fun targetHoodByScalar(scale: Double) {
        targetHoodPosition = map(scale, 0.0, 1.0, HOOD_HIGH, HOOD_LOW)
    }

    /** Aim for a target [distanceInches] downrange: set flywheel speed and hood from [ShooterModel]. */
    fun aimAtDistance(distanceInches: Double) {
        val solution = ShooterModel.aim(distanceInches)
        targetTPS = solution.targetTps
        targetHoodPosition = solution.hoodServoPosition
    }

    override fun reset() {
        targetTPS = 0.0
        currentPower = 0.0

        right.direction = REVERSE
        left.direction = DcMotorSimple.Direction.FORWARD

        motors.forEach {
            it.zeroPowerBehavior = BRAKE
            it.power = 0.0
            it.velocity = 0.0

            it.setCurrentAlert(15.0, CurrentUnit.AMPS)

            it.mode = RunMode.STOP_AND_RESET_ENCODER
            it.mode = RunMode.RUN_WITHOUT_ENCODER
        }

        hood.position = HOOD_HIGH
    }

    override fun read() {  }

    override fun update() {
        // Feedforward-first velocity control. The flywheel is single-direction, so power is clamped
        // to [0, 1]: it can spin the wheel up but never actively brakes it.
        currentPower = if (targetTPS < MIN_TPS) {
            0.0
        } else {
            (kS + kV * targetTPS + kP * (targetTPS - averageTPS)).coerceIn(0.0, 1.0)
        }

        Robot.telemetry.addData("Launcher Target TPS", targetTPS)
        Robot.telemetry.addData("Launcher Measured TPS", averageTPS)
        Robot.telemetry.addData("Launcher output power", currentPower)
        Robot.telemetry.addData("Launcher at speed?", isReady)
    }

    override fun write() {
        motors.forEach {
            if (abs(percentDifference(it.power, currentPower)) > 0.005) {
                it.power = currentPower
            }
        }

        hood.position = targetHoodPosition
    }

    companion object {
        const val HOOD_LOW = 0.905
        const val HOOD_HIGH = 0.25

        const val MIN_TPS = 100.0

        /** Manual-scalar ceiling and physics clamp share one source of truth. */
        val MAX_TPS get() = ShooterModel.MAX_TPS

        const val AT_SPEED_TOLERANCE = 0.05 // 5% tolerance

        // Feedforward-first velocity-controller gains (on-robot tunables).
        var kS = 0.0      // static power offset
        var kV = 0.0004   // power per TPS of target (~ 1 / MAX_TPS)
        var kP = 0.0003   // power per TPS of error (kept small; feedforward carries the load)
    }

    object Effects {
        fun signalAtSpeed(gamepad: Gamepad) {
            gamepad.runLedEffect(AT_SPEED_LED_EFFECT)
            gamepad.runRumbleEffect(AT_SPEED_RUMBLE_EFFECT)
        }

        fun signalWrongSpeed(gamepad: Gamepad) {
            gamepad.runLedEffect(WRONG_SPEED_LED_EFFECT)
            gamepad.runRumbleEffect(WRONG_SPEED_RUMBLE_EFFECT)
        }

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

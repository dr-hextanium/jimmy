package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.utility.absPercentDifference
import org.firstinspires.ftc.teamcode.utility.percentDifference
import kotlin.math.abs

class Launcher(val left: DcMotorEx, val right: DcMotorEx, val hood: Servo) : ISubsystem {
    val gamepad by lazy { Robot.gamepad1.gamepad }

    val motors = listOf(left, right)

    private var currentPower = 0.0

    var targetTPS = 0.0

    val averageTPS: Double
        get() = (abs(right.velocity))

    val atSpeed: Boolean
        get() {
            if (targetTPS < 100.0) return false
            return absPercentDifference(averageTPS, targetTPS) <= AT_SPEED_TOLERANCE
        }

    private var wasReady = false
    val isReady: Boolean
        get() = atSpeed

    fun scaleToTPS(scale: Double) = scale * MAX_TPS

    fun distanceToScalar(distance: Double): Double {
        val calculated = (distance * SCALAR_PER_INCH) + BASE_SCALAR
        return calculated.coerceIn(0.0, 1.0)
    }

    fun targetTPSByScalar(scale: Double) {
        targetTPS = scaleToTPS(scale)
    }

    override fun reset() {
        wasReady = false
        targetTPS = 0.0
        currentPower = 0.0

        right.direction = FORWARD
        left.direction = REVERSE

        motors.forEach {
            it.zeroPowerBehavior = FLOAT
            it.power = 0.0
            it.velocity = 0.0

            it.setCurrentAlert(15.0, CurrentUnit.AMPS)

            it.mode = RunMode.STOP_AND_RESET_ENCODER
            it.mode = RunMode.RUN_WITHOUT_ENCODER
        }

        hood.position = 0.0
    }

    override fun read() {  }

    override fun update() {
        val error = targetTPS - averageTPS

        if (error <= 0) {
            currentPower = 0.0
        } else {
            currentPower = 1.0
        }

        val nowReady = isReady

        if (nowReady != wasReady) {
//            if (nowReady) {
//                signalAtSpeed(gamepad)
//            } else {
//                signalWrongSpeed(gamepad)
//            }
            wasReady = nowReady
        }

        Robot.telemetry.addData("Launcher Target TPS", targetTPS)
        Robot.telemetry.addData("Launcher output power", currentPower)
        Robot.telemetry.addData("Left Vel", left.velocity)
        Robot.telemetry.addData("Right Vel", right.velocity)
    }

    override fun write() {
        motors.forEach {
            if (percentDifference(it.power, currentPower) > 0.005) {
                it.power = currentPower
            }
        }
    }


    companion object {
        const val MAX_TPS = 2100.0
        const val IDLE_TPS = 1000.0

        const val AT_SPEED_TOLERANCE = 0.05 // 5% tolerance

        const val BASE_SCALAR = 0.0
        const val SCALAR_PER_INCH = 0.0
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
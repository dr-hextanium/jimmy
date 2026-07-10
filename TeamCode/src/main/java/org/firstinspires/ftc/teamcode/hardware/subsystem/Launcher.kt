package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.utility.InterpLUT
import org.firstinspires.ftc.teamcode.utility.absPercentDifference
import org.firstinspires.ftc.teamcode.utility.map
import org.firstinspires.ftc.teamcode.utility.percentDifference
import kotlin.math.abs
import kotlin.math.pow

class Launcher(val left: DcMotorEx, val right: DcMotorEx, val hood: Servo) : ISubsystem {
    val gamepad by lazy { Robot.gamepad1.gamepad }

    val motors = listOf(left, right)

    var currentPower = 0.0

    var targetTPS = 0.0
    var targetHoodPosition = 0.0

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

    fun targetHoodByScalar(scale: Double) {
        targetHoodPosition = map(scale, 0.0, 1.0, HOOD_HIGH, HOOD_LOW)
    }

    override fun reset() {
        wasReady = false
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
        val error = targetTPS - averageTPS

        currentPower = if (error <= 0) {
            0.0
        } else {
            1.0
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
        Robot.telemetry.addData("Left Vel", right.velocity)
//        Robot.telemetry.addData("Right Vel", right.velocity)
    }

    override fun write() {
        motors.forEach {
            if (abs(percentDifference(it.power, currentPower)) > 0.005) {
                it.power = currentPower
            }
        }

        hood.position = targetHoodPosition
//        hood.position = HOOD_HIGH
    }

    object Regressions {
//        val POWER_LOWER_BOUND = 0.63
//        val POWER_UPPER_BOUND = 0.78

//        fun powerRegression(distance: Double): Double {
//            val a = -(2.13096e-7)
//            val b = 0.0000882897
//            val c = -0.00780014
//            val d = 0.840216
//
//            val output = (a * distance.pow(3)) + (b * distance.pow(2)) + (c * distance) + d
//
//            return output.coerceIn(POWER_LOWER_BOUND, POWER_UPPER_BOUND)
//        }

        val POWER_LOWER_BOUND = 0.53
        val POWER_UPPER_BOUND = 0.72

        fun powerRegression(distance: Double): Double {
            if (distance > 115.0) {
               return LookupTables.farPowerInterpLUT[distance]
            }

            val a = 0.495019
            val b = 1.00384

            val output = a * b.pow(distance)

            return output.coerceIn(POWER_LOWER_BOUND, POWER_UPPER_BOUND)
        }

        val HOOD_LOWER_BOUND = 0.475
        val HOOD_UPPER_BOUND = 0.88

        fun hoodRegression(distance: Double): Double {
            if (distance > 115.0) {
                return LookupTables.farHoodInterpLUT[distance]
            }

            val a = 6.55652e-7
            val b = -0.000162445
            val c = 0.00560722
            val d = 0.797042

            val output = (a * distance.pow(3)) + (b * distance.pow(2)) + (c * distance) + d

            return output.coerceIn(HOOD_LOWER_BOUND, HOOD_UPPER_BOUND)
        }

//        val HOOD_LOWER_BOUND = 0.22
//        val HOOD_UPPER_BOUND = 0.95
//
//        fun hoodRegression(distance: Double): Double {
//            val a = -0.00000481859
//            val b = 0.00109986
//            val c = -0.0799973
//            val d = 2.20371
//
//            val output = (a * distance.pow(3)) + (b * distance.pow(2)) + (c * distance) + d
//
//            return output.coerceIn(HOOD_LOWER_BOUND, HOOD_UPPER_BOUND)
//        }
    }

    object LookupTables {
//        // inches -> scalar
//        val powerInterpLUT = InterpLUT()
//            .add(27.5, 0.63)
//            .add(31.5, 0.76)
//            .add(70.0, 0.85)
//            .add(77.0, 0.78)
//            .also { it.createLUT() }
//
//        // inches -> degree
//        val hoodInterpLUT = InterpLUT()
//            .add(27.5, 0.95)
//            .add(31.5, 0.43)
//            .add(70.0, 0.29)
//            .add(77.0, 0.45)
//            .also { it.createLUT() }
//
//        // inches -> seconds
//        val timeInterpLUT = InterpLUT()
//            .add(30.0, 0.0)
//            .add(120.0, 2.0)
//            .also { it.createLUT() }

        val farPowerInterpLUT = InterpLUT()
            .add(122.2, 0.82)
            .add(130.0, 0.89)
            .also { it.createLUT() }

        val farHoodInterpLUT = InterpLUT()
            .add(122.2, 0.03)
            .add(130.0, 0.185)
            .also { it.createLUT() }
    }

    companion object {
        const val HOOD_LOW = 0.905
        const val HOOD_HIGH = 0.25

        const val MIN_TPS = 100.0
        const val IDLE_TPS = 1000.0
        const val MAX_TPS = 2500.0

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
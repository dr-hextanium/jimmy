package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.hardware.Names
import kotlin.math.abs
import kotlin.math.sin

/**
 * Standalone shooter bench: exercise the two launcher motors and the hood servo in isolation, with
 * nothing else plugged in. Direct hardwareMap access -- no [org.firstinspires.ftc.teamcode.hardware.Robot],
 * no subsystems, no drivetrain, no PID -- so it only needs the `sl`, `sr`, and `hood` devices in the
 * config. Everything is open-loop: dpad steps a single shared power to both motors and you read back
 * each motor's velocity.
 *
 * The hood is driven by the right trigger mapped into an adjustable travel window [min, max]: the
 * trigger's 0..1 sweeps the servo across the current window, and you shrink/grow that window on the
 * fly (dpad for the max, left stick X for the min) to hunt safe endpoints without slamming the servo.
 *
 * Controls (gamepad1):
 *  - dpad up / down     : shared flywheel power +/- 0.05 (never negative -- use the bumpers to reverse)
 *  - A (cross)          : immediate shooter stop (power -> 0)
 *  - left / right bumper : toggle LEFT / RIGHT motor direction (same commanded power)
 *  - B (circle)         : toggle BRAKE / FLOAT zero-power behavior (both motors)
 *  - right trigger      : hood position within the window (0 = min end, full pull = max end)
 *  - dpad right / left  : grow / shrink the hood window max (+/- 0.02)
 *  - left stick X       : move the hood window min
 *  - X (square)         : toggle servo direction (reverse, flips the 0..1 mapping)
 *  - Y (triangle)       : toggle sinusoidal hood sweep across the window (moving the trigger cancels it)
 */
@TeleOp(name = "Shooter Bench", group = "Debug")
class ShooterBench : OpMode() {
    private val left by lazy { hardwareMap.get(DcMotorEx::class.java, Names.Motors.Launcher.leftMotor) }
    private val right by lazy { hardwareMap.get(DcMotorEx::class.java, Names.Motors.Launcher.rightMotor) }
    private val hood by lazy { hardwareMap.get(Servo::class.java, Names.Servos.Launcher.servo) }

    private var sharedPower = 0.0

    // Start matching the Launcher subsystem's directions so "no toggles" == real robot behavior.
    private var leftForward = true
    private var rightForward = false
    private var brakeMode = true
    private var servoReversed = false

    // Hood travel window the right trigger maps into; starts at the full raw servo travel.
    private var hoodMin = 0.0
    private var hoodMax = 1.0

    private var sweeping = false
    private val sweepTimer = ElapsedTime()
    private var lastTrigger = 0f

    private var lastUp = false
    private var lastDown = false
    private var lastDpadLeft = false
    private var lastDpadRight = false
    private var lastLb = false
    private var lastRb = false
    private var lastA = false
    private var lastB = false
    private var lastX = false
    private var lastY = false

    override fun init() {
        listOf(left, right).forEach {
            it.mode = RunMode.STOP_AND_RESET_ENCODER
            it.mode = RunMode.RUN_WITHOUT_ENCODER
            it.power = 0.0
        }
        applyMotorDirections()
        applyBrakeMode()
        applyServoDirection()
    }

    override fun loop() {
        // --- shared flywheel power (rising-edge stepping so a held dpad doesn't ramp away) ---
        if (gamepad1.dpad_up && !lastUp) sharedPower += POWER_STEP
        if (gamepad1.dpad_down && !lastDown) sharedPower -= POWER_STEP
        if (gamepad1.a && !lastA) sharedPower = 0.0 // immediate stop
        sharedPower = Range.clip(sharedPower, 0.0, 1.0)

        // --- per-motor direction toggles (commanded power stays the same) ---
        if (gamepad1.left_bumper && !lastLb) { leftForward = !leftForward; applyMotorDirections() }
        if (gamepad1.right_bumper && !lastRb) { rightForward = !rightForward; applyMotorDirections() }

        // --- brake / float toggle ---
        if (gamepad1.b && !lastB) { brakeMode = !brakeMode; applyBrakeMode() }

        // --- servo direction toggle ---
        if (gamepad1.x && !lastX) { servoReversed = !servoReversed; applyServoDirection() }

        // --- hood window: dpad moves the max, left stick X moves the min ---
        if (gamepad1.dpad_right && !lastDpadRight) hoodMax += WINDOW_STEP
        if (gamepad1.dpad_left && !lastDpadLeft) hoodMax -= WINDOW_STEP
        hoodMin += gamepad1.left_stick_x * MIN_RATE
        // Sanity: keep 0 <= min, a minimum gap below max, and max within travel.
        hoodMax = Range.clip(hoodMax, MIN_GAP, 1.0)
        hoodMin = Range.clip(hoodMin, 0.0, hoodMax - MIN_GAP)

        // --- sweep toggle; moving the trigger hands control back to the trigger ---
        if (gamepad1.y && !lastY) { sweeping = !sweeping; if (sweeping) sweepTimer.reset() }
        val trigger = gamepad1.right_trigger
        if (abs(trigger - lastTrigger) > TRIGGER_MOVE_THRESHOLD) sweeping = false

        val hoodTarget = if (sweeping) {
            val phase = (sweepTimer.seconds() % SWEEP_SECONDS) / SWEEP_SECONDS
            val wave = (sin(phase * 2.0 * Math.PI) + 1.0) / 2.0 // 0..1
            hoodMin + wave * (hoodMax - hoodMin)
        } else {
            hoodMin + trigger * (hoodMax - hoodMin)
        }

        // --- write hardware ---
        left.power = sharedPower
        right.power = sharedPower
        hood.position = hoodTarget

        // --- telemetry ---
        telemetry.addData("shared power", "%.2f", sharedPower)
        telemetry.addData("left  dir / vel(tps)", "%s / %.0f", dirLabel(leftForward), left.velocity)
        telemetry.addData("right dir / vel(tps)", "%s / %.0f", dirLabel(rightForward), right.velocity)
        telemetry.addData("avg |vel| (tps)", "%.0f", (abs(left.velocity) + abs(right.velocity)) / 2.0)
        telemetry.addData("zero-power", if (brakeMode) "BRAKE" else "FLOAT")
        telemetry.addLine()
        telemetry.addData("hood window [min, max]", "[%.3f, %.3f]", hoodMin, hoodMax)
        telemetry.addData("hood target", "%.3f", hoodTarget)
        telemetry.addData("hood actual", "%.3f", hood.position)
        telemetry.addData("servo dir", if (servoReversed) "REVERSE" else "FORWARD")
        telemetry.addData("trigger", "%.2f", trigger)
        telemetry.addData("sweeping (Y)", sweeping)
        telemetry.addLine()
        telemetry.addLine("dpad U/D: power | A: stop | LB/RB: rev L/R motor | B: brake/float")
        telemetry.addLine("RT: hood | dpad L/R: window max | L-stick X: window min | X: rev servo | Y: sweep")

        lastUp = gamepad1.dpad_up
        lastDown = gamepad1.dpad_down
        lastDpadLeft = gamepad1.dpad_left
        lastDpadRight = gamepad1.dpad_right
        lastLb = gamepad1.left_bumper
        lastRb = gamepad1.right_bumper
        lastA = gamepad1.a
        lastB = gamepad1.b
        lastX = gamepad1.x
        lastY = gamepad1.y
        lastTrigger = trigger
    }

    override fun stop() {
        left.power = 0.0
        right.power = 0.0
    }

    private fun applyMotorDirections() {
        left.direction = if (leftForward) Direction.FORWARD else Direction.REVERSE
        right.direction = if (rightForward) Direction.FORWARD else Direction.REVERSE
    }

    private fun applyBrakeMode() {
        val behavior = if (brakeMode) ZeroPowerBehavior.BRAKE else ZeroPowerBehavior.FLOAT
        left.zeroPowerBehavior = behavior
        right.zeroPowerBehavior = behavior
    }

    private fun applyServoDirection() {
        hood.direction = if (servoReversed) Servo.Direction.REVERSE else Servo.Direction.FORWARD
    }

    private fun dirLabel(forward: Boolean) = if (forward) "FWD" else "REV"

    companion object {
        private const val POWER_STEP = 0.05
        private const val WINDOW_STEP = 0.02
        private const val MIN_RATE = 0.005 // window-min travel per loop at full left-stick deflection
        private const val MIN_GAP = 0.02   // smallest allowed window width
        private const val SWEEP_SECONDS = 3.0
        private const val TRIGGER_MOVE_THRESHOLD = 0.02
    }
}

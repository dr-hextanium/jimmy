package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.hardware.Names
import kotlin.math.sin

/**
 * Multi-servo endpoint finder for tuning the hood and gate positions (e.g. HOOD_HIGH/HOOD_LOW,
 * GATE_OPEN_POSITION/GATE_CLOSED_POSITION). Select a servo, move it, capture the endpoints, and
 * optionally sweep between them. Direct hardwareMap access -- no subsystems, no drivetrain.
 *
 * Controls (gamepad1):
 *  - left / right bumper : select previous / next servo
 *  - left stick Y        : move servo (coarse)
 *  - dpad up / down      : fine step (+/- 0.005)
 *  - X / Y               : capture current position as sweep MIN / MAX
 *  - A                   : toggle sinusoidal sweep between MIN and MAX
 */
@TeleOp(name = "Servo Debugger", group = "Debug")
class ServoDebugger : OpMode() {
    private val servoNames = listOf(Names.Servos.Launcher.servo, Names.Servos.Intake.servo)
    private val servos = HashMap<String, Servo>()
    private var index = 0

    private var target = 0.5
    private var sweepMin = 0.0
    private var sweepMax = 1.0
    private var sweeping = false
    private val sweepTimer = ElapsedTime()

    private var lastLb = false
    private var lastRb = false
    private var lastA = false
    private var lastX = false
    private var lastY = false

    private val currentName get() = servoNames[index]
    private val servo: Servo? get() = servos[currentName]

    override fun init() {
        for (name in servoNames) {
            try {
                servos[name] = hardwareMap.get(Servo::class.java, name)
            } catch (_: Exception) {
                // Servo not in the config; it just won't be selectable.
            }
        }
        if (servos.isEmpty()) telemetry.addLine("WARNING: none of $servoNames found in the config")
    }

    override fun loop() {
        // --- select servo ---
        if (gamepad1.left_bumper && !lastLb) index = (index - 1 + servoNames.size) % servoNames.size
        if (gamepad1.right_bumper && !lastRb) index = (index + 1) % servoNames.size
        lastLb = gamepad1.left_bumper
        lastRb = gamepad1.right_bumper

        // --- position control ---
        if (gamepad1.left_stick_y != 0f) {
            sweeping = false
            target -= gamepad1.left_stick_y * 0.01
        }
        when {
            gamepad1.dpad_up -> target += 0.005
            gamepad1.dpad_down -> target -= 0.005
        }

        // --- capture endpoints ---
        if (gamepad1.x && !lastX) sweepMin = target
        if (gamepad1.y && !lastY) sweepMax = target
        lastX = gamepad1.x
        lastY = gamepad1.y

        // --- sweep toggle ---
        if (gamepad1.a && !lastA) {
            sweeping = !sweeping
            if (sweeping) sweepTimer.reset()
        }
        lastA = gamepad1.a

        if (sweeping) {
            val phase = (sweepTimer.seconds() % SWEEP_SECONDS) / SWEEP_SECONDS
            val wave = (sin(phase * 2.0 * Math.PI) + 1.0) / 2.0 // 0..1
            target = sweepMin + wave * (sweepMax - sweepMin)
        }

        target = Range.clip(target, 0.0, 1.0)
        servo?.position = target

        telemetry.addData("selected servo", currentName)
        telemetry.addData("target position", "%.3f", target)
        telemetry.addData("actual position", servo?.let { String.format("%.3f", it.position) } ?: "n/a")
        telemetry.addData("sweep min (X)", "%.3f", sweepMin)
        telemetry.addData("sweep max (Y)", "%.3f", sweepMax)
        telemetry.addData("sweeping (A)", sweeping)
        telemetry.addLine("bumpers: select | stick/dpad: move | X/Y: set min/max | A: sweep")
    }

    companion object {
        private const val SWEEP_SECONDS = 3.0
    }
}

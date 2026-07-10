package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import kotlin.math.sin

/**
 * A comprehensive TeleOp for debugging and tuning a single servo.
 *
 * HOW TO USE:
 * 1. Change the SERVO_NAME constant to the name of your servo in the hardware configuration.
 * 2. Run this OpMode.
 * 3. Use the gamepad controls listed below to test the servo.
 *
 * CONTROLS (GAMEPAD 1):
 * - D-Pad Up/Down:    Fine-tune the servo position by a small increment.
 * - Left Stick Y:     Manually move the servo to any position (0.0 to 1.0).
 * - Right Stick Y:    Adjust the sweep range (min/max). Hold LEFT_BUMPER for min, RIGHT_BUMPER for max.
 *
 * - A Button:         Toggle continuous sweeping mode.
 * - B Button:         Reverse the servo's direction.
 *
 * - X Button:         Go to the sweep range's minimum position.
 * - Y Button:         Go to the sweep range's maximum position.
 *
 * - Start Button:     Disable the servo (sends it to a neutral state).
 * - Back Button:      Re-enable the servo after disabling.
 */
@TeleOp(name = "Servo Debugger", group = "Debug")
class ServoDebugger : OpMode() {

    // --- CONFIGURATION ---
    // Change this to the name of the servo you want to debug.
    private val SERVO_NAME = "hood" // EXAMPLE: "intakeGate", "launcherHood"

    // --- CONSTANTS ---
    private val FINE_TUNE_INCREMENT = 0.001
    private val SWEEP_TIME_SECONDS = 3.0 // Time for one full sweep (min to max)

    // --- STATE VARIABLES ---
    private lateinit var servo: Servo
    private var targetPosition = 0.5
    private var isReversed = false

    // Sweep functionality
    private var isSweeping = false
    private var sweepMin = 0.0
    private var sweepMax = 1.0
    private val sweepTimer = ElapsedTime()

    // Gamepad button state management to detect single presses
    private var lastA = false
    private var lastB = false
    private var lastStart = false
    private var lastBack = false

    override fun init() {
        try {
            servo = hardwareMap.get(Servo::class.java, SERVO_NAME)
            servo.direction = Servo.Direction.FORWARD
            telemetry.addData("Status", "Success! Servo '$SERVO_NAME' found.")
            telemetry.addData("Info", "Press Start to begin debugging.")
        } catch (e: Exception) {
            telemetry.addData("Error", "Could not find servo named '$SERVO_NAME'")
            telemetry.addData("Info", "Please check the SERVO_NAME variable in ServoDebugger.kt")
            telemetry.update()
            requestOpModeStop() // Stop if the servo isn't found
        }
    }

    override fun loop() {
        // --- READ INPUTS ---
        handleGamepadInput()

        // --- UPDATE LOGIC ---
        if (isSweeping) {
            // Calculate a sinusoidal wave for smooth sweeping
            val sweepRange = sweepMax - sweepMin
            val sweepPhase = (sweepTimer.seconds() % SWEEP_TIME_SECONDS) / SWEEP_TIME_SECONDS
            val wave = (sin(sweepPhase * 2.0 * Math.PI) + 1.0) / 2.0 // Produces a 0.0-1.0 wave
            targetPosition = sweepMin + (wave * sweepRange)
        }

        // --- WRITE OUTPUTS ---
        // Apply the direction reversal if needed
        val finalPosition = if (isReversed) 1.0 - targetPosition else targetPosition
        servo.position = finalPosition

        // --- TELEMETRY ---
        displayTelemetry()
    }

    private fun handleGamepadInput() {
        // --- Position Control ---
        // Fine-tuning with D-Pad
        when {
            gamepad1.dpad_up -> targetPosition += FINE_TUNE_INCREMENT
            gamepad1.dpad_down -> targetPosition -= FINE_TUNE_INCREMENT
        }
        // Manual control with Left Stick
        if (gamepad1.left_stick_y != 0f) {
            isSweeping = false // Stop sweeping if manually controlled
            targetPosition -= gamepad1.left_stick_y * 0.01 // Scale for finer control
        }
        targetPosition = Range.clip(targetPosition, 0.0, 1.0)

        // --- Go to Range Ends ---
        if (gamepad1.x) targetPosition = sweepMin
        if (gamepad1.y) targetPosition = sweepMax

        // --- Sweep Range Adjustment ---
        when {
            gamepad1.left_bumper -> sweepMin -= gamepad1.right_stick_y * 0.01
            gamepad1.right_bumper -> sweepMax -= gamepad1.right_stick_y * 0.01
        }
        sweepMin = Range.clip(sweepMin, 0.0, 1.0)
        sweepMax = Range.clip(sweepMax, 0.0, 1.0)

        // --- Mode Toggles ---
        // Toggle sweeping
        if (gamepad1.a && !lastA) {
            isSweeping = !isSweeping
            if (isSweeping) sweepTimer.reset()
        }
        lastA = gamepad1.a

        // Toggle direction
        if (gamepad1.b && !lastB) {
            isReversed = !isReversed
            servo.direction = if (isReversed) Servo.Direction.REVERSE else Servo.Direction.FORWARD
        }
        lastB = gamepad1.b

        // Enable/Disable Servo
        if (gamepad1.start && !lastStart) servo.controller.pwmDisable()
        if (gamepad1.back && !lastBack) servo.controller.pwmEnable()
        lastStart = gamepad1.start
        lastBack = gamepad1.back
    }

    private fun displayTelemetry() {
        telemetry.addLine("--- Servo Debugger for: '$SERVO_NAME' ---")
        telemetry.addData("Target Position", "%.3f", targetPosition)
        telemetry.addData("Actual HW Position", "%.3f", servo.position)
        telemetry.addData("Direction", if (isReversed) "REVERSED" else "FORWARD")
        telemetry.addLine()
        telemetry.addLine("--- Sweep Controls ---")
        telemetry.addData("Sweeping Active (A)", isSweeping)
        telemetry.addData("Sweep Min (L Bumper + R Stick)", "%.3f", sweepMin)
        telemetry.addData("Sweep Max (R Bumper + R Stick)", "%.3f", sweepMax)
        telemetry.addLine()
        telemetry.addLine("--- Controls ---")
        telemetry.addLine("D-Pad Up/Down: Fine Tune")
        telemetry.addLine("Left Stick Y: Manual Control")
        telemetry.addLine("X/Y: Go to Min/Max")
        telemetry.addLine("B: Reverse Direction")
        telemetry.addLine("Start/Back: Disable/Enable PWM")
        telemetry.update()
    }
}

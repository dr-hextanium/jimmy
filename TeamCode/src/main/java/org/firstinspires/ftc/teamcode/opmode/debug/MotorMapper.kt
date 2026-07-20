package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.hardware.Names

/*
* br -> turret
* fr -> sl
* turret -> br
* bl -> bl
* intake -> intake
* sr -> fr
* fl -> fl
* sl -> sr
* */

/**
 * Motor identification / bench tool for when motors have been plugged into the wrong ports and the
 * robot-config name -> physical-motor mapping no longer matches reality.
 *
 * It does NOT trust [Names]. It live-enumerates every DcMotor actually present in the active config
 * (`hardwareMap.getAll`), so config names that exist on the hub but are missing from [Names] (auto-scan
 * names, typos) still show up -- those are the usual culprits after a re-plug. Each enumerated motor is
 * then annotated against [Names] as its known role or `UNMAPPED`, and its port + hub are shown so you
 * can reconcile the Driver-Hub config.
 *
 * Select a motor, hold a trigger to spin it at an adjustable power, and watch which physical motor
 * moves. Direction is pure sign-of-power (right trigger = +, left trigger = -); the SDK direction is
 * left FORWARD so the encoder-delta sign you read is the raw wiring truth. Only the selected motor is
 * ever powered, and it is dead-man: every other motor is forced to zero every loop and power is applied
 * only while a trigger is held.
 *
 * Controls (gamepad1):
 *  - left / right bumper : select previous / next motor
 *  - dpad up / down      : commanded power +/- 0.05 (clamped 0..0.6)
 *  - right trigger       : spin selected motor forward  (hold; dead-man)
 *  - left trigger        : spin selected motor backward (hold; dead-man)
 *  - B (circle)          : toggle BRAKE / FLOAT zero-power behavior (all motors)
 *  - A (cross)           : re-zero the tick-delta baseline for the selected motor
 */
@TeleOp(name = "Motor Mapper", group = "Debug")
class MotorMapper : OpMode() {
    private class Entry(val motor: DcMotorEx, val names: List<String>) {
        val primary: String get() = names.firstOrNull() ?: "??"
    }

    private lateinit var entries: List<Entry>
    private var index = 0
    private var commandedPower = STARTING_POWER
    private var baseline = 0
    private var brakeMode = true

    private var lastLb = false
    private var lastRb = false
    private var lastUp = false
    private var lastDown = false
    private var lastB = false
    private var lastA = false

    override fun init() {
        // Live-enumerate, deduped by device identity (one physical port can carry several alias names).
        val seen = ArrayList<DcMotorEx>()
        for (m in hardwareMap.getAll(DcMotorEx::class.java)) if (seen.none { it === m }) seen.add(m)
        entries = seen.map { Entry(it, hardwareMap.getNamesOf(it).toList()) }

        for (e in entries) {
            e.motor.mode = RunMode.RUN_WITHOUT_ENCODER // raw open-loop duty; leaves the encoder counting
            e.motor.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
            e.motor.power = 0.0
        }

        if (entries.isEmpty()) {
            telemetry.addLine("WARNING: no DcMotor devices in the active config")
        } else {
            captureBaseline()
            telemetry.addLine("Hold RT/LT to spin the selected motor; watch which one moves.")
        }
    }

    override fun loop() {
        if (entries.isEmpty()) {
            telemetry.addLine("no DcMotor devices found in the config -- check the Robot Configuration")
            return
        }

        // --- select motor (rising-edge; re-zero the tick baseline on change) ---
        if (gamepad1.left_bumper && !lastLb) { index = (index - 1 + entries.size) % entries.size; captureBaseline() }
        if (gamepad1.right_bumper && !lastRb) { index = (index + 1) % entries.size; captureBaseline() }

        // --- commanded power (rising-edge stepping so a held dpad doesn't ramp away) ---
        if (gamepad1.dpad_up && !lastUp) commandedPower += POWER_STEP
        if (gamepad1.dpad_down && !lastDown) commandedPower -= POWER_STEP
        commandedPower = Range.clip(commandedPower, 0.0, MAX_POWER)

        // --- brake / float toggle (applies to every motor) ---
        if (gamepad1.b && !lastB) {
            brakeMode = !brakeMode
            val z = if (brakeMode) ZeroPowerBehavior.BRAKE else ZeroPowerBehavior.FLOAT
            entries.forEach { it.motor.zeroPowerBehavior = z }
        }

        // --- re-zero tick baseline ---
        if (gamepad1.a && !lastA) captureBaseline()

        // --- dead-man spin: sign from which trigger is held; both/none -> stop ---
        val forward = gamepad1.right_trigger > TRIGGER_THRESHOLD
        val reverse = gamepad1.left_trigger > TRIGGER_THRESHOLD
        val signed = when {
            forward && !reverse -> commandedPower
            reverse && !forward -> -commandedPower
            else -> 0.0
        }

        // Zero everything, then power only the selected motor -- so cycling selection while holding a
        // trigger always drops the previously-selected motor to zero.
        for (e in entries) e.motor.power = 0.0
        val sel = entries[index]
        sel.motor.power = signed

        // --- telemetry ---
        telemetry.addData("motor", "%d/%d   %s", index + 1, entries.size, sel.primary)
        if (sel.names.size > 1) telemetry.addData("  aliases", sel.names.joinToString(", "))
        telemetry.addData("  role", roleLabel(sel))
        telemetry.addData("  port / hub", "%d @ %s", sel.motor.portNumber, sel.motor.controller.connectionInfo)
        if (isLimitedTravel(sel)) telemetry.addLine("  ** LIMITED TRAVEL: tap only, do NOT hold **")
        telemetry.addLine()
        telemetry.addData("commanded power", "%.2f  (max %.2f)", commandedPower, MAX_POWER)
        telemetry.addData("state", when {
            signed > 0 -> "RUNNING +%.2f".format(signed)
            signed < 0 -> "RUNNING %.2f".format(signed)
            else -> "idle"
        })
        telemetry.addData("tick delta", sel.motor.currentPosition - baseline)
        telemetry.addData("velocity (tps)", "%.0f", sel.motor.velocity)
        telemetry.addData("zero-power", if (brakeMode) "BRAKE" else "FLOAT")
        telemetry.addLine("if it spins but delta/vel stay 0 -> encoder not wired to this port")
        telemetry.addLine()
        telemetry.addLine("-- all motors (only the selected one is powered) --")
        for ((i, e) in entries.withIndex()) {
            val mark = if (i == index) ">" else " "
            telemetry.addData("$mark ${e.primary}", "%.0f tps", e.motor.velocity)
        }
        telemetry.addLine()
        telemetry.addLine("LB/RB: select | dpad U/D: power | RT/LT: spin +/- (hold) | B: brake/float | A: zero ticks")

        lastLb = gamepad1.left_bumper
        lastRb = gamepad1.right_bumper
        lastUp = gamepad1.dpad_up
        lastDown = gamepad1.dpad_down
        lastB = gamepad1.b
        lastA = gamepad1.a
    }

    override fun stop() {
        if (::entries.isInitialized) entries.forEach { it.motor.power = 0.0 }
    }

    private fun captureBaseline() {
        baseline = entries[index].motor.currentPosition
    }

    private fun roleLabel(e: Entry): String {
        for (n in e.names) ROLES[n]?.let { return it.label }
        return "UNMAPPED (not in Names.kt)"
    }

    private fun isLimitedTravel(e: Entry): Boolean = e.names.any { ROLES[it]?.limitedTravel == true }

    private data class Role(val label: String, val limitedTravel: Boolean)

    companion object {
        private const val POWER_STEP = 0.05
        private const val MAX_POWER = 0.6
        private const val STARTING_POWER = 0.20
        private const val TRIGGER_THRESHOLD = 0.3f

        // Reverse map: config-name string -> its known role in Names.kt. Only references Names constants,
        // so it doesn't hardcode any device string. Anything not here shows up as UNMAPPED.
        private val ROLES: Map<String, Role> = mapOf(
            Names.Motors.Intake.motor to Role("Intake.motor", limitedTravel = false),
            Names.Motors.Turret.motor to Role("Turret.motor", limitedTravel = true), // hard stops
            Names.Motors.Launcher.leftMotor to Role("Launcher.leftMotor", limitedTravel = false),
            Names.Motors.Launcher.rightMotor to Role("Launcher.rightMotor", limitedTravel = false),
            Names.Motors.Drivetrain.frontRight to Role("Drivetrain.frontRight", limitedTravel = false),
            Names.Motors.Drivetrain.frontLeft to Role("Drivetrain.frontLeft", limitedTravel = false),
            Names.Motors.Drivetrain.backRight to Role("Drivetrain.backRight", limitedTravel = false),
            Names.Motors.Drivetrain.backLeft to Role("Drivetrain.backLeft", limitedTravel = false),
        )
    }
}

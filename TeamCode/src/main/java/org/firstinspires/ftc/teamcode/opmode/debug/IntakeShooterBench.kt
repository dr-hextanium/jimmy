package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.hardware.Globals
import org.firstinspires.ftc.teamcode.hardware.Names
import org.firstinspires.ftc.teamcode.hardware.subsystem.Intake
import org.firstinspires.ftc.teamcode.hardware.subsystem.Launcher

/**
 * Combined intake + shooter bench: run the full "stage a ball, spin up, feed it into the flywheel"
 * flow WITHOUT the turret, drivetrain, or odometry. Unlike the real teleop ([DriverControlled]), this
 * skips [org.firstinspires.ftc.teamcode.hardware.Robot].init(), so it needs only the five intake/
 * shooter devices in the config (`intake`, `gate`, `sl`, `sr`, `hood`) and it never energizes the
 * turret -- the safe way to test shooting on a robot whose turret isn't calibrated yet.
 *
 * It drives the *real* [Intake] and [Launcher] subsystems (not raw hardware), so the launcher runs the
 * same feedforward-first velocity loop with the gains you pasted from "Launcher Auto-Tune". Hubs are
 * bulk-cached MANUAL and cleared once per loop, matching Robot's real loop so the velocity loop reads
 * fresh TPS.
 *
 * Shooting recipe:
 *   1. dpad up/down to pick the flywheel speed, RIGHT BUMPER to spin up, wait for "AT SPEED" (rumble).
 *   2. LEFT TRIGGER to intake -- with the gate closed the ball stages just below the flywheel.
 *   3. RIGHT TRIGGER to feed (opens the gate + runs the intake) -- pushes the staged ball into the
 *      spinning flywheel = shot.
 *
 * Controls (gamepad1):
 *  - right bumper       : toggle flywheel ON / OFF (holds the current target speed)
 *  - dpad up / down     : flywheel target scalar +/- 0.05 (fraction of MAX_TPS)
 *  - dpad right / left  : hood scalar +/- 0.05 (0 = HOOD_HIGH rest, 1 = HOOD_LOW)
 *  - left trigger       : intake IN (stage; gate stays closed)
 *  - right trigger      : FEED / FIRE (open gate + intake in)
 *  - A                  : eject (intake reverse) while held
 *  - B                  : STOP ALL (flywheel off, intake off, gate closed)
 */
@TeleOp(name = "Intake + Shooter Bench", group = "Debug")
class IntakeShooterBench : OpMode() {
    private val hubs by lazy { hardwareMap.getAll(LynxModule::class.java) }

    private lateinit var intake: Intake
    private lateinit var launcher: Launcher

    private var flywheelOn = false
    private var flywheelScalar = 0.0
    private var hoodScalar = 0.0 // 0 -> HOOD_HIGH, the position the launcher parks at on reset()

    private var lastReady = false
    private var lastRb = false
    private var lastUp = false
    private var lastDown = false
    private var lastLeft = false
    private var lastRight = false

    override fun init() {
        // This OpMode does NOT call Robot.init(), so Robot.telemetry is never assigned. Launcher.update()
        // only touches it when DEBUG_TELEMETRY is on -- force it off so a dashboard toggle can't make the
        // launcher dereference the uninitialized Robot.telemetry. We render our own telemetry below.
        Globals.DEBUG_TELEMETRY = false

        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        val intakeMotor = hardwareMap.get(DcMotorEx::class.java, Names.Motors.Intake.motor)
        val gate = hardwareMap.get(Servo::class.java, Names.Servos.Intake.servo)
        val left = hardwareMap.get(DcMotorEx::class.java, Names.Motors.Launcher.leftMotor)
        val right = hardwareMap.get(DcMotorEx::class.java, Names.Motors.Launcher.rightMotor)
        val hood = hardwareMap.get(Servo::class.java, Names.Servos.Launcher.servo)

        intake = Intake(intakeMotor, gate)
        launcher = Launcher(left, right, hood)
        intake.reset()
        launcher.reset()

        telemetry.addLine("Intake + Shooter bench ready. Flywheel WILL spin -- keep clear.")
        telemetry.addLine("Recipe: set speed -> RB spin up -> LT stage -> RT fire.")
    }

    override fun loop() {
        hubs.forEach { it.clearBulkCache() }

        intake.read()
        launcher.read()

        // --- inputs (rising-edge for toggles/steps so a held button doesn't ramp away) ---
        if (gamepad1.right_bumper && !lastRb) flywheelOn = !flywheelOn
        if (gamepad1.dpad_up && !lastUp) flywheelScalar += SCALAR_STEP
        if (gamepad1.dpad_down && !lastDown) flywheelScalar -= SCALAR_STEP
        if (gamepad1.dpad_right && !lastRight) hoodScalar += SCALAR_STEP
        if (gamepad1.dpad_left && !lastLeft) hoodScalar -= SCALAR_STEP

        val stopAll = gamepad1.b
        if (stopAll) flywheelOn = false

        flywheelScalar = Range.clip(flywheelScalar, 0.0, 1.0)
        hoodScalar = Range.clip(hoodScalar, 0.0, 1.0)

        // --- intake / feed ---
        val feeding = gamepad1.right_trigger > 0.5f
        val intaking = gamepad1.left_trigger > 0.3f
        intake.power = when {
            stopAll -> 0.0
            feeding || intaking -> Intake.POWER_INTAKE
            gamepad1.a -> Intake.POWER_REVERSE
            else -> 0.0
        }
        if (feeding && !stopAll) intake.openGate() else intake.closeGate()

        // --- launcher targets (set before update(), same as a command would) ---
        launcher.targetTPSByScalar(if (flywheelOn) flywheelScalar else 0.0)
        launcher.targetHoodByScalar(hoodScalar)

        // --- compute + write ---
        intake.update()
        launcher.update()
        intake.write()
        launcher.write()

        // Buzz once when the wheel first reaches speed, so you can watch the ball instead of the screen.
        val ready = launcher.isReady
        if (ready && !lastReady) Launcher.Effects.signalAtSpeed(gamepad1)
        lastReady = ready

        // --- telemetry ---
        telemetry.addLine(if (flywheelOn) "FLYWHEEL: ON" else "FLYWHEEL: OFF")
        telemetry.addData("flywheel scalar", "%.2f", flywheelScalar)
        telemetry.addData("target / measured TPS", "%.0f / %.0f", launcher.targetTPS, launcher.averageTPS)
        telemetry.addData("at speed?", if (ready) "YES" else "no")
        telemetry.addData("output power / cap", "%.2f / %.2f", launcher.currentPower, launcher.powerCap)
        telemetry.addLine()
        telemetry.addData("hood scalar / actual", "%.2f / %.3f", hoodScalar, launcher.hood.position)
        telemetry.addLine()
        telemetry.addData("intake power", "%.2f", intake.power)
        telemetry.addData("gate", if (intake.gateOpened) "OPEN" else "CLOSED")
        telemetry.addLine()
        telemetry.addLine("RB: flywheel on/off | dpad U/D: speed | dpad L/R: hood")
        telemetry.addLine("LT: intake  RT: FEED/FIRE  A: eject  B: STOP ALL")

        lastRb = gamepad1.right_bumper
        lastUp = gamepad1.dpad_up
        lastDown = gamepad1.dpad_down
        lastLeft = gamepad1.dpad_left
        lastRight = gamepad1.dpad_right
    }

    override fun stop() {
        // Hard stop straight to hardware, independent of subsystem state.
        intake.motor.power = 0.0
        launcher.motors.forEach { it.power = 0.0 }
    }

    companion object {
        private const val SCALAR_STEP = 0.05
    }
}

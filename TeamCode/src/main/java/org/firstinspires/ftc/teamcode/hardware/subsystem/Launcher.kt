package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.control.CurrentLimiter
import org.firstinspires.ftc.teamcode.control.ShooterModel
import org.firstinspires.ftc.teamcode.control.TimeSource
import org.firstinspires.ftc.teamcode.hardware.Globals
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.utility.absPercentDifference
import org.firstinspires.ftc.teamcode.utility.map
import org.firstinspires.ftc.teamcode.utility.percentDifference
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.min

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
 * Optional current-limited spin-up ([SPINUP_CURRENT_LIMIT_A], default 0 = OFF): a [CurrentLimiter]
 * caps the commanded power so the per-motor current stays near a budget, trading a little spin-up
 * time for a bounded peak draw. It is applied as `min(feedforward+P, currentCap)`, so it only bites
 * during the high-torque spin-up (where the FF+P term saturates anyway) and hands cleanly back to the
 * velocity loop at speed -- covering both cold start and post-shot recovery. OFF by default so the
 * flywheel behaves exactly as the plain loop until a limit is set from "Launcher Auto-Tune"; a bad
 * current reading also disables it (see [CurrentLimiter]). Gains/limit are on-robot tunables.
 *
 * There are two ways to set the target:
 *  - [targetTPSByScalar] / [targetHoodByScalar]: the manual, field-tested 0..1 path (driver trims,
 *    autos' hand-tuned constants).
 *  - [aimAtDistance]: the kinematic path -- [ShooterModel] turns a field distance into the flywheel
 *    speed and hood angle that physically reach the goal.
 */
class Launcher(
    val left: DcMotorEx,
    val right: DcMotorEx,
    val hood: Servo,
    private val timeSource: TimeSource = TimeSource.SYSTEM,
) : ISubsystem {
    val motors = listOf(left, right)

    var currentPower = 0.0

    var targetTPS = 0.0
    var targetHoodPosition = 0.0

    // Flywheel speed, sampled once per loop in read(). Both motor velocities come from the same
    // bulk-cache epoch as any live read in update() would, so the value is identical -- but sampling it
    // once here (its correct lifecycle phase) avoids recomputing abs/add/div on every consumer.
    private var measuredTPS: Double = 0.0
    val averageTPS: Double
        get() = measuredTPS

    // Per-motor currents latched in read() (amps); filtered + fed to the limiter in update() (needs dt).
    private var rawCurrentLeft: Double = 0.0
    private var rawCurrentRight: Double = 0.0
    private var filteredCurrent: Double = 0.0
    private var currentInitialized: Boolean = false

    // Current-limited spin-up governor. Inert (cap 1.0) while SPINUP_CURRENT_LIMIT_A <= 0.
    private val currentLimiter = CurrentLimiter()
    // Power cap the limiter produced this loop; 1.0 means "not limiting". Public-read for telemetry.
    var powerCap: Double = 1.0
        private set

    private var lastTime: Double = 0.0
    private var loopInitialized: Boolean = false

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
        measuredTPS = 0.0

        rawCurrentLeft = 0.0
        rawCurrentRight = 0.0
        filteredCurrent = 0.0
        currentInitialized = false
        currentLimiter.reset()
        powerCap = 1.0
        lastTime = timeSource.seconds()
        loopInitialized = false
        // Seed the hood target to match the position reset() parks the servo at (below), otherwise
        // the first write() drives the hood to targetHoodPosition's 0.0 default -- past HOOD_HIGH,
        // the low end of the usable travel -- until the driver first moves it in TeleOp.
        targetHoodPosition = HOOD_HIGH

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

    override fun read() {
        measuredTPS = (abs(left.velocity) + abs(right.velocity)) / 2.0
        // Per-motor current for the spin-up limiter (magnitude; the REVERSE motor may report signed).
        rawCurrentLeft = left.getCurrent(CurrentUnit.AMPS)
        rawCurrentRight = right.getCurrent(CurrentUnit.AMPS)
    }

    override fun update() {
        // Loop dt (the limiter's recovery is rate-based, so it must be loop-rate invariant). Collapse
        // the first tick after a reset and clamp long gaps -- mirrors the Turret's dt handling.
        val now = timeSource.seconds()
        if (!loopInitialized) lastTime = now
        var dt = now - lastTime
        lastTime = now
        if (dt < 0.0 || dt > MAX_DT) dt = 0.0
        loopInitialized = true

        // Light low-pass on the worst-case (per-motor) current before it drives the limiter.
        val rawMaxCurrent = max(abs(rawCurrentLeft), abs(rawCurrentRight))
        if (!currentInitialized) {
            filteredCurrent = rawMaxCurrent
            currentInitialized = true
        } else if (dt > 0.0) {
            filteredCurrent += (1.0 - exp(-dt / CURRENT_FILTER_TAU)) * (rawMaxCurrent - filteredCurrent)
        }

        // Update the governor every loop (so its cap recovers to 1.0 while idle/at speed). When
        // SPINUP_CURRENT_LIMIT_A <= 0 the limiter is inert and powerCap stays 1.0 (== old behavior).
        currentLimiter.limitAmps = SPINUP_CURRENT_LIMIT_A
        currentLimiter.recoveryPerSecond = SPINUP_RECOVERY_PER_SEC
        powerCap = currentLimiter.update(filteredCurrent, dt)

        // Feedforward-first velocity control. The flywheel is single-direction, so power is clamped
        // to [0, 1]: it can spin the wheel up but never actively brakes it. The current cap only ever
        // lowers the command (min), so with limiting off (cap 1.0) this is the plain FF+P loop.
        currentPower = if (targetTPS < MIN_TPS) {
            0.0
        } else {
            val feedforwardPlusP = kS + kV * targetTPS + kP * (targetTPS - averageTPS)
            min(feedforwardPlusP, powerCap).coerceIn(0.0, 1.0)
        }

        if (Globals.DEBUG_TELEMETRY) {
            Robot.telemetry.addData("Launcher Target TPS", targetTPS)
            Robot.telemetry.addData("Launcher Measured TPS", averageTPS)
            Robot.telemetry.addData("Launcher output power", currentPower)
            Robot.telemetry.addData("Launcher at speed?", isReady)
            Robot.telemetry.addData("Launcher current (A)", filteredCurrent)
            Robot.telemetry.addData("Launcher power cap", powerCap)
        }
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

        // Current-limited spin-up (on-robot tunables, set from "Launcher Auto-Tune").
        // SPINUP_CURRENT_LIMIT_A <= 0 DISABLES limiting -> plain feedforward+P (the shipped default).
        // Per motor, amps. Must sit ABOVE the steady-state current needed to hold the target speed,
        // or the cap will throttle at speed and the wheel never reaches/holds it (the tuner's knee
        // recommendation respects this floor).
        var SPINUP_CURRENT_LIMIT_A = 0.0
        // How fast the power cap re-opens as back-EMF drops the current during spin-up (1/s). Faster
        // = closer to time-optimal but hunts more around the limit. Shared with the tuner's Phase B.
        var SPINUP_RECOVERY_PER_SEC = 6.0

        private const val CURRENT_FILTER_TAU = 0.02 // s, light low-pass on the measured current
        private const val MAX_DT = 0.1              // s, longest loop gap the limiter integrates
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

package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.control.FeedforwardFit
import org.firstinspires.ftc.teamcode.control.PolePlacement
import org.firstinspires.ftc.teamcode.hardware.Names
import org.firstinspires.ftc.teamcode.hardware.subsystem.Turret
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sign

/**
 * Automatic feedforward + PID tuner for the turret -- a SysId-style characterization run that ends by
 * printing the five control gains (kStatic, kV, kA, kP, kD) ready to paste into [Turret]'s companion,
 * or push live to the FTC Dashboard with one button. It replaces hand-tuning the turret's control law.
 *
 * HOW IT WORKS (all open-loop, so it is independent of the closed-loop law and of the still-
 * uncalibrated absolute encoders -- it uses only the drive-motor's own quadrature encoder for
 * velocity and for a relative position, exactly like RPMTest/ShooterBench):
 *
 *  1. SETUP        -- jog the turret to its mechanical CENTRE by eye (triggers, dead-man, capped),
 *                     then press A. Starting centred leaves ~equal travel both ways, so the first
 *                     unknown-direction probe can lunge either way without slamming a hard stop.
 *  2. SIGN_PROBE   -- a small power step establishes which tick direction "+power" drives, so all
 *                     later maths works in a self-consistent frame regardless of motor wiring.
 *  3. QUASISTATIC  -- ping-pongs at a sweep of constant powers, recording (power, velocity) only once
 *                     settled (acceleration gated to ~0) and clear of the reversal (backlash) zone.
 *                     A least-squares line -> kV and kStatic (per direction, so asymmetry shows).
 *  4. DYNAMIC      -- from a band edge, applies from-rest power steps; the velocity rise gives the
 *                     plant time constant tau (no acceleration differentiation), and kA = tau*kV.
 *  5. REPORT       -- runs [PolePlacement] on (kV, kA) for kP/kD at a target settling time, and prints
 *                     everything with fit-quality diagnostics.
 *
 * SAFETY: the routine is HOLD-TO-RUN -- power is applied only while RIGHT BUMPER is held; release it
 * (or press B) and the motor is cut instantly and the sequence pauses. Independently of the
 * sequencer, a hard motor-tick limit ([HARD_LIMIT_DEG] from the centred start) zeroes power and
 * aborts if the turret ever leaves the safe band. Everything is capped and open-loop; nothing here
 * runs the turret's closed-loop PID.
 *
 * NOTE (kD): the identified plant is characterised on the clean motor tachometer, but the live law
 * feeds kD from the noisier FadingMemoryFilter velocity unless USE_MOTOR_FUSION is on. So the printed
 * kD is only applied on the "apply live" button when fusion is enabled; otherwise the shipped kD=0
 * default is kept (see the on-screen note).
 */
@TeleOp(name = "Turret Auto-Tune", group = "Debug")
class TurretAutoTune : OpMode() {
    private val motor by lazy { hardwareMap.get(DcMotorEx::class.java, Names.Motors.Turret.motor) }
    private val battery by lazy { hardwareMap.voltageSensor.iterator().next() as VoltageSensor }

    private enum class Phase { SETUP, SIGN_PROBE, QUASISTATIC, DYN_REPOSITION, DYN_SETTLE, DYN_STEP, REPORT, ABORTED }
    private var phase = Phase.SETUP
    private var message = ""

    private val timer = ElapsedTime()
    private var now = 0.0
    private var lastNow = 0.0
    private var phaseEnter = 0.0 // wall time the current phase began; drives per-phase timeouts

    // --- relative motion frame (see class doc): +power increases posDeg once velocitySign is set ---
    private var zeroTicks = 0
    private var velocitySign = 1.0
    private var posDeg = 0.0
    private var velDeg = 0.0
    private var lastVelDeg = 0.0
    private var minVoltage = Double.MAX_VALUE

    // Pause bookkeeping (RIGHT BUMPER released mid-run).
    private var paused = false

    // --- QUASISTATIC ping-pong ---
    private val ssSamples = ArrayList<FeedforwardFit.Sample>()
    private var qIndex = 0
    private var qDir = 1.0
    private var qTraverses = 0
    private var reversalTime = 0.0

    // --- DYNAMIC steps ---
    private data class Step(val power: Double, val startEdge: Double)
    private val dynScript = listOf(
        Step(0.40, -DYN_START_EDGE_DEG),
        Step(-0.40, DYN_START_EDGE_DEG),
        Step(0.55, -DYN_START_EDGE_DEG),
        Step(-0.55, DYN_START_EDGE_DEG),
    )
    private var dynIndex = 0
    private var settleStart = 0.0
    private var stepStart = 0.0
    private val stepSamples = ArrayList<FeedforwardFit.StepSample>()
    private val taus = ArrayList<Double>()

    // --- fitted results (populated approaching REPORT) ---
    private var ss: FeedforwardFit.SteadyStateResult? = null
    private var kA = 0.0
    private var tau = 0.0
    private var gains: PolePlacement.Gains? = null
    private var applied = false

    // rising-edge latches
    private var lastA = false
    private var lastB = false
    private var lastY = false

    override fun init() {
        motor.direction = DcMotorSimple.Direction.FORWARD
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        telemetry.addLine("TURRET AUTO-TUNE")
        telemetry.addLine("Jog to mechanical CENTRE (triggers), then press A to begin.")
        telemetry.addLine("Then HOLD RIGHT BUMPER to run. Release / press B to stop instantly.")
    }

    override fun loop() {
        now = timer.seconds()
        val dt = (now - lastNow).coerceIn(0.0, 0.2)

        // Read hardware (motor quadrature only) and derive the relative frame.
        val ticks = motor.currentPosition
        posDeg = (ticks - zeroTicks) * Turret.DEG_PER_TICK * velocitySign
        velDeg = motor.velocity * Turret.DEG_PER_TICK * velocitySign
        minVoltage = minOf(minVoltage, battery.voltage)

        val bPressed = gamepad1.b && !lastB
        lastB = gamepad1.b
        if (bPressed && phase != Phase.REPORT) abort("Stopped by operator (B).")

        // Independent hard safety limit (once we have a valid centred zero).
        if (phase != Phase.SETUP && phase != Phase.ABORTED && phase != Phase.REPORT &&
            abs(posDeg) > HARD_LIMIT_DEG
        ) {
            abort("HARD LIMIT: |%.0f| > %.0f deg. Re-centre more carefully and rerun.".format(posDeg, HARD_LIMIT_DEG))
        }

        when (phase) {
            Phase.SETUP -> runSetup()
            Phase.REPORT -> { motor.power = 0.0; runReport() }
            Phase.ABORTED -> motor.power = 0.0
            else -> runAutoPhase(dt) // the hold-to-run block gates power for every active phase
        }

        lastNow = now
        lastVelDeg = velDeg
        renderTelemetry()
    }

    override fun stop() {
        motor.power = 0.0
    }

    // ---- SETUP: manual centring ----

    private fun runSetup() {
        val jog = Range.clip((gamepad1.right_trigger - gamepad1.left_trigger).toDouble(), -1.0, 1.0) * JOG_POWER
        motor.power = jog
        if (gamepad1.a && !lastA) {
            zeroTicks = motor.currentPosition
            reversalTime = now
            go(Phase.SIGN_PROBE)
            message = "Centred. HOLD RIGHT BUMPER to run the sequence."
        }
        lastA = gamepad1.a
    }

    // ---- hold-to-run gate shared by every powered auto phase ----

    private fun runAutoPhase(dt: Double) {
        if (!gamepad1.right_bumper) {
            motor.power = 0.0
            paused = true
            message = "PAUSED - hold RIGHT BUMPER to run."
            return
        }
        if (paused) {
            // Resuming: restart any time-sensitive sub-move so a stale gap can't corrupt its timing
            // or falsely trip a phase timeout.
            paused = false
            phaseEnter = now
            reversalTime = now
            settleStart = now
            if (phase == Phase.DYN_STEP) { stepStart = now; stepSamples.clear() }
        }

        val running = phase
        val commanded = when (running) {
            Phase.SIGN_PROBE -> signProbe()
            Phase.QUASISTATIC -> quasistatic(dt)
            Phase.DYN_REPOSITION -> reposition()
            Phase.DYN_SETTLE -> settle()
            Phase.DYN_STEP -> step()
            else -> 0.0
        }
        // If a sub-phase transitioned this loop (including abort), cut power for a clean handoff.
        motor.power = if (phase == running) commanded else 0.0
    }

    // ---- SIGN_PROBE ----

    private fun signProbe(): Double {
        val rawDeltaDeg = (motor.currentPosition - zeroTicks) * Turret.DEG_PER_TICK
        if (abs(rawDeltaDeg) > SIGN_PROBE_MIN_DEG) {
            velocitySign = sign(rawDeltaDeg)
            qDir = 1.0
            qTraverses = 0
            reversalTime = now
            go(Phase.QUASISTATIC)
            message = "Sign locked (velocitySign=%.0f). Sweeping steady-state powers...".format(velocitySign)
            return 0.0
        }
        if (now - phaseEnter > SIGN_PROBE_TIMEOUT) {
            abort("Turret did not move under %.2f power in %.1fs -- check power/mechanics.".format(SIGN_PROBE_POWER, SIGN_PROBE_TIMEOUT))
            return 0.0
        }
        return SIGN_PROBE_POWER
    }

    // ---- QUASISTATIC: ping-pong, record settled mid-band samples ----

    private fun quasistatic(dt: Double): Double {
        val p = QUASISTATIC_POWERS[qIndex]
        val commanded = qDir * p

        // Reverse at the band edges.
        if (qDir > 0 && posDeg > BAND_EDGE_DEG) { qDir = -1.0; qTraverses++; reversalTime = now }
        else if (qDir < 0 && posDeg < -BAND_EDGE_DEG) { qDir = 1.0; qTraverses++; reversalTime = now }

        // Record only settled (accel ~ 0), mid-band, post-backlash samples.
        val accel = if (dt > 1e-4) abs(velDeg - lastVelDeg) / dt else 0.0
        val settled = accel < ACCEL_GATE_DEG_S2
        if (abs(posDeg) < RECORD_BAND_DEG && now - reversalTime > REVERSAL_BLANK_S &&
            abs(velDeg) > MIN_RECORD_SPEED_DEG_S && settled
        ) {
            ssSamples.add(FeedforwardFit.Sample(commanded, velDeg))
        }

        // Advance to the next power level when this one is done OR has stalled (no reversal within
        // the timeout -- e.g. a power below breakaway friction, which would otherwise hang forever
        // with no motion to trip the hard limit). A stalled low level simply yields to a higher one.
        val levelDone = qTraverses >= TRAVERSES_PER_POWER
        val levelStalled = now - reversalTime > QUASISTATIC_LEVEL_TIMEOUT
        if (levelDone || levelStalled) {
            if (levelStalled && !levelDone) message = "Power %.2f stalled (below breakaway?); skipping.".format(p)
            qIndex++
            qTraverses = 0
            reversalTime = now
            if (qIndex >= QUASISTATIC_POWERS.size) finishQuasistatic()
        }
        return commanded
    }

    private fun finishQuasistatic() {
        try {
            ss = FeedforwardFit.fitSteadyState(ssSamples)
        } catch (e: IllegalArgumentException) {
            abort("Steady-state fit failed: ${e.message}")
            return
        }
        dynIndex = 0
        go(Phase.DYN_REPOSITION)
        message = "Steady-state done (${ssSamples.size} samples). Running dynamic steps..."
    }

    // ---- DYNAMIC: reposition to an edge, let it settle, apply a from-rest step ----

    private fun reposition(): Double {
        val target = dynScript[dynIndex].startEdge
        if (now - phaseEnter > REPO_TIMEOUT) {
            abort("Reposition to %.0f deg timed out -- jam or disconnect?".format(target))
            return 0.0
        }
        val err = target - posDeg
        if (abs(err) < REPO_TOL_DEG) {
            settleStart = now
            go(Phase.DYN_SETTLE)
            return 0.0
        }
        return REPO_POWER * sign(err)
    }

    private fun settle(): Double {
        if (abs(velDeg) > SETTLE_SPEED_DEG_S) settleStart = now // still coasting; restart dwell
        if (now - phaseEnter > SETTLE_TIMEOUT) {
            // Never settled (oscillating / sensor issue) -- skip this step rather than fit from a
            // moving start, which would corrupt tau.
            message = "Step ${dynIndex + 1} skipped: turret never settled."
            advanceDynamic()
            return 0.0
        }
        if (now - settleStart > SETTLE_DWELL_S) {
            stepStart = now
            stepSamples.clear()
            go(Phase.DYN_STEP)
        }
        return 0.0
    }

    private fun step(): Double {
        val stepPower = dynScript[dynIndex].power
        stepSamples.add(FeedforwardFit.StepSample(now - stepStart, velDeg))

        val kVEst = ss!!.kV
        val vTermMag = max(1.0, (abs(stepPower) - ss!!.kStatic) / kVEst)
        val nearPlateau = abs(velDeg) > PLATEAU_FRACTION * vTermMag
        val crossedFar = (stepPower > 0 && posDeg > RECORD_BAND_DEG) ||
            (stepPower < 0 && posDeg < -RECORD_BAND_DEG)
        val timedOut = now - stepStart > MAX_STEP_TIME_S

        if (nearPlateau || crossedFar || timedOut) {
            val vTermSigned = if (stepPower > 0) vTermMag else -vTermMag
            try {
                taus.add(FeedforwardFit.fitTimeConstant(stepSamples, vTermSigned, kVEst).tau)
            } catch (e: IllegalArgumentException) {
                message = "Step ${dynIndex + 1} discarded: ${e.message}"
            }
            advanceDynamic()
            return 0.0
        }
        return stepPower
    }

    private fun advanceDynamic() {
        dynIndex++
        if (dynIndex >= dynScript.size) finishDynamic() else go(Phase.DYN_REPOSITION)
    }

    private fun finishDynamic() {
        if (taus.isEmpty()) {
            abort("No usable step responses -- could not identify kA.")
            return
        }
        tau = taus.average()
        kA = tau * ss!!.kV
        gains = PolePlacement.positionGains(ss!!.kV, kA, SETTLING_TIME_S)
        go(Phase.REPORT)
        message = "DONE. Y = apply live to dashboard. Copy the gains below into Turret."
    }

    // ---- REPORT ----

    private fun runReport() {
        if (gamepad1.y && !lastY && gains != null && !applied) {
            val s = ss!!
            val g = gains!!
            Turret.kStatic = s.kStatic
            Turret.kV = s.kV
            Turret.kA = kA
            Turret.kP = g.kP
            // kD only when the clean-velocity path (motor fusion) is active; otherwise keep 0.
            Turret.kD = if (Turret.USE_MOTOR_FUSION) g.kD else 0.0
            applied = true
        }
        lastY = gamepad1.y
    }

    // ---- helpers ----

    private fun go(next: Phase) {
        phase = next
        phaseEnter = now
    }

    private fun abort(reason: String) {
        motor.power = 0.0
        message = reason
        go(Phase.ABORTED)
    }

    private fun renderTelemetry() {
        telemetry.addData("PHASE", phase)
        telemetry.addData("note", message)
        telemetry.addData("pos (deg, rel)", "%.1f", posDeg)
        telemetry.addData("vel (deg/s)", "%.1f", velDeg)
        telemetry.addData("min battery (V)", if (minVoltage == Double.MAX_VALUE) 0.0 else minVoltage)

        when (phase) {
            Phase.SETUP -> {
                telemetry.addData("motor ticks", motor.currentPosition)
                telemetry.addLine("triggers: jog +/- (capped $JOG_POWER) | A: set centre & begin")
            }
            Phase.QUASISTATIC -> {
                telemetry.addData("sweep power", "%.2f (%d/%d)", QUASISTATIC_POWERS[qIndex], qIndex + 1, QUASISTATIC_POWERS.size)
                telemetry.addData("steady-state samples", ssSamples.size)
            }
            Phase.DYN_REPOSITION, Phase.DYN_SETTLE, Phase.DYN_STEP -> {
                telemetry.addData("dynamic step", "%d/%d (power %.2f)", dynIndex + 1, dynScript.size, dynScript[dynIndex.coerceAtMost(dynScript.size - 1)].power)
                telemetry.addData("taus so far", taus.joinToString { "%.3f".format(it) })
            }
            Phase.REPORT -> renderReport()
            else -> {}
        }

        if (phase != Phase.SETUP && phase != Phase.REPORT && phase != Phase.ABORTED) {
            telemetry.addLine("HOLD RIGHT BUMPER to run | B: stop")
            telemetry.addLine("If it grazes a hard stop, release and re-centre more carefully.")
        }
    }

    private fun renderReport() {
        val s = ss ?: return
        val g = gains ?: return
        telemetry.addLine("--- IDENTIFIED PLANT ---")
        telemetry.addData("kStatic +/-", "%.4f / %.4f (R2 %.3f / %.3f)", s.kStaticPositive, s.kStaticNegative, s.positive.r2, s.negative.r2)
        val asym = abs(s.kStaticPositive - s.kStaticNegative)
        if (asym > STICTION_ASYMMETRY_WARN) telemetry.addLine("  ! kStatic asymmetry %.4f is high -- check for a mechanical bind".format(asym))
        telemetry.addData("tau (s)", "%.4f  (from %d steps)", tau, taus.size)
        telemetry.addLine("--- PASTE INTO Turret companion ---")
        telemetry.addData("kStatic", "%.4f", s.kStatic)
        telemetry.addData("kV", "%.6f", s.kV)
        telemetry.addData("kA", "%.6f", kA)
        telemetry.addData("kP", "%.4f", g.kP)
        telemetry.addData("kD", "%.4f%s", g.kD, if (g.kdClamped) " (clamped 0)" else "")
        telemetry.addData("(placed at)", "wn=%.1f rad/s, t_s~%.2fs", g.omegaN, SETTLING_TIME_S)
        // Implied free speed at full power -- a guide for the manual MAX_VELOCITY profile limit
        // (which the tuner does not set, as it caps the profile rather than models the plant).
        telemetry.addData("implied max vel", "%.0f deg/s @ power 1.0 (guides MAX_VELOCITY)", (1.0 - s.kStatic) / s.kV)
        if (!Turret.USE_MOTOR_FUSION) telemetry.addLine("kD applies only with USE_MOTOR_FUSION; live-apply keeps kD=0")
        telemetry.addLine(if (applied) "APPLIED live to dashboard (kD per note)." else "Y: apply live to dashboard")
    }

    companion object {
        // Manual centring jog (SETUP), dead-man via triggers.
        private const val JOG_POWER = 0.2

        // Sign probe.
        private const val SIGN_PROBE_POWER = 0.16
        private const val SIGN_PROBE_MIN_DEG = 4.0
        private const val SIGN_PROBE_TIMEOUT = 1.5

        // Travel band (deg from the centred start). Hard limit is the independent backstop; the sweep
        // edges and record band sit well inside it. Turret travel is ~+/-90, so ~20 deg of margin.
        private const val HARD_LIMIT_DEG = 70.0
        private const val BAND_EDGE_DEG = 55.0
        private const val RECORD_BAND_DEG = 45.0
        private const val DYN_START_EDGE_DEG = 55.0

        // Quasistatic sweep. Modest powers only: at higher power the terminal velocity crosses the
        // limited travel too fast to settle; the dynamic step covers the high-accel regime instead.
        // Span from likely-below-breakaway up: low levels give clean low-velocity points; if some
        // stall out (high stiction) the level timeout skips them, and the higher levels still anchor
        // the slope. Powers above ~0.26 cross the limited travel too fast to settle -- the dynamic
        // step covers the high regime instead.
        private val QUASISTATIC_POWERS = doubleArrayOf(0.09, 0.12, 0.15, 0.18, 0.22, 0.26)
        private const val TRAVERSES_PER_POWER = 4 // reversals per power (~2 each direction)
        private const val REVERSAL_BLANK_S = 0.25 // skip accel + backlash right after a reversal
        private const val ACCEL_GATE_DEG_S2 = 300.0 // record only when |accel| below this (settled)
        private const val MIN_RECORD_SPEED_DEG_S = 8.0
        // Give up on a power level if it produces no reversal within this long (below breakaway, or
        // a jam) and move to the next -- must exceed the ping-pong half-period at the slowest power.
        private const val QUASISTATIC_LEVEL_TIMEOUT = 3.5

        // Dynamic steps.
        private const val REPO_POWER = 0.18
        private const val REPO_TOL_DEG = 4.0
        private const val REPO_TIMEOUT = 3.0        // abort if a start edge is unreachable (jam/disconnect)
        private const val SETTLE_SPEED_DEG_S = 10.0
        private const val SETTLE_DWELL_S = 0.3
        private const val SETTLE_TIMEOUT = 2.0      // skip the step if it never settles to rest
        private const val PLATEAU_FRACTION = 0.9 // end the step once v reaches this fraction of vTerm
        private const val MAX_STEP_TIME_S = 0.6

        // Feedback placement knob: closed-loop settling time (t_s ~= 4/wn). Smaller = stiffer.
        private const val SETTLING_TIME_S = 0.2

        private const val STICTION_ASYMMETRY_WARN = 0.02
    }
}

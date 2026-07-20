package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.control.CurrentLimiter
import org.firstinspires.ftc.teamcode.control.FeedforwardFit
import org.firstinspires.ftc.teamcode.control.KneeFinder
import org.firstinspires.ftc.teamcode.control.PolePlacement
import org.firstinspires.ftc.teamcode.hardware.Names
import org.firstinspires.ftc.teamcode.hardware.subsystem.Launcher
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.min

/**
 * Automatic tuner for the launcher flywheel -- the launcher counterpart to "Turret Auto-Tune". It
 * characterizes the flywheel open-loop and prints the three velocity-loop gains (kS, kV, kP) ready to
 * paste into [Launcher], AND sweeps the spin-up current limit to recommend a `SPINUP_CURRENT_LIMIT_A`
 * at the "knee" of the spin-up-time-vs-current trade-off (fastest spin-up within a sane current
 * budget). Everything is open-loop and touches only the two launcher motors, like RPMTest/ShooterBench.
 *
 * Two phases, both HOLD-TO-RUN (power only while RIGHT BUMPER is held; release / press B cuts it):
 *
 *  A. FEEDFORWARD/PID -- from rest, apply a mid power and log the spin-up rise: its plateau gives the
 *     plant time constant tau (via [FeedforwardFit.fitTimeConstant]) and the first steady-state point.
 *     Then step through more powers, recording (power, TPS) once each settles. A single-direction
 *     least-squares line gives kS/kV ([FeedforwardFit.fitSteadyStateSingleDirection]); kP comes from a
 *     first-order pole placement on (kV, kA=tau*kV) ([PolePlacement.velocityLoopGain]).
 *
 *  B. CURRENT KNEE -- for a sweep of current limits (up to [CURRENT_LIMIT_CEILING_A] per motor), spin
 *     up from rest under a real [CurrentLimiter] (the same governor the subsystem runs, so the result
 *     transfers) and time how long it takes to reach a reference speed, recording the measured peak
 *     current too. [KneeFinder] picks the knee of (limit -> time); the recommendation is floored at the
 *     observed running current so the limit can never throttle the wheel at speed.
 *
 * SAFETY: single-direction only (power clamped to [0,1], never negative). A runaway guard aborts if
 * speed exceeds ~1.15x MAX_TPS or a motor draws more than [CURRENT_ABORT_A] (well above the swept
 * limits -- the enforced limit is soft, so the brief spin-up spike is expected to exceed the setpoint).
 * Per-phase timeouts guard a stuck spin-up/down.
 */
@TeleOp(name = "Launcher Auto-Tune", group = "Debug")
class LauncherAutoTune : OpMode() {
    private val left by lazy { hardwareMap.get(DcMotorEx::class.java, Names.Motors.Launcher.leftMotor) }
    private val right by lazy { hardwareMap.get(DcMotorEx::class.java, Names.Motors.Launcher.rightMotor) }
    private val battery by lazy { hardwareMap.voltageSensor.iterator().next() as VoltageSensor }

    private enum class Phase { READY, A_TAU, A_SWEEP, B_SPINDOWN, B_SPINUP, REPORT, ABORTED }
    private var phase = Phase.READY
    private var message = "HOLD RIGHT BUMPER to run. Flywheel will spin -- keep clear."

    private val timer = ElapsedTime()
    private var now = 0.0
    private var lastNow = 0.0
    private var phaseEnter = 0.0
    private var paused = false

    private var avgTPS = 0.0
    private var filteredCurrent = 0.0
    private var currentInited = false
    private var minVoltage = Double.MAX_VALUE

    // settle detector (tolerance-band on raw TPS)
    private var settleRefTPS = 0.0
    private var settleRefTime = 0.0

    // --- Phase A ---
    private var aIndex = 0
    private var spinupStart = 0.0
    private val riseSamples = ArrayList<FeedforwardFit.StepSample>()
    private val steadySamples = ArrayList<FeedforwardFit.Sample>()

    // --- Phase B ---
    private var bIndex = 0
    private val bLimiter = CurrentLimiter()
    private var referenceTPS = 0.0
    private var bPeakCurrent = 0.0
    private val kneePoints = ArrayList<KneeFinder.Point>()
    private var maxRunningCurrent = 0.0

    // --- results ---
    private var ss: FeedforwardFit.SingleDirectionResult? = null
    private var tau = 0.0
    private var kA = 0.0
    private var tauR2 = 0.0
    private var kP = Launcher.kP
    private var kpFromTune = false
    private var recommendedLimit = 0.0
    private var kneeWelldefined = false
    private var applied = false

    private var lastB = false
    private var lastY = false

    override fun init() {
        left.direction = DcMotorSimple.Direction.FORWARD
        right.direction = DcMotorSimple.Direction.REVERSE
        listOf(left, right).forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.power = 0.0
        }
        bLimiter.recoveryPerSecond = Launcher.SPINUP_RECOVERY_PER_SEC // match the subsystem exactly
        telemetry.addLine("LAUNCHER AUTO-TUNE -- flywheel WILL spin. Keep hands/artifacts clear.")
        telemetry.addLine("HOLD RIGHT BUMPER to run; release or press B to stop instantly.")
    }

    override fun loop() {
        now = timer.seconds()
        val dt = (now - lastNow).coerceIn(0.0, 0.2)

        avgTPS = (abs(left.velocity) + abs(right.velocity)) / 2.0
        val rawCurrent = max(abs(left.getCurrent(CurrentUnit.AMPS)), abs(right.getCurrent(CurrentUnit.AMPS)))
        if (!currentInited) { filteredCurrent = rawCurrent; currentInited = true }
        else filteredCurrent += (1.0 - exp(-dt / CURRENT_FILTER_TAU)) * (rawCurrent - filteredCurrent)
        minVoltage = minOf(minVoltage, battery.voltage)

        if (gamepad1.b && !lastB && phase != Phase.REPORT) abort("Stopped by operator (B).")
        lastB = gamepad1.b

        // Runaway guards (independent of the sequencer).
        if (phase != Phase.READY && phase != Phase.REPORT && phase != Phase.ABORTED) {
            if (avgTPS > Launcher.MAX_TPS * 1.15) abort("Overspeed %.0f TPS -- aborted.".format(avgTPS))
            if (rawCurrent > CURRENT_ABORT_A) abort("Overcurrent %.1f A -- aborted.".format(rawCurrent))
        }

        when (phase) {
            Phase.READY -> { drive(0.0); if (gamepad1.right_bumper) { go(Phase.A_TAU); onEnterPowerLevel() } }
            Phase.REPORT -> { drive(0.0); runReport() }
            Phase.ABORTED -> drive(0.0)
            else -> runAutoPhase(dt)
        }

        lastNow = now
        renderTelemetry()
    }

    override fun stop() {
        left.power = 0.0
        right.power = 0.0
    }

    private fun runAutoPhase(dt: Double) {
        if (!gamepad1.right_bumper) {
            drive(0.0)
            paused = true
            message = "PAUSED - hold RIGHT BUMPER to run."
            return
        }
        if (paused) {
            paused = false
            phaseEnter = now
            // Restart the current sub-move cleanly so a stale gap can't corrupt a timing/settle window.
            settleRefTPS = avgTPS
            settleRefTime = now
            if (phase == Phase.B_SPINUP) restartSpinup()
            if (phase == Phase.A_TAU) { spinupStart = now; riseSamples.clear() }
        }

        when (phase) {
            Phase.A_TAU -> phaseATau()
            Phase.A_SWEEP -> phaseASweep()
            Phase.B_SPINDOWN -> phaseBSpindown()
            Phase.B_SPINUP -> phaseBSpinup(dt)
            else -> drive(0.0)
        }
    }

    // ---- Phase A: feedforward/PID ----

    private fun phaseATau() {
        val power = A_POWERS[0]
        drive(power)
        riseSamples.add(FeedforwardFit.StepSample(now - spinupStart, avgTPS))
        if (settled()) {
            steadySamples.add(FeedforwardFit.Sample(power, avgTPS))
            // vTerm for the tau fit is this measured plateau (the flywheel actually reaches it).
            fitTauFrom(riseSamples, avgTPS)
            aIndex = 1
            go(Phase.A_SWEEP)
            onEnterPowerLevel()
        } else if (now - phaseEnter > A_SETTLE_TIMEOUT) {
            abort("First spin-up never settled -- check the flywheel/motors.")
        }
    }

    private fun phaseASweep() {
        val power = A_POWERS[aIndex]
        drive(power)
        if (settled()) {
            steadySamples.add(FeedforwardFit.Sample(power, avgTPS))
            aIndex++
            if (aIndex >= A_POWERS.size) finishPhaseA() else onEnterPowerLevel()
        } else if (now - phaseEnter > A_SETTLE_TIMEOUT) {
            // Skip a level that won't settle rather than hang; the fit tolerates fewer points.
            message = "Power %.2f never settled; skipping.".format(power)
            aIndex++
            if (aIndex >= A_POWERS.size) finishPhaseA() else onEnterPowerLevel()
        }
    }

    private fun finishPhaseA() {
        try {
            ss = FeedforwardFit.fitSteadyStateSingleDirection(steadySamples, Launcher.MIN_TPS)
        } catch (e: IllegalArgumentException) {
            abort("Steady-state fit failed: ${e.message}")
            return
        }
        val s = ss!!
        // kP from first-order pole placement, if tau came out clean; otherwise keep the shipped kP.
        if (tau > 0.0 && tauR2 >= MIN_TAU_R2) {
            kA = tau * s.kV
            val g = PolePlacement.velocityLoopGain(s.kV, kA, tau / CLOSED_LOOP_SPEEDUP)
            kP = g.kP
            kpFromTune = true
        }
        // Reference speed for the current-knee sweep: a high, reachable fraction of the full-power
        // terminal (power=1 => v=(1-kS)/kV), capped for safety below MAX_TPS.
        val fullPowerTerminal = (1.0 - s.kStatic) / s.kV
        referenceTPS = min(REFERENCE_FRACTION * fullPowerTerminal, 0.9 * Launcher.MAX_TPS)
        bIndex = 0
        go(Phase.B_SPINDOWN)
        message = "FF done (${steadySamples.size} pts). Current-knee sweep to %.0f TPS...".format(referenceTPS)
    }

    // ---- Phase B: current-limit knee sweep ----

    private fun phaseBSpindown() {
        drive(0.0) // BRAKE zero-power bleeds the wheel down
        if (avgTPS < SPINDOWN_TPS) {
            restartSpinup()
            go(Phase.B_SPINUP)
        } else if (now - phaseEnter > B_SPINDOWN_TIMEOUT) {
            // Coasting slowly; proceed anyway -- the spin-up timing just starts from a low speed.
            restartSpinup()
            go(Phase.B_SPINUP)
        }
    }

    private fun restartSpinup() {
        bLimiter.limitAmps = B_LIMITS[bIndex]
        bLimiter.recoveryPerSecond = Launcher.SPINUP_RECOVERY_PER_SEC
        bLimiter.reset()
        spinupStart = now
        bPeakCurrent = 0.0
    }

    private fun phaseBSpinup(dt: Double) {
        // Command the current cap directly: at full demand the subsystem's min(FF+P, cap) == cap, so
        // this reproduces the real spin-up without re-deriving the FF+P law.
        val cap = bLimiter.update(filteredCurrent, dt)
        drive(cap)
        bPeakCurrent = max(bPeakCurrent, filteredCurrent)

        if (avgTPS >= referenceTPS) {
            val elapsed = now - spinupStart
            kneePoints.add(KneeFinder.Point(B_LIMITS[bIndex], elapsed))
            maxRunningCurrent = max(maxRunningCurrent, filteredCurrent) // ~steady draw at reference
            bIndex++
            if (bIndex >= B_LIMITS.size) finishPhaseB() else go(Phase.B_SPINDOWN)
        } else if (now - spinupStart > B_SPINUP_TIMEOUT) {
            // This limit is too low to reach the reference in time -- record it as the timeout (a very
            // slow spin-up) so the knee still sees a monotone curve, then move on.
            kneePoints.add(KneeFinder.Point(B_LIMITS[bIndex], B_SPINUP_TIMEOUT))
            bIndex++
            if (bIndex >= B_LIMITS.size) finishPhaseB() else go(Phase.B_SPINDOWN)
        }
    }

    private fun finishPhaseB() {
        drive(0.0)
        val knee = KneeFinder.findKnee(kneePoints)
        kneeWelldefined = knee.welldefined
        // Floor the recommendation at the running current so the limit can't throttle the wheel at
        // speed (a limit below the hold current would stall shots).
        recommendedLimit = max(knee.x, maxRunningCurrent * RUNNING_CURRENT_MARGIN)
        go(Phase.REPORT)
        message = "DONE. Y = apply live. Copy the gains + limit into Launcher."
    }

    // ---- REPORT ----

    private fun runReport() {
        if (gamepad1.y && !lastY && ss != null && !applied) {
            Launcher.kS = ss!!.kStatic
            Launcher.kV = ss!!.kV
            if (kpFromTune) Launcher.kP = kP
            Launcher.SPINUP_CURRENT_LIMIT_A = recommendedLimit
            applied = true
        }
        lastY = gamepad1.y
    }

    // ---- helpers ----

    private fun drive(power: Double) {
        val safe = if (phase == Phase.ABORTED) 0.0 else Range.clip(power, 0.0, 1.0)
        left.power = safe
        right.power = safe
    }

    private fun go(next: Phase) {
        phase = next
        phaseEnter = now
    }

    private fun onEnterPowerLevel() {
        // (Re)start the settle detector for the newly-commanded power level.
        settleRefTPS = avgTPS
        settleRefTime = now
        if (phase == Phase.A_TAU) { spinupStart = now; riseSamples.clear() }
    }

    /** Settled == raw TPS hasn't moved more than a band over the dwell window (robust on noisy TPS). */
    private fun settled(): Boolean {
        if (abs(avgTPS - settleRefTPS) > SETTLE_BAND_TPS) {
            settleRefTPS = avgTPS
            settleRefTime = now
        }
        return now - settleRefTime > SETTLE_DWELL_S
    }

    private fun fitTauFrom(rise: List<FeedforwardFit.StepSample>, vTerm: Double) {
        try {
            val r = FeedforwardFit.fitTimeConstant(rise, vTerm, ss?.kV ?: Launcher.kV)
            tau = r.tau
            tauR2 = r.r2
        } catch (e: IllegalArgumentException) {
            tau = 0.0
            tauR2 = 0.0
            message = "tau fit failed (${e.message}); kP will keep its default."
        }
    }

    private fun abort(reason: String) {
        left.power = 0.0
        right.power = 0.0
        message = reason
        go(Phase.ABORTED)
    }

    private fun renderTelemetry() {
        telemetry.addData("PHASE", phase)
        telemetry.addData("note", message)
        telemetry.addData("TPS", "%.0f", avgTPS)
        telemetry.addData("current (A, filtered)", "%.1f", filteredCurrent)
        telemetry.addData("min battery (V)", if (minVoltage == Double.MAX_VALUE) 0.0 else minVoltage)

        when (phase) {
            Phase.A_TAU, Phase.A_SWEEP ->
                telemetry.addData("FF sweep", "power %.2f (%d/%d), %d pts",
                    A_POWERS[aIndex.coerceAtMost(A_POWERS.size - 1)], aIndex + 1, A_POWERS.size, steadySamples.size)
            Phase.B_SPINDOWN, Phase.B_SPINUP ->
                telemetry.addData("knee sweep", "limit %.0f A (%d/%d) -> ref %.0f TPS",
                    B_LIMITS[bIndex.coerceAtMost(B_LIMITS.size - 1)], bIndex + 1, B_LIMITS.size, referenceTPS)
            Phase.REPORT -> renderReport()
            else -> {}
        }
        if (phase != Phase.READY && phase != Phase.REPORT && phase != Phase.ABORTED) {
            telemetry.addLine("HOLD RIGHT BUMPER to run | B: stop")
        }
    }

    private fun renderReport() {
        val s = ss ?: return
        telemetry.addLine("--- PASTE INTO Launcher companion ---")
        telemetry.addData("kS", "%.4f", s.kStatic)
        telemetry.addData("kV", "%.6f", s.kV)
        telemetry.addData("kP", "%.6f%s", kP, if (kpFromTune) "" else " (kept default -- poor tau fit)")
        telemetry.addData("SPINUP_CURRENT_LIMIT_A", "%.1f", recommendedLimit)
        telemetry.addLine("--- diagnostics ---")
        telemetry.addData("FF fit R2 / n", "%.3f / %d", s.r2, s.n)
        telemetry.addData("tau (s) / R2", "%.3f / %.3f", tau, tauR2)
        telemetry.addData("full-power terminal TPS", "%.0f", (1.0 - s.kStatic) / s.kV)
        telemetry.addData("running current at ref (A)", "%.1f", maxRunningCurrent)
        if (!kneeWelldefined) telemetry.addLine("! no clear knee -- using fastest tested limit; widen the sweep")
        telemetry.addLine("time-vs-limit (A -> s):")
        kneePoints.forEach { telemetry.addData("  %.0f A".format(it.x), "%.2f s".format(it.y)) }
        telemetry.addLine(if (applied) "APPLIED live (persists into next TeleOp)." else "Y: apply live")
    }

    companion object {
        // Phase A open-loop sweep. First entry is applied FROM REST (its rise gives tau); the rest are
        // stepped through for steady-state points. Spread of powers = spread of velocities for the fit.
        private val A_POWERS = doubleArrayOf(0.50, 0.65, 0.80, 0.35, 0.25)
        private const val SETTLE_BAND_TPS = 30.0   // "settled" if TPS stays within this band...
        private const val SETTLE_DWELL_S = 0.35    // ...for this long
        private const val A_SETTLE_TIMEOUT = 4.0   // give up on a power level after this

        // Phase B current-limit sweep (per motor, amps), up to the characterization ceiling.
        private val B_LIMITS = doubleArrayOf(6.0, 8.0, 10.0, 12.0, 14.0, CURRENT_LIMIT_CEILING_A)
        private const val CURRENT_LIMIT_CEILING_A = 15.0 // matches the subsystem's current alert
        private const val REFERENCE_FRACTION = 0.85      // reference speed = this x full-power terminal
        private const val SPINDOWN_TPS = 150.0           // "at rest enough" to start a spin-up
        private const val B_SPINUP_TIMEOUT = 8.0
        private const val B_SPINDOWN_TIMEOUT = 6.0
        private const val RUNNING_CURRENT_MARGIN = 1.2   // floor the knee this far above the hold current

        // kP placement: closed-loop time constant = openLoop tau / this (conservative -> kP ~ 0.5*kV).
        private const val CLOSED_LOOP_SPEEDUP = 1.5
        private const val MIN_TAU_R2 = 0.90              // below this, keep the shipped kP

        private const val CURRENT_FILTER_TAU = 0.03      // s, light low-pass on the measured current
        private const val CURRENT_ABORT_A = 30.0         // runaway backstop (well above swept limits)
    }
}

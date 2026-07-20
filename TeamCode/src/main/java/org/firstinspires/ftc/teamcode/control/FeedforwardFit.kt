package org.firstinspires.ftc.teamcode.control

import kotlin.math.abs
import kotlin.math.ln

/**
 * SysId-style feedforward identification for the turret's power-based control law
 *
 *     power = kStatic * sign(v) + kV * v + kA * a
 *
 * fit SEQUENTIALLY (each step conditions on the previous). A single three-parameter regression
 * `power ~ sign(v) + v + a` is badly conditioned because kA would lean entirely on the noisiest
 * column -- a differentiated, hence noise-amplified, acceleration. Instead:
 *
 *  1. [fitSteadyState] -- from constant-power ("quasistatic") samples taken where acceleration has
 *     died out, an ordinary least-squares line `power = kStatic*sign(v) + kV*v` gives kV (slope) and
 *     kStatic (intercept). Each travel direction is fit independently so an asymmetric static
 *     friction (kStatic+ vs kStatic-) is surfaced rather than averaged away -- a large split flags a
 *     mechanical problem (a rubbing wire, a tight bearing) rather than a control gain.
 *
 *  2. [fitTimeConstant] -- from a constant-power step response, velocity rises as
 *     `v(t) = vTerm * (1 - e^(-t/tau))`. Linearising, `ln(1 - v/vTerm) = -t/tau`, so an OLS line
 *     through `(t, ln(1 - v/vTerm))` recovers tau WITHOUT differentiating velocity. Then
 *     `kA = tau * kV` (from `tau = kA/kV`, the plant's mechanical time constant). Because vTerm is
 *     predicted from the already-identified (kV, kStatic), the step need not reach its plateau --
 *     important on a travel-limited turret that would hit a hard stop first. The fit window
 *     [fitLow, fitHigh] drops the near-zero start (dominated by backlash/stiction) and the
 *     near-plateau tail (where `ln(1 - v/vTerm)` is dominated by vTerm error and noise).
 *
 * Pure and unit-tested on the JVM: no hardware, no clock. The on-robot `TurretAutoTune` OpMode
 * collects the samples and feeds them here; the resulting (kV, kA) go on to [PolePlacement] for the
 * feedback gains.
 */
object FeedforwardFit {
    /** One steady-state (constant-power, settled) sample: commanded [power], measured [velocity]. */
    data class Sample(val power: Double, val velocity: Double)

    /** One point on a step response: time [t] since the step onset, measured [velocity]. */
    data class StepSample(val t: Double, val velocity: Double)

    /** An ordinary-least-squares line `y = slope*x + intercept` with its fit quality. */
    data class LineFit(val slope: Double, val intercept: Double, val r2: Double, val n: Int)

    data class SteadyStateResult(
        val kV: Double,               // count-weighted slope across both directions (power per deg/s)
        val kStatic: Double,          // symmetric average the control law's kStatic*sign(v) term uses
        val kStaticPositive: Double,  // intercept driving the turret in the +v direction
        val kStaticNegative: Double,  // intercept driving the turret in the -v direction
        val positive: LineFit,        // fit over v > +minSpeed samples
        val negative: LineFit,        // fit over v < -minSpeed samples
    )

    data class TimeConstantResult(
        val tau: Double,              // mechanical time constant (s)
        val kA: Double,               // = tau * kV (power per deg/s^2)
        val r2: Double,
        val n: Int,
    )

    data class SingleDirectionResult(
        val kV: Double,               // slope (power per unit velocity)
        val kStatic: Double,          // intercept (breakaway power in the driven direction)
        val r2: Double,
        val n: Int,
    )

    /**
     * Fit `power = kStatic*sign(v) + kV*v` to [samples], one line per travel direction.
     *
     * [minSpeed] (deg/s) drops near-stationary samples: below breakaway the velocity is ~0 while the
     * turret is still powered, which corrupts the intercept (that is the stiction we want to measure,
     * not fit through). Requires at least two usable samples on each side.
     */
    fun fitSteadyState(samples: List<Sample>, minSpeed: Double = 5.0): SteadyStateResult {
        val pos = samples.filter { it.velocity > minSpeed }
        val neg = samples.filter { it.velocity < -minSpeed }
        require(pos.size >= 2) { "need >= 2 samples with v > +$minSpeed deg/s, had ${pos.size}" }
        require(neg.size >= 2) { "need >= 2 samples with v < -$minSpeed deg/s, had ${neg.size}" }

        // power = kStatic*(+1) + kV*v  -> slope = kV, intercept = +kStatic
        val posFit = ols(pos.map { it.velocity }, pos.map { it.power })
        // power = kStatic*(-1) + kV*v  -> slope = kV, intercept = -kStatic
        val negFit = ols(neg.map { it.velocity }, neg.map { it.power })

        val kStaticPositive = posFit.intercept
        val kStaticNegative = -negFit.intercept

        val kV = (posFit.slope * pos.size + negFit.slope * neg.size) / (pos.size + neg.size)
        val kStatic = (kStaticPositive + kStaticNegative) / 2.0

        return SteadyStateResult(kV, kStatic, kStaticPositive, kStaticNegative, posFit, negFit)
    }

    /**
     * Single-direction steady-state fit: `power = kStatic + kV*v` over one travel direction, for a
     * mechanism that only ever moves one way (the launcher flywheel is clamped to `[0,1]` power and
     * cannot reverse, so [fitSteadyState]'s two-direction requirement can't be met). `kV` is the line
     * slope, `kStatic` its intercept (the breakaway power in the driven direction).
     *
     * [minSpeed] is in the sample's velocity unit (for the flywheel, TPS -- pass ~`MIN_TPS`); it drops
     * near-stationary powered samples below breakaway, which would otherwise corrupt the intercept.
     * Samples are taken by absolute velocity so the same call works whichever sign the encoder reports.
     */
    fun fitSteadyStateSingleDirection(samples: List<Sample>, minSpeed: Double): SingleDirectionResult {
        val usable = samples.filter { abs(it.velocity) > minSpeed }
        require(usable.size >= 2) { "need >= 2 samples with |v| > $minSpeed, had ${usable.size}" }
        val fit = ols(usable.map { it.velocity }, usable.map { it.power })
        return SingleDirectionResult(fit.slope, fit.intercept, fit.r2, fit.n)
    }

    /**
     * Recover the plant time constant tau from a single step response and return `kA = tau*kV`.
     *
     * [vTerm] is the terminal velocity the step would reach (predicted from the identified
     * (kV, kStatic): `vTerm = (|power| - kStatic) / kV`), so the step need not actually plateau.
     * A constant time offset between the true step onset and `t = 0` is harmless: it only shifts the
     * fitted line's intercept, never its slope, and tau comes solely from the slope.
     */
    fun fitTimeConstant(
        samples: List<StepSample>,
        vTerm: Double,
        kV: Double,
        fitLow: Double = 0.1,
        fitHigh: Double = 0.85,
    ): TimeConstantResult {
        require(vTerm != 0.0) { "vTerm must be non-zero" }
        require(fitLow in 0.0..fitHigh && fitHigh < 1.0) { "need 0 <= fitLow <= fitHigh < 1" }

        val mag = abs(vTerm)
        val xs = ArrayList<Double>()
        val ys = ArrayList<Double>()
        for (s in samples) {
            val ratio = abs(s.velocity) / mag
            if (ratio < fitLow || ratio > fitHigh) continue
            xs.add(s.t)
            ys.add(ln(1.0 - ratio)) // model: ln(1 - v/vTerm) = -t/tau (+ const)
        }
        require(xs.size >= 2) {
            "need >= 2 step samples with v/vTerm in [$fitLow, $fitHigh], had ${xs.size}"
        }

        val fit = ols(xs, ys)
        require(fit.slope < 0.0) {
            "step-response slope must be negative (velocity rising toward vTerm), was ${fit.slope}"
        }
        val tau = -1.0 / fit.slope
        return TimeConstantResult(tau, tau * kV, fit.r2, xs.size)
    }

    /** Ordinary least squares of y on x. Requires >= 2 points and non-zero variance in x. */
    private fun ols(x: List<Double>, y: List<Double>): LineFit {
        val n = x.size
        require(n >= 2 && n == y.size) { "need >= 2 matched (x,y) points, had ${x.size}/${y.size}" }

        val meanX = x.average()
        val meanY = y.average()
        var sxx = 0.0
        var sxy = 0.0
        for (i in 0 until n) {
            val dx = x[i] - meanX
            sxx += dx * dx
            sxy += dx * (y[i] - meanY)
        }
        require(sxx > 0.0) { "x has zero variance; cannot fit a line" }

        val slope = sxy / sxx
        val intercept = meanY - slope * meanX

        var ssRes = 0.0
        var ssTot = 0.0
        for (i in 0 until n) {
            val predicted = slope * x[i] + intercept
            ssRes += (y[i] - predicted) * (y[i] - predicted)
            ssTot += (y[i] - meanY) * (y[i] - meanY)
        }
        val r2 = if (ssTot > 0.0) 1.0 - ssRes / ssTot else 1.0

        return LineFit(slope, intercept, r2, n)
    }
}

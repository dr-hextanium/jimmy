package org.firstinspires.ftc.teamcode.control

import org.firstinspires.ftc.teamcode.control.FeedforwardFit.Sample
import org.firstinspires.ftc.teamcode.control.FeedforwardFit.StepSample
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.exp
import kotlin.math.sign

/**
 * Tests for the feedforward identification math. These prove the *fitting algebra*: fabricate
 * samples from a known ground-truth model (kStatic, kV, kA/tau), optionally add noise, and assert
 * the fit recovers the truth. They do NOT validate any on-robot gain value -- those come out of the
 * TurretAutoTune OpMode.
 */
class FeedforwardFitTest {
    // Ground-truth feedforward the synthetic data is generated from.
    private val kStatic = 0.02
    private val kV = 0.0012
    private val eps = 1e-9

    // ---- steady-state fit (kV, kStatic) ----

    private fun steadyState(kSPos: Double, kSNeg: Double): List<Sample> {
        // power = kStatic*sign(v) + kV*v, swept across velocities in both directions.
        val out = ArrayList<Sample>()
        for (v in listOf(30.0, 60.0, 90.0, 120.0, 150.0)) {
            out.add(Sample(kSPos + kV * v, v))       // +v
            out.add(Sample(-kSNeg + kV * (-v), -v))  // -v
        }
        return out
    }

    @Test
    fun steadyState_recoversSymmetricGains() {
        val r = FeedforwardFit.fitSteadyState(steadyState(kStatic, kStatic))
        assertEquals(kV, r.kV, 1e-6)
        assertEquals(kStatic, r.kStatic, 1e-6)
        assertEquals(kStatic, r.kStaticPositive, 1e-6)
        assertEquals(kStatic, r.kStaticNegative, 1e-6)
        assertEquals(1.0, r.positive.r2, 1e-9)
        assertEquals(1.0, r.negative.r2, 1e-9)
    }

    @Test
    fun steadyState_surfacesAsymmetricStiction() {
        // Heavier breakaway in the negative direction (e.g. a rubbing wire on one side).
        val r = FeedforwardFit.fitSteadyState(steadyState(0.018, 0.031))
        assertEquals(0.018, r.kStaticPositive, 1e-6)
        assertEquals(0.031, r.kStaticNegative, 1e-6)
        assertEquals((0.018 + 0.031) / 2.0, r.kStatic, 1e-6)
        assertEquals(kV, r.kV, 1e-6) // slope is unaffected by the intercept split
    }

    @Test
    fun steadyState_recoversGainsUnderNoise() {
        val rng = java.util.Random(42)
        val noisy = steadyState(kStatic, kStatic).flatMap { s ->
            // Repeat each operating point with small measurement noise, as an on-robot sweep would.
            (0 until 20).map {
                Sample(s.power, s.velocity + rng.nextGaussian() * 2.0) // +/- ~2 deg/s noise
            }
        }
        val r = FeedforwardFit.fitSteadyState(noisy)
        assertEquals(kV, r.kV, 1e-4)
        assertEquals(kStatic, r.kStatic, 2e-3)
        assertTrue("fit quality should be high", r.positive.r2 > 0.95 && r.negative.r2 > 0.95)
    }

    @Test
    fun steadyState_dropsSubBreakawaySamples() {
        // Add stationary-but-powered points (v ~ 0): they must be excluded so they don't corrupt
        // the intercept. minSpeed defaults to 5 deg/s.
        val data = steadyState(kStatic, kStatic) +
            listOf(Sample(0.015, 0.0), Sample(0.015, 1.0), Sample(-0.015, -0.5))
        val r = FeedforwardFit.fitSteadyState(data)
        assertEquals(kStatic, r.kStatic, 1e-6)
        assertEquals(kV, r.kV, 1e-6)
    }

    @Test(expected = IllegalArgumentException::class)
    fun steadyState_rejectsTooFewSamplesOnASide() {
        FeedforwardFit.fitSteadyState(
            listOf(Sample(0.1, 50.0), Sample(0.15, 100.0), Sample(-0.1, -50.0)) // only 1 negative
        )
    }

    // ---- time-constant fit (tau, kA) ----

    private fun stepResponse(vTerm: Double, tau: Double, dt: Double, duration: Double, t0: Double = 0.0):
        List<StepSample> {
        val out = ArrayList<StepSample>()
        var t = 0.0
        while (t <= duration) {
            // v measured on a clock offset from the true onset by t0 (the fit must be immune to it).
            val vTrue = vTerm * (1.0 - exp(-(t) / tau))
            out.add(StepSample(t + t0, vTrue))
            t += dt
        }
        return out
    }

    @Test
    fun timeConstant_recoversTauAndKa() {
        val tau = 0.05
        val vTerm = 150.0
        val r = FeedforwardFit.fitTimeConstant(stepResponse(vTerm, tau, 0.005, 0.25), vTerm, kV)
        assertEquals(tau, r.tau, 1e-6)
        assertEquals(tau * kV, r.kA, eps)
        assertEquals(1.0, r.r2, 1e-6)
    }

    @Test
    fun timeConstant_immuneToOnsetOffset() {
        // A constant offset between true onset and t=0 shifts only the line's intercept, not slope.
        val tau = 0.04
        val vTerm = -120.0 // negative-direction step
        val r = FeedforwardFit.fitTimeConstant(
            stepResponse(vTerm, tau, 0.004, 0.2, t0 = 0.037), vTerm, kV
        )
        assertEquals(tau, r.tau, 1e-6)
        assertTrue("kA should be positive", r.kA > 0.0)
    }

    @Test
    fun timeConstant_worksWithoutReachingPlateau() {
        // Cut the step off early (turret would hit a hard stop): still recovers tau from the rise.
        val tau = 0.06
        val vTerm = 200.0
        val truncated = stepResponse(vTerm, tau, 0.005, 0.09) // ~1.5 tau of data only
        val r = FeedforwardFit.fitTimeConstant(truncated, vTerm, kV)
        assertEquals(tau, r.tau, 1e-3)
    }

    @Test
    fun timeConstant_recoversTauUnderNoise() {
        val rng = java.util.Random(7)
        val tau = 0.05
        val vTerm = 150.0
        val noisy = stepResponse(vTerm, tau, 0.005, 0.25).map {
            StepSample(it.t, it.velocity + rng.nextGaussian() * 1.5)
        }
        val r = FeedforwardFit.fitTimeConstant(noisy, vTerm, kV)
        assertEquals(tau, r.tau, 5e-3)
        assertTrue("fit quality should be high", r.r2 > 0.95)
    }

    @Test(expected = IllegalArgumentException::class)
    fun timeConstant_rejectsFallingResponse() {
        // Velocity decreasing over time can't be a rising step toward vTerm -> positive slope.
        val falling = (0..20).map { StepSample(it * 0.005, 150.0 - it * 2.0) }
        FeedforwardFit.fitTimeConstant(falling, 150.0, kV)
    }

    // Guard against a silent sign-convention regression in the synthetic generator itself.
    @Test
    fun sanity_syntheticSignsConsistent() {
        steadyState(kStatic, kStatic).forEach { assertEquals(sign(it.power), sign(it.velocity), eps) }
    }
}

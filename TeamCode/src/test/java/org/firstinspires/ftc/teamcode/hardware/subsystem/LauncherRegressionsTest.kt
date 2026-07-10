package org.firstinspires.ftc.teamcode.hardware.subsystem

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Contract tests for Launcher.Regressions and Launcher.LookupTables (pure math, no hardware).
 *
 * Philosophy: these are empirical shooter-tuning curves, so the tests assert *shape and safety
 * contracts* -- output bounds, monotonicity, and the exact wired LUT data points -- rather than
 * pinning the fitted coefficients. A single labeled characterization value guards against
 * accidental coefficient corruption.
 *
 * Documented quirk: for distance > 115 in both regressions the far-field InterpLUT branch is used
 * and is NOT re-clamped to the near-field power/hood bounds, so its outputs deliberately fall
 * outside those near-field bounds (a separate long-range regime).
 */
class LauncherRegressionsTest {
    private val eps = 1e-6

    private val powerLo = Launcher.Regressions.POWER_LOWER_BOUND
    private val powerHi = Launcher.Regressions.POWER_UPPER_BOUND
    private val hoodLo = Launcher.Regressions.HOOD_LOWER_BOUND
    private val hoodHi = Launcher.Regressions.HOOD_UPPER_BOUND

    // ---- powerRegression ----

    @Test
    fun powerRegression_nearFieldStaysWithinBounds() {
        var d = 0.0
        while (d <= 115.0) {
            val p = Launcher.Regressions.powerRegression(d)
            assertTrue("power $p out of bounds at d=$d", p in powerLo..powerHi)
            d += 0.5
        }
    }

    @Test
    fun powerRegression_nearFieldEndpointsClampToBounds() {
        // Raw exponential is below the lower bound at d=0 and above the upper bound by d=115.
        assertEquals(powerLo, Launcher.Regressions.powerRegression(0.0), eps)
        assertEquals(powerHi, Launcher.Regressions.powerRegression(115.0), eps)
    }

    @Test
    fun powerRegression_isMonotonicNonDecreasingAcrossFullRange() {
        var d = 0.0
        var prev = Launcher.Regressions.powerRegression(0.0)
        while (d <= 140.0) {
            val p = Launcher.Regressions.powerRegression(d)
            assertTrue("power dipped at d=$d ($p < $prev)", p >= prev - eps)
            prev = p
            d += 0.5
        }
    }

    @Test
    fun powerRegression_farFieldUsesWiredLutEndpoints() {
        assertEquals(0.82, Launcher.Regressions.powerRegression(122.2), eps)
        assertEquals(0.89, Launcher.Regressions.powerRegression(130.0), eps)
        // Between the > 115 branch and the first LUT point, the LUT clamps to its first value.
        assertEquals(0.82, Launcher.Regressions.powerRegression(118.0), eps)
    }

    // ---- hoodRegression ----

    @Test
    fun hoodRegression_nearFieldStaysWithinBounds() {
        var d = 0.0
        while (d <= 115.0) {
            val h = Launcher.Regressions.hoodRegression(d)
            assertTrue("hood $h out of bounds at d=$d", h in hoodLo..hoodHi)
            d += 0.5
        }
    }

    @Test
    fun hoodRegression_characterizationAtZeroDistance() {
        // Current value = cubic constant term (0.797042). Update this if the curve is re-tuned.
        assertEquals(0.797042, Launcher.Regressions.hoodRegression(0.0), 1e-4)
    }

    @Test
    fun hoodRegression_farFieldUsesWiredLutEndpoints() {
        assertEquals(0.03, Launcher.Regressions.hoodRegression(122.2), eps)
        assertEquals(0.185, Launcher.Regressions.hoodRegression(130.0), eps)
    }

    // ---- LookupTables (direct) ----

    @Test
    fun farPowerLut_endpointsAndMidpoint() {
        val lut = Launcher.LookupTables.farPowerInterpLUT
        assertEquals(0.82, lut[122.2], eps)
        assertEquals(0.89, lut[130.0], eps)
        // Midpoint interpolates linearly between the two data points.
        assertEquals((0.82 + 0.89) / 2.0, lut[(122.2 + 130.0) / 2.0], 1e-6)
    }

    @Test
    fun farHoodLut_endpoints() {
        val lut = Launcher.LookupTables.farHoodInterpLUT
        assertEquals(0.03, lut[122.2], eps)
        assertEquals(0.185, lut[130.0], eps)
    }
}

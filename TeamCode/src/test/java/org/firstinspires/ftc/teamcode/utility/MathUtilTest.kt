package org.firstinspires.ftc.teamcode.utility

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Tests for the standalone math helpers in map.kt and percentDifference.kt.
 *
 * These pin down current behavior, including the deliberately-unclamped [map] and the degenerate
 * divide-by-zero cases of [percentDifference] (NaN / Infinity). Those degenerate results are
 * documented here as characterization, not asserted-as-desirable: callers rely on them being
 * handled downstream (e.g. Launcher.write treats NaN as "no change").
 */
class MathUtilTest {
    private val eps = 1e-9

    // ---- map ----

    @Test
    fun map_linearMidpoint() {
        assertEquals(50.0, map(5.0, 0.0, 10.0, 0.0, 100.0), eps)
    }

    @Test
    fun map_hitsBothEndpoints() {
        assertEquals(0.0, map(0.0, 0.0, 10.0, 0.0, 100.0), eps)
        assertEquals(100.0, map(10.0, 0.0, 10.0, 0.0, 100.0), eps)
    }

    @Test
    fun map_invertedOutputRange() {
        // Output decreasing as input increases (the pattern Launcher uses for the hood).
        assertEquals(75.0, map(0.25, 0.0, 1.0, 100.0, 0.0), eps)
    }

    @Test
    fun map_negativeInputRange() {
        assertEquals(50.0, map(-5.0, -10.0, 0.0, 0.0, 100.0), eps)
    }

    @Test
    fun map_doesNotClamp_extrapolatesBeyondRange() {
        // map (unlike mapClamped) extrapolates past the input bounds.
        assertEquals(150.0, map(15.0, 0.0, 10.0, 0.0, 100.0), eps)
        assertEquals(-50.0, map(-5.0, 0.0, 10.0, 0.0, 100.0), eps)
    }

    @Test
    fun map_zeroWidthInputReturnsOutputMin() {
        // Divide-by-zero guard: identical input bounds -> outputMin.
        assertEquals(10.0, map(5.0, 3.0, 3.0, 10.0, 20.0), eps)
    }

    // ---- mapClamped ----

    @Test
    fun mapClamped_withinRangeMatchesMap() {
        assertEquals(50.0, mapClamped(5.0, 0.0, 10.0, 0.0, 100.0), eps)
    }

    @Test
    fun mapClamped_clampsAboveAndBelow() {
        assertEquals(100.0, mapClamped(15.0, 0.0, 10.0, 0.0, 100.0), eps)
        assertEquals(0.0, mapClamped(-5.0, 0.0, 10.0, 0.0, 100.0), eps)
    }

    @Test
    fun mapClamped_clampsWithInvertedOutputBounds() {
        // Output bounds given high-to-low; clamp must still use the true min/max.
        assertEquals(0.0, mapClamped(15.0, 0.0, 10.0, 100.0, 0.0), eps)
        assertEquals(100.0, mapClamped(-5.0, 0.0, 10.0, 100.0, 0.0), eps)
    }

    @Test
    fun mapClamped_zeroWidthInputReturnsOutputMin() {
        assertEquals(10.0, mapClamped(5.0, 3.0, 3.0, 10.0, 20.0), eps)
    }

    // ---- percentDifference ----

    @Test
    fun percentDifference_normalCase() {
        // (110-100)/((110+100)/2) = 10/105
        assertEquals(10.0 / 105.0, percentDifference(110.0, 100.0), eps)
    }

    @Test
    fun percentDifference_equalValuesIsZero() {
        assertEquals(0.0, percentDifference(50.0, 50.0), eps)
    }

    @Test
    fun percentDifference_isSignedByArgumentOrder() {
        assertTrue(percentDifference(90.0, 100.0) < 0.0)
        assertTrue(percentDifference(100.0, 90.0) > 0.0)
    }

    @Test
    fun percentDifference_bothZero_isNaN() {
        // 0 / 0 -> NaN. Characterization of a degenerate input, not a desired value.
        assertTrue(percentDifference(0.0, 0.0).isNaN())
    }

    @Test
    fun percentDifference_oppositeAndEqual_isInfinite() {
        // a + b == 0 but a != b -> divide by zero -> +/-Infinity.
        assertTrue(percentDifference(5.0, -5.0).isInfinite())
    }

    // ---- absPercentDifference ----

    @Test
    fun absPercentDifference_isNonNegativeAndSymmetric() {
        val ab = absPercentDifference(110.0, 100.0)
        val ba = absPercentDifference(100.0, 110.0)
        assertTrue(ab >= 0.0)
        assertEquals(ab, ba, eps) // |pct(a,b)| == |pct(b,a)|
    }

    @Test
    fun absPercentDifference_matchesAbsOfSigned() {
        assertEquals(
            Math.abs(percentDifference(90.0, 100.0)),
            absPercentDifference(90.0, 100.0),
            eps
        )
    }
}

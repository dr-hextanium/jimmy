package org.firstinspires.ftc.teamcode.utility

import org.junit.Assert.assertEquals
import org.junit.Assert.assertThrows
import org.junit.Test

/**
 * Tests for the linear interpolation lookup table used by the launcher's far-field power/hood
 * curves. Covers out-of-range clamping, exact hits, interpolation, the auto-build path, and the
 * empty-table guard.
 */
class InterpLUTTest {
    private val eps = 1e-9

    private fun lut(vararg points: Pair<Double, Double>): InterpLUT {
        val l = InterpLUT()
        points.forEach { l.add(it.first, it.second) }
        l.createLUT()
        return l
    }

    @Test
    fun interpolatesLinearlyBetweenPoints() {
        val l = lut(0.0 to 0.0, 10.0 to 100.0)
        assertEquals(50.0, l[5.0], eps)
        assertEquals(25.0, l[2.5], eps)
    }

    @Test
    fun returnsExactValueAtAKnownPoint() {
        val l = lut(0.0 to 1.0, 10.0 to 2.0, 20.0 to 5.0)
        assertEquals(2.0, l[10.0], eps)
    }

    @Test
    fun clampsBelowFirstAndAboveLast() {
        val l = lut(10.0 to 1.0, 20.0 to 2.0)
        assertEquals(1.0, l[-100.0], eps) // below range -> first y
        assertEquals(1.0, l[10.0], eps)   // exactly first
        assertEquals(2.0, l[20.0], eps)   // exactly last
        assertEquals(2.0, l[999.0], eps)  // above range -> last y
    }

    @Test
    fun sortsPointsAddedOutOfOrder() {
        // Deliberately add descending; createLUT must sort before interpolating.
        val l = lut(20.0 to 200.0, 0.0 to 0.0, 10.0 to 100.0)
        assertEquals(50.0, l[5.0], eps)
        assertEquals(150.0, l[15.0], eps)
    }

    @Test
    fun autoBuildsWhenCreateLutNotCalled() {
        val l = InterpLUT().add(0.0, 0.0).add(10.0, 100.0) // no createLUT()
        assertEquals(50.0, l[5.0], eps)
    }

    @Test
    fun singlePointActsAsConstant() {
        val l = lut(5.0 to 42.0)
        assertEquals(42.0, l[-1.0], eps)
        assertEquals(42.0, l[5.0], eps)
        assertEquals(42.0, l[100.0], eps)
    }

    @Test
    fun addIsChainable() {
        val l = InterpLUT()
        val returned = l.add(1.0, 1.0)
        assertEquals(l, returned)
    }

    @Test
    fun queryingEmptyTableThrows() {
        val l = InterpLUT()
        assertThrows(IllegalArgumentException::class.java) { l[1.0] }
    }
}

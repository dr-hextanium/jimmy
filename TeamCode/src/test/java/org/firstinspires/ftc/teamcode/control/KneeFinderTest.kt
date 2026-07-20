package org.firstinspires.ftc.teamcode.control

import org.firstinspires.ftc.teamcode.control.KneeFinder.Point
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Tests for the knee/elbow finder. These prove it locates the corner of a trade-off curve (steep
 * drop then flat) and degrades sensibly on inputs where no interior knee is defined.
 */
class KneeFinderTest {
    @Test
    fun findsCornerOfSharpLCurve() {
        // Spin-up time drops steeply from 6 A to 10 A, then flattens — the corner is at ~10 A.
        val pts = listOf(
            Point(6.0, 1.60),
            Point(8.0, 0.90),
            Point(10.0, 0.55),
            Point(12.0, 0.50),
            Point(14.0, 0.47),
            Point(16.0, 0.46),
        )
        val knee = KneeFinder.findKnee(pts)
        assertTrue(knee.welldefined)
        assertEquals(10.0, knee.x, 1e-9)
    }

    @Test
    fun ignoresPointOrder() {
        val pts = listOf(
            Point(14.0, 0.47),
            Point(6.0, 1.60),
            Point(12.0, 0.50),
            Point(10.0, 0.55),
            Point(16.0, 0.46),
            Point(8.0, 0.90),
        )
        val knee = KneeFinder.findKnee(pts)
        assertEquals(10.0, knee.x, 1e-9)
    }

    @Test
    fun straightLineHasNoStrongKnee_returnsAnInteriorPoint() {
        // A perfectly linear curve: every interior point is equidistant (zero) from the chord; the
        // finder still returns a valid interior point without error.
        val pts = (0..5).map { Point(it.toDouble(), 10.0 - it.toDouble()) }
        val knee = KneeFinder.findKnee(pts)
        assertTrue(knee.welldefined)
        assertTrue("knee should be an interior point", knee.x > 0.0 && knee.x < 5.0)
    }

    @Test
    fun fewerThanThreePoints_fallsBackToLastByX() {
        val knee = KneeFinder.findKnee(listOf(Point(8.0, 1.0), Point(12.0, 0.5)))
        assertFalse(knee.welldefined)
        assertEquals(12.0, knee.x, 1e-9) // highest current limit = fastest spin-up (safe default)
    }

    @Test
    fun zeroRangeAxis_fallsBack() {
        // All the same spin-up time (flat) -> no knee; fall back to the last (highest-x) point.
        val flat = listOf(Point(6.0, 0.5), Point(10.0, 0.5), Point(14.0, 0.5))
        val knee = KneeFinder.findKnee(flat)
        assertFalse(knee.welldefined)
        assertEquals(14.0, knee.x, 1e-9)
    }

    @Test(expected = IllegalArgumentException::class)
    fun rejectsEmpty() {
        KneeFinder.findKnee(emptyList())
    }
}

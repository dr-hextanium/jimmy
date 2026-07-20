package org.firstinspires.ftc.teamcode.control

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

/**
 * Tests for [ComplementaryFilter]. Asserts contracts -- seeds at the first absolute, dead-reckons at
 * alpha 0, snaps to the absolute at alpha 1, corrects an initial error toward the absolute, rejects a
 * lone spike but can't wedge, holds on non-positive dt, and reports motionDelta/dt as velocity.
 */
class ComplementaryFilterTest {
    private val dt = 0.01

    @Test
    fun firstUpdateSeedsAtAbsoluteMeasurementWithZeroVelocity() {
        val f = ComplementaryFilter()
        f.update(dt, motionDelta = 5.0, absoluteMeasurement = 10.0, alpha = 0.5)
        assertEquals("first update seeds at the absolute, not the motion", 10.0, f.position, 1e-12)
        assertEquals(0.0, f.velocity, 1e-12)
    }

    @Test
    fun resetSeedsPositionAndZeroesVelocity() {
        val f = ComplementaryFilter()
        repeat(10) { f.update(dt, 2.0, 2.0 * it, alpha = 0.5) }
        f.reset(7.0)
        assertEquals(7.0, f.position, 1e-12)
        assertEquals(0.0, f.velocity, 1e-12)
    }

    @Test
    fun alphaZeroIsPureDeadReckoningAndIgnoresAbsolute() {
        val f = ComplementaryFilter()
        f.reset(0.0)
        repeat(10) { f.update(dt, motionDelta = 1.0, absoluteMeasurement = 1000.0, alpha = 0.0) }
        assertEquals("alpha 0 integrates motion and ignores the absolute", 10.0, f.position, 1e-9)
    }

    @Test
    fun alphaOneSnapsToAbsolute() {
        val f = ComplementaryFilter()
        f.reset(0.0)
        f.update(dt, motionDelta = 1.0, absoluteMeasurement = 50.0, alpha = 1.0)
        assertEquals("alpha 1 snaps to the absolute regardless of motion", 50.0, f.position, 1e-9)
    }

    @Test
    fun correctsAnInitialErrorTowardTheAbsolute() {
        // Seeded 5 deg off; motion says "not moving" and absolute says 0 -> the correction must pull
        // the estimate to 0.
        val f = ComplementaryFilter()
        f.reset(5.0)
        repeat(50) { f.update(dt, motionDelta = 0.0, absoluteMeasurement = 0.0, alpha = 0.5) }
        assertEquals(0.0, f.position, 1e-3)
    }

    @Test
    fun tracksExactlyWhenMotionAndAbsoluteAgree() {
        val f = ComplementaryFilter()
        f.reset(0.0)
        var truth = 0.0
        repeat(50) {
            truth += 2.0
            f.update(dt, motionDelta = 2.0, absoluteMeasurement = truth, alpha = 0.2)
        }
        assertEquals(truth, f.position, 1e-9)
    }

    @Test
    fun rejectsAnIsolatedAbsoluteSpike() {
        val f = ComplementaryFilter()
        f.reset(5.0)
        repeat(10) { f.update(dt, 0.0, 5.0, alpha = 0.5, gate = 10.0) }
        f.update(dt, motionDelta = 0.0, absoluteMeasurement = 55.0, alpha = 0.5, gate = 10.0)
        assertTrue("a lone spike must be gated out", abs(f.position - 5.0) < 1e-6)
    }

    @Test
    fun sustainedAbsoluteStepForcesAcceptanceAndSelfHeals() {
        val f = ComplementaryFilter()
        f.reset(5.0)
        repeat(40) { f.update(dt, motionDelta = 0.0, absoluteMeasurement = 25.0, alpha = 0.5, gate = 10.0) }
        assertEquals("a genuine step over the gate must not wedge", 25.0, f.position, 1e-3)
    }

    @Test
    fun nonPositiveDtHoldsTheEstimate() {
        val f = ComplementaryFilter()
        f.reset(3.0)
        f.update(0.0, 99.0, 99.0, alpha = 1.0)
        assertEquals(3.0, f.position, 1e-12)
        f.update(-0.01, 99.0, 99.0, alpha = 1.0)
        assertEquals(3.0, f.position, 1e-12)
    }

    @Test
    fun velocityIsMotionDeltaOverDt() {
        val f = ComplementaryFilter()
        f.reset(0.0)
        f.update(dt, motionDelta = 1.5, absoluteMeasurement = 0.0, alpha = 0.5)
        assertEquals(1.5 / dt, f.velocity, 1e-9)
    }
}

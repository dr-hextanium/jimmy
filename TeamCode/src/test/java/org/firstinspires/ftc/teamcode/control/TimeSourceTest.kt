package org.firstinspires.ftc.teamcode.control

import org.firstinspires.ftc.teamcode.testfakes.FakeTimeSource
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class TimeSourceTest {
    @Test
    fun lambdaSource_returnsSuppliedValue() {
        var t = 5.0
        val src = TimeSource { t }
        assertEquals(5.0, src.seconds(), 0.0)
        t = 9.0
        assertEquals(9.0, src.seconds(), 0.0)
    }

    @Test
    fun fakeSource_advancesDeterministically() {
        val clock = FakeTimeSource()
        assertEquals(0.0, clock.seconds(), 0.0)
        clock.advance(0.02)
        assertEquals(0.02, clock.seconds(), 1e-12)
        clock.advance(0.02)
        assertEquals(0.04, clock.seconds(), 1e-12)
    }

    @Test
    fun systemSource_isMonotonicNonDecreasing() {
        val a = TimeSource.SYSTEM.seconds()
        val b = TimeSource.SYSTEM.seconds()
        assertTrue("system clock went backwards", b >= a)
    }
}

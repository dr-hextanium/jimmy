package org.firstinspires.ftc.teamcode.control

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.max

/**
 * Tests for the flywheel current governor. These prove the governor's *contract* (backs off over the
 * limit, re-opens under it, clamps, inert when disabled/blind) and its *emergent behaviour* against a
 * simple linear motor model: it catches a stall spike in ~one loop and then holds current at the
 * limit through a spin-up, which is the whole point.
 */
class CurrentLimiterTest {
    // Linear motor model at a fixed operating point: current = ampsPerPower * power (0 speed).
    // With back-EMF: current = ampsPerPower * power - ampsPerSpeed * speed.

    @Test
    fun disabled_capsAtOneRegardlessOfCurrent() {
        val cl = CurrentLimiter(limitAmps = 0.0)
        assertEquals(1.0, cl.update(999.0, 0.02), 0.0)
        cl.limitAmps = -5.0
        assertEquals(1.0, cl.update(999.0, 0.02), 0.0)
    }

    @Test
    fun badReading_isInert() {
        val cl = CurrentLimiter(limitAmps = 12.0)
        assertEquals(1.0, cl.update(Double.NaN, 0.02), 0.0)
        assertEquals(1.0, cl.update(-3.0, 0.02), 0.0)
        assertEquals(1.0, cl.update(Double.POSITIVE_INFINITY, 0.02), 0.0)
    }

    @Test
    fun overLimit_backsOffProportionallyInOneStep() {
        val cl = CurrentLimiter(limitAmps = 12.0)
        // cap starts at 1.0; a 24 A reading (2x limit) should halve the cap in a single update.
        val cap = cl.update(24.0, 0.02)
        assertEquals(0.5, cap, 1e-9)
    }

    @Test
    fun underLimit_recoversAtBoundedRate() {
        val cl = CurrentLimiter(limitAmps = 12.0, recoveryPerSecond = 5.0)
        // Drive the cap down first.
        cl.update(24.0, 0.02) // -> 0.5
        val before = cl.powerCap
        val after = cl.update(6.0, 0.1) // under limit, dt=0.1 -> +0.5
        assertEquals(before + 0.5, after, 1e-9)
    }

    @Test
    fun capNeverLeavesUnitInterval() {
        val cl = CurrentLimiter(limitAmps = 12.0, recoveryPerSecond = 100.0)
        // Huge recovery must clamp at 1.0.
        repeat(10) { cl.update(0.0, 0.1) }
        assertEquals(1.0, cl.powerCap, 0.0)
        // Massive over-current must clamp at >= 0.
        repeat(10) { cl.update(1000.0, 0.02) }
        assertTrue(cl.powerCap >= 0.0)
    }

    @Test
    fun reset_reopensCap() {
        val cl = CurrentLimiter(limitAmps = 12.0)
        cl.update(48.0, 0.02)
        assertTrue(cl.powerCap < 1.0)
        cl.reset()
        assertEquals(1.0, cl.powerCap, 0.0)
    }

    @Test
    fun stallSpikeCaughtInOneLoop_thenHoldsAtLimitDuringSpinup() {
        // Simulate a spin-up with the subsystem's read->update->write ordering: each loop applies the
        // CURRENT cap, the resulting current is drawn, then the governor reacts for the NEXT loop (so
        // the cap lags current by one loop, exposing the real first-loop stall spike). Model:
        // current = ampsPerPower*power - ampsPerSpeed*speed; full-power terminal = 30/0.02 = 1500.
        val limit = 12.0
        val ampsPerPower = 30.0   // stall current at full power = 30 A
        val ampsPerSpeed = 0.02   // back-EMF: current falls as speed rises
        val accelPerAmp = 200.0   // speed units/s per amp of (torque ~ current)
        val dt = 0.01
        val target = 1000.0       // below the 1500 terminal so it is actually reachable
        val cl = CurrentLimiter(limitAmps = limit, recoveryPerSecond = 8.0)

        var speed = 0.0
        var worstSustained = 0.0
        var firstLoop = true
        var loops = 0
        while (speed < target && loops < 2000) {
            val power = 1.0.coerceAtMost(cl.powerCap)                 // velocity law saturates -> 1.0
            val current = currentAt(power, speed, ampsPerPower, ampsPerSpeed)
            if (!firstLoop) worstSustained = max(worstSustained, current) // skip the 1-loop spike
            firstLoop = false
            speed += current * accelPerAmp * dt
            cl.update(current, dt)                                    // governor reacts for next loop
            loops++
        }
        assertTrue("wheel should reach target speed (got $speed)", speed >= target)
        // Past the first loop, current holds within a modest margin of the limit (constant-current
        // spin-up), never near the 30 A stall.
        assertTrue("sustained current $worstSustained exceeded limit margin", worstSustained <= limit * 1.4)
    }

    private fun currentAt(power: Double, speed: Double, ampsPerPower: Double, ampsPerSpeed: Double): Double =
        max(0.0, ampsPerPower * power - ampsPerSpeed * speed)
}

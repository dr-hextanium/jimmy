package org.firstinspires.ftc.teamcode.control

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.random.Random

/**
 * Tests for [FadingMemoryFilter].
 *
 * These assert the filter's *contracts* -- seeds on first sample, converges to a constant, tracks a
 * constant-velocity ramp with zero steady-state lag, actually attenuates noise, rejects spikes but
 * can't wedge, holds on non-positive dt, and is loop-rate (dt) invariant -- rather than pinning exact
 * gain values. Thresholds come from a reference simulation of the same recurrence.
 */
class FadingMemoryFilterTest {
    private val tau = 0.05
    private val dt = 0.01

    private fun std(xs: List<Double>): Double {
        val mean = xs.average()
        return sqrt(xs.sumOf { (it - mean) * (it - mean) } / xs.size)
    }

    @Test
    fun firstUpdateSeedsAtMeasurementWithZeroVelocity() {
        val f = FadingMemoryFilter()
        f.update(dt, 12.34, tau)
        assertEquals("first sample should seed the position", 12.34, f.position, 1e-12)
        assertEquals("first sample should start at rest", 0.0, f.velocity, 1e-12)
    }

    @Test
    fun resetSeedsPositionAndZeroesVelocity() {
        val f = FadingMemoryFilter()
        repeat(20) { f.update(dt, 100.0 * it * dt, tau) } // build up some velocity
        f.reset(7.0)
        assertEquals(7.0, f.position, 1e-12)
        assertEquals(0.0, f.velocity, 1e-12)
    }

    @Test
    fun constantInputConvergesAndVelocityGoesToZero() {
        val f = FadingMemoryFilter()
        f.reset(0.0)
        repeat(100) { f.update(dt, 10.0, tau) }
        assertEquals("should converge to the constant", 10.0, f.position, 1e-3)
        assertEquals("velocity should settle to zero", 0.0, f.velocity, 1e-3)
    }

    @Test
    fun constantVelocityRampIsTrackedWithZeroSteadyStateLag() {
        val f = FadingMemoryFilter()
        f.reset(0.0)
        val rate = 100.0 // deg/s
        var t = 0.0
        repeat(300) {
            t += dt
            f.update(dt, rate * t, tau)
        }
        assertEquals("velocity estimate should match the ramp slope", rate, f.velocity, 1e-2)
        assertEquals("position should track the ramp with no steady-state lag", rate * t, f.position, 1e-2)
    }

    @Test
    fun attenuatesWhiteNoise() {
        val f = FadingMemoryFilter()
        f.reset(0.0)
        val rng = Random(42)
        val raw = ArrayList<Double>()
        val filtered = ArrayList<Double>()
        repeat(4000) {
            val z = rng.nextDouble(-1.0, 1.0) + rng.nextDouble(-1.0, 1.0) // ~triangular noise about 0
            f.update(dt, z, tau)
            raw.add(z); filtered.add(f.position)
        }
        // Discard warm-up, then the filtered signal must be markedly quieter than the raw input.
        val rawStd = std(raw.drop(500))
        val filtStd = std(filtered.drop(500))
        assertTrue("filter should reduce noise std (raw=$rawStd filt=$filtStd)", filtStd < 0.7 * rawStd)
    }

    @Test
    fun rejectsAnIsolatedSpike() {
        val f = FadingMemoryFilter()
        f.reset(5.0)
        repeat(25) { f.update(dt, 5.0, tau, spikeGate = 10.0) } // settle at 5
        val before = f.position
        f.update(dt, 55.0, tau, spikeGate = 10.0) // +50 spike, well over the gate
        assertTrue("a lone spike must barely move the estimate", abs(f.position - before) < 1.0)
    }

    @Test
    fun sustainedStepForcesAcceptanceAndSelfHeals() {
        val f = FadingMemoryFilter()
        f.reset(5.0)
        repeat(20) { f.update(dt, 5.0, tau, spikeGate = 10.0) }
        // A genuine step of +20 (over the gate) must NOT wedge the estimate forever.
        repeat(60) { f.update(dt, 25.0, tau, spikeGate = 10.0, maxConsecutiveRejects = 5) }
        assertEquals("estimate must recover to the stepped value", 25.0, f.position, 2.0)
    }

    @Test
    fun nonPositiveDtHoldsTheEstimate() {
        val f = FadingMemoryFilter()
        f.reset(3.0)
        f.update(0.0, 99.0, tau)
        assertEquals(3.0, f.position, 1e-12)
        f.update(-0.01, 99.0, tau)
        assertEquals(3.0, f.position, 1e-12)
    }

    @Test
    fun isDtInvariant() {
        // Same tau, same total elapsed time and target: one big step vs many small ones should land
        // in nearly the same place (the discount factor is exp(-dt/tau), not a fixed per-sample alpha).
        val coarse = FadingMemoryFilter().apply { reset(0.0) }
        coarse.update(0.5, 10.0, tau)

        val fine = FadingMemoryFilter().apply { reset(0.0) }
        repeat(50) { fine.update(0.01, 10.0, tau) }

        assertEquals(coarse.position, fine.position, 0.05)
    }
}

package org.firstinspires.ftc.teamcode.control

import org.firstinspires.ftc.teamcode.control.TrapezoidalProfile.State
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Tests for the trapezoidal motion profile. These validate the profile's *math contracts* against
 * closed-form kinematics (analytic total times, peak velocities) and its *dynamic invariants*
 * (velocity/acceleration never exceed the limits, the profile always converges to and holds the
 * goal). The reference-governor test exercises the exact incremental pattern the turret uses.
 */
class TrapezoidalProfileTest {
    private val eps = 1e-9

    // Densely sample a one-shot profile and return (times, states).
    private fun sample(p: TrapezoidalProfile, current: State, goal: State, dt: Double = 1e-3):
        List<Pair<Double, State>> {
        val end = p.totalTime(current, goal)
        val out = ArrayList<Pair<Double, State>>()
        var t = 0.0
        while (t <= end + dt) {
            out.add(t to p.calculate(t, current, goal))
            t += dt
        }
        return out
    }

    // ---- analytic total time ----

    @Test
    fun totalTime_trapezoidReachingCruise_matchesClosedForm() {
        // maxVel 10, maxAccel 5. Distance 40 from rest to rest:
        // accel 2 s (covers 10), cruise 20 -> 2 s, decel 2 s (covers 10). Total 6 s.
        val p = TrapezoidalProfile(10.0, 5.0)
        assertEquals(6.0, p.totalTime(State(0.0, 0.0), State(40.0, 0.0)), 1e-9)
    }

    @Test
    fun totalTime_triangularProfile_matchesClosedForm() {
        // maxVel 10, maxAccel 5, distance 10 from rest to rest never reaches cruise:
        // peak v = sqrt(a*D) = sqrt(50), each ramp v/a = sqrt(50)/5, total 2*that.
        val p = TrapezoidalProfile(10.0, 5.0)
        val expected = 2.0 * sqrt(50.0) / 5.0
        assertEquals(expected, p.totalTime(State(0.0, 0.0), State(10.0, 0.0)), 1e-9)
    }

    @Test
    fun totalTime_zeroDistance_isZero() {
        val p = TrapezoidalProfile(10.0, 5.0)
        assertEquals(0.0, p.totalTime(State(3.0, 0.0), State(3.0, 0.0)), 1e-9)
    }

    // ---- endpoints ----

    @Test
    fun calculate_atZero_returnsCurrent() {
        val p = TrapezoidalProfile(10.0, 5.0)
        val s = p.calculate(0.0, State(2.0, 0.0), State(40.0, 0.0))
        assertEquals(2.0, s.position, eps)
        assertEquals(0.0, s.velocity, eps)
    }

    @Test
    fun calculate_atTotalTime_reachesGoalAtRest() {
        val p = TrapezoidalProfile(10.0, 5.0)
        val goal = State(40.0, 0.0)
        val end = p.totalTime(State(0.0, 0.0), goal)
        val s = p.calculate(end, State(0.0, 0.0), goal)
        assertEquals(40.0, s.position, 1e-6)
        assertEquals(0.0, s.velocity, 1e-6)
    }

    @Test
    fun calculate_pastEnd_holdsGoal() {
        val p = TrapezoidalProfile(10.0, 5.0)
        val goal = State(40.0, 0.0)
        val s = p.calculate(1000.0, State(0.0, 0.0), goal)
        assertEquals(40.0, s.position, eps)
        assertEquals(0.0, s.velocity, eps)
    }

    @Test
    fun calculate_negativeTime_clampsToStart() {
        val p = TrapezoidalProfile(10.0, 5.0)
        val s = p.calculate(-5.0, State(2.0, 0.0), State(40.0, 0.0))
        assertEquals(2.0, s.position, eps)
        assertEquals(0.0, s.velocity, eps)
    }

    // ---- kinematic bounds & peak velocities ----

    @Test
    fun trapezoid_reachesCruiseVelocity_andNeverExceedsIt() {
        val p = TrapezoidalProfile(10.0, 5.0)
        val samples = sample(p, State(0.0, 0.0), State(40.0, 0.0))
        val peak = samples.maxOf { it.second.velocity }
        assertEquals("cruise velocity should be reached", 10.0, peak, 1e-6)
        assertTrue("velocity exceeded maxVelocity", samples.all { it.second.velocity <= 10.0 + 1e-6 })
    }

    @Test
    fun triangular_peakVelocityIsBelowCruise_andMatchesClosedForm() {
        val p = TrapezoidalProfile(10.0, 5.0)
        val samples = sample(p, State(0.0, 0.0), State(10.0, 0.0), dt = 1e-4)
        val peak = samples.maxOf { it.second.velocity }
        val expected = sqrt(5.0 * 10.0) // sqrt(a*D)
        assertEquals(expected, peak, 1e-3)
        assertTrue("triangular peak should be below cruise", peak < 10.0)
    }

    @Test
    fun accelerationNeverExceedsLimit_acrossProfile() {
        val p = TrapezoidalProfile(10.0, 5.0)
        val samples = sample(p, State(0.0, 0.0), State(40.0, 0.0), dt = 1e-3)
        for (i in 1 until samples.size) {
            val dv = samples[i].second.velocity - samples[i - 1].second.velocity
            val dt = samples[i].first - samples[i - 1].first
            val accel = abs(dv / dt)
            assertTrue("accel $accel exceeded limit at t=${samples[i].first}", accel <= 5.0 + 1e-3)
        }
    }

    @Test
    fun positionIsMonotonicForRestToRestMove() {
        val p = TrapezoidalProfile(10.0, 5.0)
        val samples = sample(p, State(0.0, 0.0), State(40.0, 0.0))
        for (i in 1 until samples.size) {
            assertTrue(
                "position went backwards at t=${samples[i].first}",
                samples[i].second.position >= samples[i - 1].second.position - 1e-9
            )
        }
    }

    // ---- direction / mirroring ----

    @Test
    fun negativeDirection_mirrorsPositiveCase() {
        val p = TrapezoidalProfile(10.0, 5.0)
        val fwd = State(0.0, 0.0)
        val goalF = State(40.0, 0.0)
        val goalB = State(-40.0, 0.0)
        assertEquals(p.totalTime(fwd, goalF), p.totalTime(fwd, goalB), 1e-12)
        // Sample midway; positions should be exact negatives, velocities exact negatives.
        val t = 2.5
        val sf = p.calculate(t, fwd, goalF)
        val sb = p.calculate(t, fwd, goalB)
        assertEquals(-sf.position, sb.position, 1e-9)
        assertEquals(-sf.velocity, sb.velocity, 1e-9)
    }

    @Test
    fun nonZeroGoalVelocity_arrivesWithThatVelocity() {
        val p = TrapezoidalProfile(10.0, 5.0)
        val goal = State(30.0, 6.0) // fly through at 6 unit/s
        val end = p.totalTime(State(0.0, 0.0), goal)
        val s = p.calculate(end, State(0.0, 0.0), goal)
        assertEquals(30.0, s.position, 1e-6)
        assertEquals(6.0, s.velocity, 1e-6)
    }

    // ---- reference governor (turret usage) ----

    @Test
    fun referenceGovernor_convergesToStaticTargetWithinBounds() {
        val p = TrapezoidalProfile(180.0, 720.0) // deg/s, deg/s^2 -- turret-scale numbers
        val dt = 0.02 // 50 Hz
        var state = State(0.0, 0.0)
        val target = State(90.0, 0.0)
        var maxVel = 0.0
        repeat(2000) {
            val next = p.calculate(dt, state, target)
            val accel = abs(next.velocity - state.velocity) / dt
            assertTrue("governor exceeded accel limit", accel <= 720.0 + 1.0)
            assertTrue("governor exceeded velocity limit", abs(next.velocity) <= 180.0 + 1e-6)
            maxVel = maxOf(maxVel, abs(next.velocity))
            state = next
        }
        assertEquals("did not converge to target", 90.0, state.position, 1e-3)
        assertEquals("did not settle to rest", 0.0, state.velocity, 1e-3)
        assertTrue("should have cruised near max velocity", maxVel > 150.0)
    }

    @Test
    fun referenceGovernor_tracksAMovingTarget() {
        // Target ramps away each loop; the profiled setpoint should follow it without exceeding
        // limits and stay close once the target's rate is below the profile's max velocity.
        val p = TrapezoidalProfile(180.0, 720.0)
        val dt = 0.02
        var state = State(0.0, 0.0)
        var target = 0.0
        repeat(500) {
            target += 1.0 // 50 deg/s ramp, well under 180 deg/s cap
            val next = p.calculate(dt, state, State(target, 0.0))
            assertTrue("velocity exceeded limit while tracking", abs(next.velocity) <= 180.0 + 1e-6)
            state = next
        }
        // After settling, the profiled setpoint should trail the moving target only slightly.
        assertTrue("setpoint fell too far behind a slow target", abs(state.position - target) < 5.0)
    }

    @Test
    fun referenceGovernor_handlesTargetReversalWithinBounds() {
        // Track toward +90, then flip the target to -90 mid-travel. The plan now starts with a
        // velocity opposing the new travel direction; it must decelerate, reverse, and converge
        // without ever breaching the velocity/acceleration limits.
        val p = TrapezoidalProfile(180.0, 720.0)
        val dt = 0.02
        var state = State(0.0, 0.0)
        repeat(20) { state = p.calculate(dt, state, State(90.0, 0.0)) } // build up +velocity
        assertTrue("precondition: should be moving toward +target", state.velocity > 10.0)

        repeat(2000) {
            val next = p.calculate(dt, state, State(-90.0, 0.0))
            val accel = abs(next.velocity - state.velocity) / dt
            assertTrue("reversal breached accel limit", accel <= 720.0 + 1.0)
            assertTrue("reversal breached velocity limit", abs(next.velocity) <= 180.0 + 1e-6)
            state = next
        }
        assertEquals("did not converge after reversal", -90.0, state.position, 1e-3)
        assertEquals("did not settle to rest after reversal", 0.0, state.velocity, 1e-3)
    }

    // ---- validation ----

    @Test(expected = IllegalArgumentException::class)
    fun rejectsNonPositiveMaxVelocity() {
        TrapezoidalProfile(0.0, 5.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun rejectsNonPositiveMaxAcceleration() {
        TrapezoidalProfile(10.0, -1.0)
    }
}

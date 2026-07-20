package org.firstinspires.ftc.teamcode.control

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.sqrt

/**
 * Tests for the second-order pole-placement gain solver. These prove the closed-form maps to the
 * intended error dynamics (round-tripping kP/kD back through wn and the damping identity) and pin
 * the documented behaviours (kD clamp, monotonicity in the settling-time knob). One characterization
 * test checks the gains land in the turret's real ballpark for representative feedforward.
 */
class PolePlacementTest {
    @Test
    fun placesNaturalFrequencyFromSettlingTime() {
        val g = PolePlacement.positionGains(kV = 0.0012, kA = 6e-5, settlingTimeSeconds = 0.2)
        assertEquals(4.0 / 0.2, g.omegaN, 1e-9) // t_s ~= 4/wn
    }

    @Test
    fun gainsSatisfyTheErrorDynamicsIdentities() {
        val kV = 0.0012
        val kA = 6e-5
        val zeta = 1.0
        val g = PolePlacement.positionGains(kV, kA, settlingTimeSeconds = 0.2, dampingRatio = zeta)
        // wn^2 = kP/kA  and  2*zeta*wn = (kV + kD)/kA
        assertEquals(g.omegaN, sqrt(g.kP / kA), 1e-9)
        assertEquals(2.0 * zeta * g.omegaN, (kV + g.kD) / kA, 1e-9)
        assertFalse(g.kdClamped)
    }

    @Test
    fun underdampedProducesLowerKdThanCritical() {
        val kV = 0.0012
        val kA = 6e-5
        val crit = PolePlacement.positionGains(kV, kA, 0.2, dampingRatio = 1.0)
        val under = PolePlacement.positionGains(kV, kA, 0.2, dampingRatio = 0.7)
        assertEquals(crit.kP, under.kP, 1e-12)               // kP depends only on wn
        assertTrue("less damping -> less kD", under.kD < crit.kD)
    }

    @Test
    fun clampsKdToZeroWhenVelocityFeedforwardOverDamps() {
        // Large kV, tiny kA, slow bandwidth -> 2*zeta*wn*kA < kV, so raw kD is negative.
        val g = PolePlacement.positionGains(kV = 0.01, kA = 1e-6, settlingTimeSeconds = 1.0)
        assertEquals(0.0, g.kD, 0.0)
        assertTrue(g.kdClamped)
    }

    @Test
    fun fasterSettlingRaisesKp() {
        val kV = 0.0012
        val kA = 6e-5
        val slow = PolePlacement.positionGains(kV, kA, 0.4)
        val fast = PolePlacement.positionGains(kV, kA, 0.1)
        assertTrue("shorter settling time -> stiffer kP", fast.kP > slow.kP)
    }

    @Test
    fun characterization_landsInTurretBallpark() {
        // With a plausible identified plant (kA ~ 6e-5, kV ~ 0.0012) and a ~0.16 s settling target,
        // kP should land near the turret's hand-tuned 0.038 -- a coarse sanity band, not an exact
        // match. If this drifts, suspect the kA fit before the formula.
        val g = PolePlacement.positionGains(kV = 0.0012, kA = 6e-5, settlingTimeSeconds = 0.16)
        assertTrue("kP=${g.kP} outside plausible band", g.kP in 0.02..0.06)
    }

    // ---- first-order velocity-loop gain (launcher flywheel) ----

    @Test
    fun velocityGain_satisfiesFirstOrderIdentity() {
        val kV = 0.0004
        val kA = 0.0002 // tau_openLoop = kA/kV = 0.5 s
        val g = PolePlacement.velocityLoopGain(kV, kA, closedLoopTau = 0.4)
        // tau_cl = kA/(kV+kP)  ⇒  kP = kA/tau_cl − kV
        assertEquals(kA / 0.4 - kV, g.kP, 1e-12)
        assertEquals(kA / kV, g.openLoopTau, 1e-12)
        assertFalse(g.kpCapped)
    }

    @Test
    fun velocityGain_fasterClosedLoopRaisesKp() {
        val kV = 0.0004
        val kA = 0.0002
        val slow = PolePlacement.velocityLoopGain(kV, kA, 0.45)
        val fast = PolePlacement.velocityLoopGain(kV, kA, 0.30)
        assertTrue("shorter closed-loop tau -> larger kP", fast.kP > slow.kP)
    }

    @Test
    fun velocityGain_capsKpToStayFeedforwardDominant() {
        // Aggressive closed loop (tau_cl << tau_openLoop) would push kP well past kV; the default
        // cap (kP <= kV) holds it, keeping the loop feedforward-dominant.
        val kV = 0.0004
        val kA = 0.0002 // tau_openLoop = 0.5 s
        val g = PolePlacement.velocityLoopGain(kV, kA, closedLoopTau = 0.05)
        assertTrue(g.kpCapped)
        assertEquals(kV, g.kP, 1e-12)
    }

    @Test
    fun velocityGain_clampsKpToZeroWhenClosedLoopSlowerThanOpenLoop() {
        // closedLoopTau > openLoopTau would imply a negative kP (slowing the loop below the plant);
        // clamp to 0 -- pure feedforward.
        val kV = 0.0004
        val kA = 0.0002 // tau_openLoop = 0.5 s
        val g = PolePlacement.velocityLoopGain(kV, kA, closedLoopTau = 0.8)
        assertEquals(0.0, g.kP, 0.0)
    }

    @Test
    fun characterization_velocityGainLandsNearLauncherDefault() {
        // Plausible flywheel plant + a conservative 1.5x speed-up (tau_cl = tau_openLoop/1.5) should
        // land kP near the launcher's hand-picked 0.0003. Coarse sanity band, not an exact match.
        val kV = 0.0004
        val kA = 0.00016 // tau_openLoop = 0.4 s
        val g = PolePlacement.velocityLoopGain(kV, kA, closedLoopTau = 0.4 / 1.5)
        assertTrue("kP=${g.kP} outside plausible band", g.kP in 0.0001..0.0006)
    }

    @Test(expected = IllegalArgumentException::class)
    fun velocityGain_rejectsNonPositiveKv() {
        PolePlacement.velocityLoopGain(kV = 0.0, kA = 0.0002, closedLoopTau = 0.4)
    }

    @Test(expected = IllegalArgumentException::class)
    fun rejectsNonPositiveKa() {
        PolePlacement.positionGains(kV = 0.001, kA = 0.0, settlingTimeSeconds = 0.2)
    }

    @Test(expected = IllegalArgumentException::class)
    fun rejectsNonPositiveSettlingTime() {
        PolePlacement.positionGains(kV = 0.001, kA = 6e-5, settlingTimeSeconds = 0.0)
    }
}

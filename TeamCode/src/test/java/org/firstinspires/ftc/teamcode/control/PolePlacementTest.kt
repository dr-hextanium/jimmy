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

    @Test(expected = IllegalArgumentException::class)
    fun rejectsNonPositiveKa() {
        PolePlacement.positionGains(kV = 0.001, kA = 0.0, settlingTimeSeconds = 0.2)
    }

    @Test(expected = IllegalArgumentException::class)
    fun rejectsNonPositiveSettlingTime() {
        PolePlacement.positionGains(kV = 0.001, kA = 6e-5, settlingTimeSeconds = 0.0)
    }
}

package org.firstinspires.ftc.teamcode.control

import org.firstinspires.ftc.teamcode.control.ProjectileSolver.AeroParams
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.cos

/**
 * Tests for the projectile physics core. These check the closed-form solver against known
 * kinematics and cross-check it against the numerical integrator. Because the model is derived
 * from physics rather than fitted, the assertions are exact invariants (not tuning tolerances):
 * a solved trajectory must actually pass through its target, and the integrator must collapse onto
 * the closed form when aerodynamics are off.
 */
class ProjectileSolverTest {
    private val g = ProjectileSolver.G

    // ---- constants / setup sanity ----

    @Test
    fun gravityIsSiValue() {
        assertEquals(9.81, ProjectileSolver.G, 0.0)
    }

    // ---- minimum-speed angle ----

    @Test
    fun minSpeedAngle_isFortyFiveForLevelTarget() {
        assertEquals(PI / 4.0, ProjectileSolver.minimumSpeedAngle(5.0, 0.0), 1e-12)
    }

    @Test
    fun minSpeedAngle_approachesVerticalForTargetOverhead() {
        // Target nearly straight up: elevation -> 90deg, min-speed angle -> 90deg.
        val theta = ProjectileSolver.minimumSpeedAngle(1e-6, 5.0)
        assertTrue("expected near-vertical launch, got $theta", theta > Math.toRadians(89.0))
    }

    @Test
    fun minSpeedAngle_reachesTargetAndIsGloballyMinimal() {
        val d = 4.0
        val h = 1.5
        val thetaStar = ProjectileSolver.minimumSpeedAngle(d, h)
        val vStar = ProjectileSolver.solveExitSpeed(d, h, thetaStar)
        // Sweep other reachable angles; none should need a smaller exit speed.
        var theta = Math.toRadians(5.0)
        while (theta < Math.toRadians(89.0)) {
            val v = ProjectileSolver.solveExitSpeed(d, h, theta)
            if (!v.isNaN()) {
                assertTrue("angle ${Math.toDegrees(theta)} beat the min-speed angle", v >= vStar - 1e-6)
            }
            theta += Math.toRadians(1.0)
        }
    }

    // ---- reachability ----

    @Test
    fun reachability_shallowAngleUnderTargetIsUnreachable() {
        // Target high and close; a shallow launch line stays below it.
        assertFalse(ProjectileSolver.isReachable(d = 2.0, h = 3.0, theta = Math.toRadians(10.0)))
        assertTrue(ProjectileSolver.solveExitSpeed(2.0, 3.0, Math.toRadians(10.0)).isNaN())
    }

    @Test
    fun reachability_zeroOrNegativeDistanceIsUnreachable() {
        assertFalse(ProjectileSolver.isReachable(0.0, 0.0, PI / 4.0))
    }

    // ---- closed-form solve: hit the target ----

    @Test
    fun solve_levelTargetAtFortyFive_matchesHandComputation() {
        // d=10, theta=45, h=0: v^2 = g*100/(2*0.5*10) = 10g -> v = sqrt(10g).
        val v = ProjectileSolver.solveExitSpeed(10.0, 0.0, PI / 4.0)
        assertEquals(kotlin.math.sqrt(10.0 * g), v, 1e-9)
    }

    @Test
    fun solve_producedSpeedActuallyPassesThroughTargetHeight() {
        // For a grid of reachable targets/angles, the closed-form height at d must equal h.
        for (d in listOf(1.0, 2.5, 4.0)) {
            for (h in listOf(-0.5, 0.0, 0.8, 1.5)) {
                for (deg in listOf(30.0, 45.0, 60.0, 75.0)) {
                    val theta = Math.toRadians(deg)
                    val v = ProjectileSolver.solveExitSpeed(d, h, theta)
                    if (v.isNaN()) continue
                    assertEquals(
                        "closed-form trajectory missed target d=$d h=$h deg=$deg",
                        h, ProjectileSolver.closedFormHeight(v, theta, d), 1e-9
                    )
                }
            }
        }
    }

    @Test
    fun solve_requiredSpeedIncreasesWithDistanceAtFixedAngle() {
        var prev = 0.0
        var d = 1.0
        while (d <= 6.0) {
            val v = ProjectileSolver.solveExitSpeed(d, 0.0, PI / 4.0)
            assertTrue("speed should grow with distance at d=$d", v > prev)
            prev = v
            d += 0.5
        }
    }

    // ---- horizontal speed / time of flight ----

    @Test
    fun timeOfFlight_carriesBallExactlyToDistance() {
        val d = 3.0
        val theta = Math.toRadians(55.0)
        val v = ProjectileSolver.solveExitSpeed(d, 1.0, theta)
        val tof = ProjectileSolver.timeOfFlight(d, v, theta)
        // x(t) = v*cos(theta)*t must equal d at the time of flight.
        assertEquals(d, v * cos(theta) * tof, 1e-9)
    }

    // ---- numerical integrator: reduces to closed form, and aero acts as expected ----

    @Test
    fun integrator_withNoAero_matchesClosedFormExactly() {
        for (d in listOf(1.0, 3.0, 5.0)) {
            for (deg in listOf(35.0, 50.0, 70.0)) {
                val theta = Math.toRadians(deg)
                val v = ProjectileSolver.solveExitSpeed(d, 0.5, theta)
                if (v.isNaN()) continue
                // RK4 is exact for constant acceleration; residual is the final-step crossing
                // interpolation, which shrinks with dt. A fine dt makes the collapse near-exact.
                val numeric = ProjectileSolver.heightAtDistance(v, theta, d, AeroParams.NONE, dt = 1e-4)
                val closed = ProjectileSolver.closedFormHeight(v, theta, d)
                assertEquals("RK4 diverged from closed form d=$d deg=$deg", closed, numeric, 1e-7)
            }
        }
    }

    @Test
    fun integrator_solvedSpeedLandsOnTargetWithNoAero() {
        val d = 4.0
        val h = 1.2
        val theta = Math.toRadians(60.0)
        val v = ProjectileSolver.solveExitSpeed(d, h, theta)
        assertEquals(h, ProjectileSolver.heightAtDistance(v, theta, d, AeroParams.NONE), 1e-5)
    }

    @Test
    fun integrator_dragLowersTrajectory_backspinRaisesIt() {
        val d = 4.0
        val theta = Math.toRadians(55.0)
        val v = ProjectileSolver.solveExitSpeed(d, 0.0, theta) // lands at h=0 with no aero
        val noAero = ProjectileSolver.heightAtDistance(v, theta, d, AeroParams.NONE)
        val withDrag = ProjectileSolver.heightAtDistance(v, theta, d, AeroParams(drag = 0.05))
        val withBackspin = ProjectileSolver.heightAtDistance(v, theta, d, AeroParams(drag = 0.05, magnus = 0.3))
        assertTrue("drag should lower the trajectory at range", withDrag < noAero)
        assertTrue("backspin lift should raise it back up", withBackspin > withDrag)
    }
}

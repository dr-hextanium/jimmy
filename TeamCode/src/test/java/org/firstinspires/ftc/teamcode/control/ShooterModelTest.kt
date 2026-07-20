package org.firstinspires.ftc.teamcode.control

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * Tests for the kinematic [ShooterModel].
 *
 * Per the suite's philosophy these assert *physics invariants*, not the tuned coefficient values:
 * the exit-speed<->TPS conversion is an exact inverse pair, required speed grows with distance, a
 * "reachable" solution's ball actually passes through the goal, outputs stay within the hood/servo
 * limits, and the tunables move the model in the physically-correct direction. [ShooterModel] is a
 * mutable @Configurable singleton, so every field it touches is reset to a known default here.
 */
class ShooterModelTest {
    private val i2m = ShooterModel.INCHES_TO_METERS

    @Before
    fun setUp() {
        ShooterModel.FLYWHEEL_DIAMETER_MM = 72.0
        ShooterModel.COUNTER_ROLLER_DIAMETER_MM = 28.0
        ShooterModel.COUNTER_ROLLER_GEAR_RATIO = 2.0
        ShooterModel.LAUNCHER_TICKS_PER_REV = 28.0
        ShooterModel.SLIP_EFFICIENCY = 0.85
        ShooterModel.TARGET_HEIGHT_DELTA_M = 0.5
        ShooterModel.HOOD_MIN_ANGLE_RAD = Math.toRadians(30.0)
        ShooterModel.HOOD_MAX_ANGLE_RAD = Math.toRadians(60.0)
        ShooterModel.SERVO_AT_MIN_ANGLE = 0.25
        ShooterModel.SERVO_AT_MAX_ANGLE = 0.905
        ShooterModel.MAX_TPS = 2500.0
    }

    // ---- exit-speed <-> TPS conversion ----

    @Test
    fun exitSpeedPerTps_isPositive() {
        assertTrue(ShooterModel.exitSpeedPerTps() > 0.0)
    }

    @Test
    fun exitSpeedAndTps_areExactInverses() {
        for (tps in listOf(0.0, 250.0, 1000.0, 2500.0)) {
            assertEquals(tps, ShooterModel.tpsForExitSpeed(ShooterModel.exitSpeedFromTps(tps)), 1e-6)
        }
    }

    @Test
    fun exitSpeedPerTps_matchesMeanContactRadiusDerivation() {
        // v/tps = SLIP * (2pi/ticks) * 0.5 * (r_flywheel + gearRatio*r_counter), radii in metres.
        val rFly = 0.072 / 2.0
        val rCtr = 0.028 / 2.0
        val expected = 0.85 * (2.0 * Math.PI / 28.0) * 0.5 * (rFly + 2.0 * rCtr)
        assertEquals(expected, ShooterModel.exitSpeedPerTps(), 1e-12)
    }

    // ---- aiming: required speed grows with distance; reachable shots hit the goal ----

    @Test
    fun requiredTps_increasesWithDistance_whileUnclamped() {
        var prev = -1.0
        var inches = 40.0
        while (inches <= 200.0) {
            val sol = ShooterModel.aim(inches)
            assertTrue("expected an unclamped, reachable solution at $inches in", sol.reachable)
            assertTrue("TPS should grow with distance at $inches in", sol.targetTps > prev)
            prev = sol.targetTps
            inches += 10.0
        }
    }

    @Test
    fun reachableSolution_launchesBallThroughTheGoal() {
        for (inches in listOf(50.0, 80.0, 120.0, 180.0)) {
            val sol = ShooterModel.aim(inches)
            assertTrue("precondition: $inches in should be reachable", sol.reachable)
            val v = ShooterModel.exitSpeedFromTps(sol.targetTps)
            val height = ProjectileSolver.closedFormHeight(v, sol.launchAngleRad, inches * i2m)
            assertEquals(
                "solved shot missed the goal height at $inches in",
                ShooterModel.TARGET_HEIGHT_DELTA_M, height, 1e-6
            )
        }
    }

    @Test
    fun launchAngle_staysWithinHoodRange() {
        var inches = 20.0
        while (inches <= 400.0) {
            val theta = ShooterModel.launchAngleForDistance(inches)
            assertTrue(
                "launch angle out of hood range at $inches in",
                theta >= ShooterModel.HOOD_MIN_ANGLE_RAD - 1e-12 &&
                    theta <= ShooterModel.HOOD_MAX_ANGLE_RAD + 1e-12
            )
            inches += 20.0
        }
    }

    @Test
    fun hoodServo_staysWithinCalibratedTravelAndUnitInterval() {
        val lo = minOf(ShooterModel.SERVO_AT_MIN_ANGLE, ShooterModel.SERVO_AT_MAX_ANGLE)
        val hi = maxOf(ShooterModel.SERVO_AT_MIN_ANGLE, ShooterModel.SERVO_AT_MAX_ANGLE)
        var inches = 20.0
        while (inches <= 400.0) {
            val pos = ShooterModel.aim(inches).hoodServoPosition
            assertTrue("servo $pos outside [0,1] at $inches in", pos in 0.0..1.0)
            assertTrue("servo $pos outside calibrated travel at $inches in", pos in lo..hi)
            inches += 20.0
        }
    }

    @Test
    fun angleToServo_mapsCalibrationEndpoints() {
        assertEquals(0.25, ShooterModel.angleToServo(ShooterModel.HOOD_MIN_ANGLE_RAD), 1e-9)
        assertEquals(0.905, ShooterModel.angleToServo(ShooterModel.HOOD_MAX_ANGLE_RAD), 1e-9)
    }

    // ---- clamp / unreachable behaviour ----

    @Test
    fun farBeyondRange_clampsTpsToMaxAndReportsUnreachable() {
        val sol = ShooterModel.aim(5000.0)
        assertEquals(ShooterModel.MAX_TPS, sol.targetTps, 1e-9)
        assertFalse("a shot needing more than MAX_TPS must report unreachable", sol.reachable)
    }

    // ---- horizontal exit speed / time-of-flight for the turret ----

    @Test
    fun horizontalExitSpeed_isPositiveAndConsistentWithSolution() {
        val inches = 120.0
        val sol = ShooterModel.aim(inches)
        val v = ShooterModel.exitSpeedFromTps(sol.targetTps)
        val expectedInPerSec = ProjectileSolver.horizontalSpeed(v, sol.launchAngleRad) / i2m
        val actual = ShooterModel.horizontalExitSpeedInchesPerSec(inches)
        assertTrue(actual > 0.0)
        assertEquals(expectedInPerSec, actual, 1e-9)
    }

    @Test
    fun horizontalExitSpeed_matchesAimReferenceAcrossDistanceSweep() {
        // The inlined hot-path implementation must equal the aim()-based reference bit-for-bit, across a
        // fine sweep AND at a deep-clamp/unreachable distance (5000 in -> targetTps clamped to MAX_TPS).
        val distances = (20..400 step 5).map { it.toDouble() } + listOf(5000.0)
        for (d in distances) {
            val sol = ShooterModel.aim(d)
            val expected = ProjectileSolver.horizontalSpeed(
                ShooterModel.exitSpeedFromTps(sol.targetTps), sol.launchAngleRad
            ) / i2m
            assertEquals(
                "horizontalExitSpeed mismatch at $d in",
                expected, ShooterModel.horizontalExitSpeedInchesPerSec(d), 1e-12
            )
        }
    }

    // ---- tunable sensitivity (physical directions) ----

    @Test
    fun higherSlipEfficiency_yieldsMoreExitSpeedPerTps() {
        val low = ShooterModel.exitSpeedPerTps()
        ShooterModel.SLIP_EFFICIENCY = 0.95
        assertTrue(ShooterModel.exitSpeedPerTps() > low)
    }

    @Test
    fun moreTicksPerRev_yieldsLessExitSpeedPerTps() {
        val base = ShooterModel.exitSpeedPerTps()
        ShooterModel.LAUNCHER_TICKS_PER_REV = 56.0
        assertTrue(ShooterModel.exitSpeedPerTps() < base)
    }
}

package org.firstinspires.ftc.teamcode.hardware

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI

/**
 * Guards the reset-pose heading units and the goal-pose cartesian construction.
 *
 * The reset-pose test is a regression guard for the toDegrees -> toRadians fix: before it, the
 * headings were ~2119 / ~8193 (radians interpreted from a degrees value), i.e. nonsense. Pose
 * heading is radians, so a sane field heading must be within +/- 2*PI.
 */
class GlobalsTest {
    private val eps = 1e-9

    @Test
    fun resetPoseHeadingsAreSaneRadians() {
        assertTrue(
            "RED reset heading not in radian range: ${Globals.RED_RESET_POSE.heading}",
            Math.abs(Globals.RED_RESET_POSE.heading) <= 2 * PI
        )
        assertTrue(
            "BLUE reset heading not in radian range: ${Globals.BLUE_RESET_POSE.heading}",
            Math.abs(Globals.BLUE_RESET_POSE.heading) <= 2 * PI
        )
        assertEquals(Math.toRadians(37.0), Globals.RED_RESET_POSE.heading, eps)
        assertEquals(Math.toRadians(143.0), Globals.BLUE_RESET_POSE.heading, eps)
    }

    @Test
    fun goalPosesAreCartesian() {
        // Must be built with the cartesian Vector(Pose) ctor, not the polar Vector(mag, theta) one.
        assertEquals(144.0, Globals.RED_GOAL_POSE.xComponent, eps)
        assertEquals(144.0, Globals.RED_GOAL_POSE.yComponent, eps)
        assertEquals(0.0, Globals.BLUE_GOAL_POSE.xComponent, eps)
        assertEquals(144.0, Globals.BLUE_GOAL_POSE.yComponent, eps)
    }
}

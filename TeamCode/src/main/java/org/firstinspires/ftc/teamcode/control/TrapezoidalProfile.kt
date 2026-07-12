package org.firstinspires.ftc.teamcode.control

import kotlin.math.sqrt

/**
 * Stateless trapezoidal motion profile over (position, velocity) states, constrained by a maximum
 * velocity and acceleration. This is a port of WPILib's well-tested TrapezoidProfile algorithm,
 * which handles arbitrary (possibly non-zero) initial and goal velocities and either direction of
 * travel by planning the profile in a canonical +position frame and mirroring the result back.
 *
 * Two ways it is used here:
 *  - One-shot plan: [totalTime] gives the duration; [calculate] samples the profiled state at any
 *    time from the start.
 *  - Reference governor (how the turret drives it): each control loop, call
 *    `calculate(dt, lastProfiledState, State(target, 0.0))`. The profiled setpoint accelerates
 *    toward the target, cruises at [maxVelocity], and decelerates to a stop on it, re-planning every
 *    loop from where it actually is. A target that moves (goal-lock tracking a driving robot) is
 *    then handled exactly like a large step command, with velocity and acceleration always bounded.
 */
class TrapezoidalProfile(val maxVelocity: Double, val maxAcceleration: Double) {
    init {
        require(maxVelocity > 0.0) { "maxVelocity must be > 0, was $maxVelocity" }
        require(maxAcceleration > 0.0) { "maxAcceleration must be > 0, was $maxAcceleration" }
    }

    /** A point on the profile: [position] in the profiled unit, [velocity] in unit/second. */
    data class State(val position: Double = 0.0, val velocity: Double = 0.0)

    // Timing breakpoints for one planned move, computed in the canonical +position frame.
    private class Timing(
        val direction: Double,
        val initialPos: Double,
        val initialVel: Double,
        val goalPos: Double,
        val goalVel: Double,
        val endAccel: Double,
        val endFullSpeed: Double,
        val endDeccel: Double,
    )

    private fun plan(current: State, goal: State): Timing {
        // Plan in a frame where travel is toward +position, then mirror the result back.
        val direction = if (current.position > goal.position) -1.0 else 1.0

        val initPos = current.position * direction
        var initVel = current.velocity * direction
        val goalPos = goal.position * direction
        val goalVel = goal.velocity * direction

        if (initVel > maxVelocity) initVel = maxVelocity

        // Treat the (possibly truncated) profile as a full trapezoid that begins and ends at rest,
        // by adding back the phantom accel/decel segments the non-zero endpoint velocities imply.
        val cutoffBegin = initVel / maxAcceleration
        val cutoffDistBegin = cutoffBegin * cutoffBegin * maxAcceleration / 2.0

        val cutoffEnd = goalVel / maxAcceleration
        val cutoffDistEnd = cutoffEnd * cutoffEnd * maxAcceleration / 2.0

        val fullTrapezoidDist = cutoffDistBegin + (goalPos - initPos) + cutoffDistEnd
        var accelerationTime = maxVelocity / maxAcceleration
        var fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * maxAcceleration

        // Never reaches cruise velocity -> triangular profile.
        if (fullSpeedDist < 0) {
            accelerationTime = sqrt(fullTrapezoidDist / maxAcceleration)
            fullSpeedDist = 0.0
        }

        val endAccel = accelerationTime - cutoffBegin
        val endFullSpeed = endAccel + fullSpeedDist / maxVelocity
        val endDeccel = endFullSpeed + accelerationTime - cutoffEnd

        return Timing(direction, initPos, initVel, goalPos, goalVel, endAccel, endFullSpeed, endDeccel)
    }

    /** The profiled state at time [t] seconds after the move from [current] to [goal] begins. */
    fun calculate(t: Double, current: State, goal: State): State {
        val tm = plan(current, goal)
        val tt = t.coerceAtLeast(0.0)

        val pos: Double
        val vel: Double

        when {
            tt < tm.endAccel -> {
                vel = tm.initialVel + tt * maxAcceleration
                pos = tm.initialPos + (tm.initialVel + tt * maxAcceleration / 2.0) * tt
            }
            tt < tm.endFullSpeed -> {
                vel = maxVelocity
                pos = tm.initialPos +
                    (tm.initialVel + tm.endAccel * maxAcceleration / 2.0) * tm.endAccel +
                    maxVelocity * (tt - tm.endAccel)
            }
            tt <= tm.endDeccel -> {
                val timeLeft = tm.endDeccel - tt
                vel = tm.goalVel + timeLeft * maxAcceleration
                pos = tm.goalPos - (tm.goalVel + timeLeft * maxAcceleration / 2.0) * timeLeft
            }
            else -> {
                vel = tm.goalVel
                pos = tm.goalPos
            }
        }

        return State(pos * tm.direction, vel * tm.direction)
    }

    /** Total time, in seconds, to travel from [current] to [goal] under the constraints. */
    fun totalTime(current: State, goal: State): Double = plan(current, goal).endDeccel
}

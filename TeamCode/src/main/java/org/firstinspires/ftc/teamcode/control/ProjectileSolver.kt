package org.firstinspires.ftc.teamcode.control

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * Pure point-mass projectile kinematics, in SI units (metres, m/s, radians; g in m/s^2). This is
 * the physics core the shooter uses instead of empirical regressions: given how far and how high
 * the target is and the launch angle, it solves the exact exit speed required to pass through it.
 *
 * The closed-form solver ([solveExitSpeed]) is drag-free and is what the robot uses in its control
 * loop -- cheap, exact, and having no coefficients to tune. An optional numerical integrator
 * ([heightAtDistance]) models quadratic drag and Magnus lift for offline analysis; it is OFF by
 * default ([AeroParams.NONE]) and provably reduces to the closed form when both coefficients are
 * zero (see the self-consistency tests). Do not put the integrator in the hot control loop.
 *
 * Coordinate frame: launch from the origin, +x downrange (horizontal), +y up. The target is at
 * horizontal distance `d > 0` and height `h` (positive = above the launch point).
 */
object ProjectileSolver {
    /** Gravitational acceleration, m/s^2. */
    const val G = 9.81

    private const val EPS = 1e-9

    /**
     * Launch angle (rad) that reaches the target `(d, h)` with the least possible exit speed.
     * Classic result: it bisects vertical and the line of sight to the target,
     * `theta* = 45deg + elevation/2`, where `elevation = atan2(h, d)`.
     */
    fun minimumSpeedAngle(d: Double, h: Double): Double = Math.PI / 4.0 + 0.5 * atan2(h, d)

    /**
     * Whether a projectile launched at [theta] can reach `(d, h)` at all. Requires a forward,
     * upward-of-horizontal launch (`0 < theta < 90deg`) and that the straight ballistic line at
     * [theta] clears the target height (`d*tan(theta) > h`); otherwise no positive exit speed hits.
     */
    fun isReachable(d: Double, h: Double, theta: Double): Boolean {
        if (d <= EPS) return false
        val c = cos(theta)
        if (c <= EPS) return false
        return d * tan(theta) - h > EPS
    }

    /**
     * Exit speed (m/s) required to pass through `(d, h)` at launch angle [theta], from
     * `v^2 = g*d^2 / (2*cos^2(theta)*(d*tan(theta) - h))`. Returns [Double.NaN] if the target is
     * unreachable at that angle (see [isReachable]).
     */
    fun solveExitSpeed(d: Double, h: Double, theta: Double): Double {
        if (!isReachable(d, h, theta)) return Double.NaN
        val c = cos(theta)
        val denom = 2.0 * c * c * (d * tan(theta) - h)
        return sqrt(G * d * d / denom)
    }

    /** Horizontal component of the exit velocity (m/s): `v * cos(theta)`. */
    fun horizontalSpeed(v: Double, theta: Double): Double = v * cos(theta)

    /**
     * Time of flight (s) to reach horizontal distance [d] at exit speed [v] and angle [theta].
     * Drag-free: horizontal speed is constant, so `t = d / (v*cos(theta))`.
     */
    fun timeOfFlight(d: Double, v: Double, theta: Double): Double = d / horizontalSpeed(v, theta)

    /**
     * Drag-free height (m) of the trajectory at horizontal distance [d]:
     * `d*tan(theta) - g*d^2 / (2*v^2*cos^2(theta))`. Used to validate the numerical integrator.
     */
    fun closedFormHeight(v: Double, theta: Double, d: Double): Double {
        val c = cos(theta)
        return d * tan(theta) - G * d * d / (2.0 * v * v * c * c)
    }

    /**
     * Aerodynamic coefficients for the optional numerical model. Both fold their physical constants
     * (air density, cross-section, mass, spin rate) into a single tunable each; [NONE] disables the
     * model, recovering the closed-form parabola exactly.
     *
     * @param drag quadratic-drag coefficient (1/m): drag acceleration is `-drag * speed * velocity`.
     * @param magnus Magnus/lift coefficient (1/s): lift acceleration is `magnus * (-vy, vx)`,
     *   i.e. velocity rotated +90deg, so a positive value models backspin lift.
     */
    data class AeroParams(val drag: Double = 0.0, val magnus: Double = 0.0) {
        companion object {
            val NONE = AeroParams(0.0, 0.0)
        }
    }

    /**
     * Trajectory height (m) at horizontal distance [d] for a launch at speed [v], angle [theta],
     * under [aero], integrated with RK4. With [AeroParams.NONE] the dynamics are constant-
     * acceleration and RK4 is exact, so this equals [closedFormHeight] to numerical precision.
     * Returns [Double.NaN] if the projectile never reaches [d] (e.g. drag stalls it out).
     */
    fun heightAtDistance(
        v: Double,
        theta: Double,
        d: Double,
        aero: AeroParams = AeroParams.NONE,
        dt: Double = 1e-3,
        maxTime: Double = 30.0,
    ): Double {
        // state = [x, y, vx, vy]
        var s = doubleArrayOf(0.0, 0.0, horizontalSpeed(v, theta), v * kotlin.math.sin(theta))
        var t = 0.0
        while (t < maxTime) {
            if (s[0] >= d) return s[1]
            val prev = s
            s = rk4Step(s, dt, aero)
            t += dt
            // Crossed x = d during this step: linearly interpolate y by x.
            if (prev[0] < d && s[0] >= d) {
                val frac = (d - prev[0]) / (s[0] - prev[0])
                return prev[1] + frac * (s[1] - prev[1])
            }
        }
        return Double.NaN
    }

    private fun derivative(s: DoubleArray, aero: AeroParams): DoubleArray {
        val vx = s[2]
        val vy = s[3]
        val speed = hypot(vx, vy)
        val ax = -aero.drag * speed * vx + aero.magnus * (-vy)
        val ay = -G - aero.drag * speed * vy + aero.magnus * vx
        return doubleArrayOf(vx, vy, ax, ay)
    }

    private fun rk4Step(s: DoubleArray, dt: Double, aero: AeroParams): DoubleArray {
        val k1 = derivative(s, aero)
        val k2 = derivative(add(s, k1, dt / 2.0), aero)
        val k3 = derivative(add(s, k2, dt / 2.0), aero)
        val k4 = derivative(add(s, k3, dt), aero)
        val out = DoubleArray(4)
        for (i in 0..3) out[i] = s[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
        return out
    }

    private fun add(s: DoubleArray, k: DoubleArray, scale: Double): DoubleArray {
        val out = DoubleArray(4)
        for (i in 0..3) out[i] = s[i] + k[i] * scale
        return out
    }
}

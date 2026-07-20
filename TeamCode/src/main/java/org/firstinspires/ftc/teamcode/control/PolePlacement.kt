package org.firstinspires.ftc.teamcode.control

/**
 * Second-order pole placement for the turret's profiled feedforward+PID law.
 *
 * With the feedforward (kV, kA, kStatic) cancelling the plant along the profiled trajectory, the
 * position-error `e = profiledPosition - currentAngle` obeys a clean second-order ODE:
 *
 *     kA*e'' + (kV + kD)*e' + kP*e = 0
 *
 * i.e. a mass-spring-damper. Matching it to the canonical form `e'' + 2*zeta*wn*e' + wn^2*e = 0`:
 *
 *     wn^2       = kP / kA
 *     2*zeta*wn  = (kV + kD) / kA
 *
 * Inverting for a chosen closed-loop bandwidth -- expressed as an approximate settling time
 * `t_s ~= 4 / wn` -- and damping ratio (1.0 = critically damped, no overshoot):
 *
 *     kP = kA * wn^2
 *     kD = 2*zeta*wn*kA - kV
 *
 * Deterministic, closed-form, no iteration -- so it is trivially unit-testable. (Chosen over an LQR
 * port, which yields the same family of gains from cost weights but is far harder to validate.)
 *
 * kD is clamped at zero: if `2*zeta*wn*kA < kV`, the velocity feedforward alone already over-damps
 * the error dynamics, and a negative kD would be meaningless for the control law. A clamp firing
 * means the plant is naturally sluggish relative to the requested bandwidth -- not an error.
 */
object PolePlacement {
    /** Feedback gains plus the natural frequency they were placed at, for telemetry/diagnostics. */
    data class Gains(val kP: Double, val kD: Double, val omegaN: Double, val kdClamped: Boolean)

    /**
     * @param kV velocity feedforward (power per deg/s), from [FeedforwardFit.fitSteadyState].
     * @param kA acceleration feedforward (power per deg/s^2), from [FeedforwardFit.fitTimeConstant].
     * @param settlingTimeSeconds target closed-loop settling time; smaller = stiffer/faster.
     * @param dampingRatio 1.0 (default) is critically damped (fastest with no overshoot).
     */
    fun positionGains(
        kV: Double,
        kA: Double,
        settlingTimeSeconds: Double,
        dampingRatio: Double = 1.0,
    ): Gains {
        require(kA > 0.0) { "kA must be > 0, was $kA" }
        require(settlingTimeSeconds > 0.0) { "settlingTimeSeconds must be > 0, was $settlingTimeSeconds" }
        require(dampingRatio > 0.0) { "dampingRatio must be > 0, was $dampingRatio" }

        val omegaN = 4.0 / settlingTimeSeconds
        val kP = kA * omegaN * omegaN
        val kdRaw = 2.0 * dampingRatio * omegaN * kA - kV
        val kD = kdRaw.coerceAtLeast(0.0)
        return Gains(kP, kD, omegaN, kdClamped = kdRaw < 0.0)
    }
}

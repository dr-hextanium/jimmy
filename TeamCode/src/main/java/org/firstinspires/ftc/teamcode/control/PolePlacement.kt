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

    /** First-order velocity-loop gain plus the time constants, for telemetry/diagnostics. */
    data class VelocityGain(
        val kP: Double,
        val openLoopTau: Double,
        val closedLoopTau: Double,
        val kpCapped: Boolean,
    )

    /**
     * First-order velocity-loop gain for a feedforward+P controller `u = kS + kV*r + kP*(r - v)` on a
     * plant `kA*v̇ = u - kS - kV*v` (with `kA = tau_openLoop * kV`, the electromechanical time
     * constant). Substituting the control law makes the error `e = r - v` first-order:
     *
     *     kA*ė + (kV + kP)*e = 0    ⇒    tau_closedLoop = kA / (kV + kP)
     *
     * so for a chosen closed-loop time constant, `kP = kA/closedLoopTau − kV`. Used by the launcher
     * flywheel tuner (its loop has no kA/kD term of its own; kP is a small trim on top of the
     * feedforward). `kP` is clamped to `[0, maxKpOverKv*kV]`: never negative (a P term can't brake a
     * single-direction flywheel), and capped so the proportional term stays subordinate to the
     * feedforward (the default cap of 1.0 keeps the closed loop no more than 2x the open-loop
     * bandwidth). A firing cap means the requested spin-up is faster than the feedforward-dominant
     * design allows -- raise `closedLoopTau` (accept a slower loop) rather than the cap.
     */
    fun velocityLoopGain(
        kV: Double,
        kA: Double,
        closedLoopTau: Double,
        maxKpOverKv: Double = 1.0,
    ): VelocityGain {
        require(kV > 0.0) { "kV must be > 0, was $kV" }
        require(kA > 0.0) { "kA must be > 0, was $kA" }
        require(closedLoopTau > 0.0) { "closedLoopTau must be > 0, was $closedLoopTau" }
        require(maxKpOverKv > 0.0) { "maxKpOverKv must be > 0, was $maxKpOverKv" }

        val openLoopTau = kA / kV
        val cap = maxKpOverKv * kV
        val kpRaw = kA / closedLoopTau - kV
        val kP = kpRaw.coerceIn(0.0, cap)
        return VelocityGain(kP, openLoopTau, closedLoopTau, kpCapped = kpRaw > cap)
    }

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

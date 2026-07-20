package org.firstinspires.ftc.teamcode.control

import kotlin.math.abs
import kotlin.math.exp

/**
 * First-order fading-memory (g-h / alpha-beta) tracking filter over a continuous scalar.
 *
 * It models a constant-velocity target, so each [update] yields both a smoothed [position] and a
 * smoothed [velocity] from a single intuitive knob -- the smoothing time constant `tau` (seconds).
 * This is the steady-state Kalman filter for a constant-velocity process, but parameterised by a
 * time constant instead of process/measurement covariances, so it can be tuned without a noise
 * characterisation.
 *
 * dt-aware: each call recomputes the discount factor `theta = exp(-dt/tau)` and the gains
 * `g = 1 - theta^2`, `h = (1 - theta)^2`, so the response is loop-rate invariant (matching how the
 * turret already handles a variable `dt`). `tau -> 0` snaps to the raw measurement; larger `tau`
 * smooths harder and adds lag.
 *
 * An optional innovation gate rejects outlier samples (sensor spikes): a measurement whose deviation
 * from the predicted position exceeds `spikeGate` is dropped and the estimate coasts on the model
 * instead. To guarantee the estimate can never wedge if the signal makes a genuine step, it
 * force-accepts after [DEFAULT_MAX_CONSECUTIVE_REJECTS] consecutive rejects.
 *
 * The input must NOT wrap. Feed a continuous, unwrapped signal -- e.g. the turret's fused angle,
 * which stays within the mechanical travel range and never crosses a 0/360 deg seam.
 */
class FadingMemoryFilter {
    /** Smoothed position estimate (same unit as the measurement). */
    var position: Double = 0.0
        private set

    /** Smoothed velocity estimate (measurement-unit per second). */
    var velocity: Double = 0.0
        private set

    private var initialized: Boolean = false
    private var consecutiveRejects: Int = 0

    /** Seed the estimate at [position] with zero velocity. Used on first lock and on re-acquisition. */
    fun reset(position: Double = 0.0) {
        this.position = position
        this.velocity = 0.0
        this.initialized = true
        this.consecutiveRejects = 0
    }

    /**
     * Fuse one [measurement] taken [dt] seconds after the previous update.
     *
     * @param dt seconds since the last update; a non-positive [dt] holds the estimate (no update).
     * @param tau smoothing time constant (s), must be > 0; larger = smoother and laggier.
     * @param spikeGate reject a measurement whose deviation from the prediction exceeds this
     *   (same unit as [measurement]); [Double.MAX_VALUE] disables the gate.
     * @param maxConsecutiveRejects force-accept after this many rejects in a row, so a genuine step
     *   can never permanently wedge the estimate.
     */
    fun update(
        dt: Double,
        measurement: Double,
        tau: Double,
        spikeGate: Double = Double.MAX_VALUE,
        maxConsecutiveRejects: Int = DEFAULT_MAX_CONSECUTIVE_REJECTS,
    ) {
        if (!initialized) {
            reset(measurement)
            return
        }
        if (dt <= 0.0) return

        // Constant-velocity prediction.
        val predictedPosition = position + velocity * dt
        val innovation = measurement - predictedPosition

        // Outlier gate: coast on the model rather than jump to a spike, but never wedge forever.
        if (abs(innovation) > spikeGate && consecutiveRejects < maxConsecutiveRejects) {
            position = predictedPosition
            // velocity unchanged (constant-velocity coast)
            consecutiveRejects++
            return
        }
        consecutiveRejects = 0

        val theta = exp(-dt / tau)
        val g = 1.0 - theta * theta
        val h = (1.0 - theta) * (1.0 - theta)

        position = predictedPosition + g * innovation
        velocity += (h / dt) * innovation
    }

    companion object {
        const val DEFAULT_MAX_CONSECUTIVE_REJECTS = 5
    }
}

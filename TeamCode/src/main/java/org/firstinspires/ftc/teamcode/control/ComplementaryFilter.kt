package org.firstinspires.ftc.teamcode.control

import kotlin.math.abs

/**
 * First-order complementary filter that fuses a trustworthy short-term *motion* measurement with a
 * drift-free long-term *absolute* measurement of the same continuous scalar.
 *
 * Each [update] takes a [motionDelta] (how far the quantity moved since the last call, e.g. from a
 * relative encoder's tick difference) and an [absoluteMeasurement] (a noisy but drift-free reading,
 * e.g. the fused turret angle). The motion drives a prediction; the absolute corrects it:
 *
 *     predicted = position + motionDelta          // high-pass: follows fast motion with no lag/noise
 *     position  = predicted + alpha * (absoluteMeasurement - predicted)   // low-pass toward truth
 *
 * [alpha] in (0, 1] is the correction fraction: `alpha -> 0` trusts the motion source and lets the
 * absolute leak in slowly (heavy smoothing, but a scale error in the motion source drifts out only as
 * fast as alpha); `alpha == 1` snaps to the absolute every loop (ignores the motion source). The
 * caller is expected to derive `alpha` from a crossover time constant and the loop `dt`
 * (`alpha = 1 - exp(-dt/tau)`) so behaviour is loop-rate invariant -- this class does not assume a
 * fixed per-loop alpha.
 *
 * Unlike a [FadingMemoryFilter] (a single-sensor g-h filter that *estimates* velocity in order to
 * predict), here motion is measured directly, so [velocity] is simply `motionDelta / dt` -- but
 * prefer a hardware tachometer for velocity where available; this value is exposed mainly for
 * telemetry/contrast.
 *
 * An optional innovation gate rejects an absolute reading that disagrees with the motion prediction
 * by more than [gate] (a glitch or a revolution mis-decode), coasting on the motion instead; it
 * force-accepts after [DEFAULT_MAX_CONSECUTIVE_REJECTS] rejects so a genuine step can never wedge the
 * estimate. NOTE: the gate prevents wedging, it does NOT make a mis-signed motion source safe -- under
 * a wrong motion sign the *absolute* is the correct signal and the gate would reject it, so a wrong
 * sign must be prevented upstream, not relied on to be caught here.
 *
 * The input must NOT wrap. Feed a continuous, unwrapped signal (the turret's fused angle qualifies).
 */
class ComplementaryFilter {
    /** Fused position estimate (same unit as the measurements). */
    var position: Double = 0.0
        private set

    /** Velocity estimate = motionDelta / dt (same unit per second). Exposed for telemetry/contrast. */
    var velocity: Double = 0.0
        private set

    private var initialized: Boolean = false
    private var consecutiveRejects: Int = 0

    /** Seed the estimate at [position] with zero velocity. Used on first lock and re-acquisition. */
    fun reset(position: Double = 0.0) {
        this.position = position
        this.velocity = 0.0
        this.initialized = true
        this.consecutiveRejects = 0
    }

    /**
     * Fuse one loop: advance by [motionDelta], then correct toward [absoluteMeasurement].
     *
     * @param dt seconds since the last update; a non-positive [dt] holds the estimate (no update).
     * @param motionDelta measured displacement since the last update (already sign/scale calibrated).
     * @param absoluteMeasurement drift-free (but noisy) reading to correct toward.
     * @param alpha correction fraction in (0, 1]; caller derives it from a crossover tau and [dt].
     * @param gate reject an absolute whose deviation from the prediction exceeds this (same unit);
     *   [Double.MAX_VALUE] disables the gate.
     * @param maxConsecutiveRejects force-accept after this many rejects in a row (non-wedge guard).
     */
    fun update(
        dt: Double,
        motionDelta: Double,
        absoluteMeasurement: Double,
        alpha: Double,
        gate: Double = Double.MAX_VALUE,
        maxConsecutiveRejects: Int = DEFAULT_MAX_CONSECUTIVE_REJECTS,
    ) {
        if (!initialized) {
            reset(absoluteMeasurement)
            return
        }
        if (dt <= 0.0) return

        val predicted = position + motionDelta
        velocity = motionDelta / dt
        val innovation = absoluteMeasurement - predicted

        // Outlier gate: coast on the motion prediction rather than jump to a bad absolute, but never
        // wedge forever.
        if (abs(innovation) > gate && consecutiveRejects < maxConsecutiveRejects) {
            position = predicted
            consecutiveRejects++
            return
        }
        consecutiveRejects = 0

        position = predicted + alpha * innovation
    }

    companion object {
        const val DEFAULT_MAX_CONSECUTIVE_REJECTS = 5
    }
}

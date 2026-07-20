package org.firstinspires.ftc.teamcode.control

/**
 * Model-free per-loop current governor for a single-direction motor (the launcher flywheel). It
 * produces a power cap in `[0,1]`; the caller applies it as `commanded = min(requestedPower, cap)`,
 * so the cap only ever *reduces* power — it never adds or reverses it.
 *
 * The goal is the fastest spin-up that keeps draw at/under a limit. A DC motor's current is highest
 * at stall (0 speed, full power) and falls as back-EMF builds, so holding current at the limit during
 * the high-torque phase gives constant torque ⇒ constant acceleration ⇒ the shortest time-to-speed
 * for that current budget. This governor approximates that without any motor model:
 *
 *  - **Over the limit** → `cap *= limit / current`. Because current is ~proportional to applied power
 *    at a fixed speed, that factor brings the *next* loop's draw to ~the limit in a single step — so
 *    the initial stall spike is caught in one loop. It is also inherently anti-windup: the cap tracks
 *    what the current actually allows and can never wind up above it.
 *  - **Under the limit** → the cap re-opens at a bounded rate ([recoveryPerSecond]). As the flywheel
 *    speeds up and back-EMF drops the current, this lets progressively more power through (tracking
 *    the rising power needed to hold the limit) without chattering on a noisy current reading.
 *
 * Once the wheel is near speed the steady-state current sits well below the limit, the cap recovers to
 * 1, and `min(requestedPower, cap)` hands cleanly back to the caller's feedforward+P velocity hold —
 * one law covers both cold spin-up and post-shot recovery.
 *
 * **Disabled / failsafe:** a non-positive [limitAmps], or a non-finite/negative current reading,
 * forces the cap to 1 (inert) — so the feature is off until a real limit is set, and a bad current
 * sensor degrades to the plain velocity loop rather than throttling the flywheel.
 *
 * Pure and unit-tested: no hardware, no clock (the caller passes `dt`).
 */
class CurrentLimiter(
    var limitAmps: Double = 0.0,          // <= 0 disables limiting (cap stays 1.0)
    var recoveryPerSecond: Double = 6.0,  // how fast the cap re-opens when under the limit (1/s)
) {
    var powerCap: Double = 1.0
        private set

    /** Re-arm to fully open (call from the subsystem's reset()). */
    fun reset() {
        powerCap = 1.0
    }

    /**
     * @param measuredCurrent motor current in amps; a non-finite or negative value is treated as
     *   "no reading" and makes the governor inert (cap = 1.0).
     * @param dt seconds since the last call; `<= 0` skips the recovery step (but still backs off).
     * @return the power cap in `[0,1]` to `min()` against the requested power.
     */
    fun update(measuredCurrent: Double, dt: Double): Double {
        if (limitAmps <= 0.0 || !measuredCurrent.isFinite() || measuredCurrent < 0.0) {
            powerCap = 1.0
            return powerCap
        }
        if (measuredCurrent > limitAmps) {
            powerCap *= limitAmps / measuredCurrent
        } else if (dt > 0.0) {
            powerCap += recoveryPerSecond * dt
        }
        powerCap = powerCap.coerceIn(0.0, 1.0)
        return powerCap
    }
}

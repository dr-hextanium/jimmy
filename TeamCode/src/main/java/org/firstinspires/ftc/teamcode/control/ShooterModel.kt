package org.firstinspires.ftc.teamcode.control

import com.bylazar.configurables.annotations.Configurable
import org.firstinspires.ftc.teamcode.utility.map

/**
 * Kinematic shooter model: turns a field distance into a shooting solution (flywheel speed + hood
 * position) from physical measurements and projectile physics, replacing the old empirical
 * power/hood regressions.
 *
 * Chain of reasoning:
 *  1. Pick a launch angle. The minimum-speed angle for the target `(distance, height)` is used,
 *     clamped to the hood's achievable range -- this is the launch angle that reaches the goal with
 *     the least flywheel effort, and it needs no tuning.
 *  2. Solve the exit speed that passes a projectile through the goal at that angle
 *     ([ProjectileSolver]).
 *  3. Convert exit speed to flywheel ticks/sec through the drivetrain geometry.
 *
 * The exit-speed conversion is derived, not fitted. A ball pinched between the flywheel and the
 * (counter-rotating, geared-up) back rollers leaves at roughly the mean of the two contact surface
 * speeds -- the standard no-slip two-wheel-shooter result. Everything downstream of flywheel
 * angular speed is linear, so a single [SLIP_EFFICIENCY] folds in all compression/slip losses.
 *
 * Every field below is a live-tunable (FTC Dashboard via @Configurable). The geometry constants are
 * real measurements; the ones marked CALIBRATE/CONFIRM must be set from the robot (they mirror the
 * turret's ENCODER_*_ZERO_OFFSET pattern) and default to safe, in-range values.
 */
@Configurable
object ShooterModel {
    const val INCHES_TO_METERS = 0.0254

    // --- Flywheel drivetrain geometry (physical measurements) ---
    var FLYWHEEL_DIAMETER_MM = 72.0
    var COUNTER_ROLLER_DIAMETER_MM = 28.0

    /** Back rollers spin reversed and 2x faster (40t -> 20t), so they drive the ball the same way. */
    var COUNTER_ROLLER_GEAR_RATIO = 2.0

    /** Flywheel-motor encoder counts per revolution. Bare motor default -- CONFIRM against datasheet. */
    var LAUNCHER_TICKS_PER_REV = 28.0

    // --- Energy transfer: one tunable folding all compression/slip losses. CALIBRATE on robot. ---
    var SLIP_EFFICIENCY = 0.85

    // --- Target geometry: goal-center height above the launch point (metres). CONFIRM/measure. ---
    var TARGET_HEIGHT_DELTA_M = 0.5

    // --- Hood: achievable launch-angle range and its linear servo calibration. CALIBRATE on robot. ---
    var HOOD_MIN_ANGLE_RAD = Math.toRadians(30.0)
    var HOOD_MAX_ANGLE_RAD = Math.toRadians(60.0)

    /** Hood servo position at [HOOD_MIN_ANGLE_RAD] / [HOOD_MAX_ANGLE_RAD] (defaults span the safe travel). */
    var SERVO_AT_MIN_ANGLE = 0.25
    var SERVO_AT_MAX_ANGLE = 0.905

    /** Flywheel ceiling in ticks/sec; the required speed is clamped to this. */
    var MAX_TPS = 2500.0

    private fun flywheelRadiusM() = FLYWHEEL_DIAMETER_MM / 2.0 / 1000.0
    private fun counterRollerRadiusM() = COUNTER_ROLLER_DIAMETER_MM / 2.0 / 1000.0

    /**
     * Ball exit speed produced per unit of flywheel ticks/sec (m/s per tick/s). Linear:
     * `v_ball = SLIP_EFFICIENCY * omega_f * meanContactRadius`, with
     * `omega_f = TPS * 2pi / TICKS_PER_REV` and the mean contact radius combining the flywheel and
     * the geared-up counter roller.
     */
    fun exitSpeedPerTps(): Double {
        val omegaPerTps = 2.0 * Math.PI / LAUNCHER_TICKS_PER_REV
        val meanContactRadius = 0.5 * (flywheelRadiusM() + COUNTER_ROLLER_GEAR_RATIO * counterRollerRadiusM())
        return SLIP_EFFICIENCY * omegaPerTps * meanContactRadius
    }

    /** Ball exit speed (m/s) for a given flywheel speed in ticks/sec. */
    fun exitSpeedFromTps(tps: Double): Double = exitSpeedPerTps() * tps

    /** Flywheel speed (ticks/sec) needed to launch the ball at [v] m/s. */
    fun tpsForExitSpeed(v: Double): Double = v / exitSpeedPerTps()

    /** Launch angle (rad) for a target at [distanceInches]: min-speed angle clamped to hood range. */
    fun launchAngleForDistance(distanceInches: Double): Double {
        val d = distanceInches * INCHES_TO_METERS
        val theta = ProjectileSolver.minimumSpeedAngle(d, TARGET_HEIGHT_DELTA_M)
        return theta.coerceIn(HOOD_MIN_ANGLE_RAD, HOOD_MAX_ANGLE_RAD)
    }

    /** Hood servo position for a launch angle, via the linear angle->servo calibration (clamped 0..1). */
    fun angleToServo(angleRad: Double): Double =
        map(angleRad, HOOD_MIN_ANGLE_RAD, HOOD_MAX_ANGLE_RAD, SERVO_AT_MIN_ANGLE, SERVO_AT_MAX_ANGLE)
            .coerceIn(0.0, 1.0)

    /**
     * A complete shooting solution for a given distance.
     *
     * @param targetTps flywheel target, ticks/sec, clamped to [MAX_TPS].
     * @param hoodServoPosition hood servo command, 0..1.
     * @param launchAngleRad the (clamped) launch angle used.
     * @param reachable true only if the geometry admits a solution AND the required speed is within
     *   [MAX_TPS]. When false, [targetTps] is the best (clamped) effort but the shot will fall short.
     */
    data class AimingSolution(
        val targetTps: Double,
        val hoodServoPosition: Double,
        val launchAngleRad: Double,
        val reachable: Boolean,
    )

    /** Solve the shooting solution for a target at [distanceInches] downrange. */
    fun aim(distanceInches: Double): AimingSolution {
        val d = distanceInches * INCHES_TO_METERS
        val theta = launchAngleForDistance(distanceInches)
        val requiredSpeed = ProjectileSolver.solveExitSpeed(d, TARGET_HEIGHT_DELTA_M, theta)

        val requiredTps = if (requiredSpeed.isNaN()) Double.NaN else tpsForExitSpeed(requiredSpeed)
        val reachable = !requiredTps.isNaN() && requiredTps <= MAX_TPS

        val targetTps = if (requiredTps.isNaN()) MAX_TPS else requiredTps.coerceIn(0.0, MAX_TPS)
        return AimingSolution(targetTps, angleToServo(theta), theta, reachable)
    }

    /**
     * Horizontal component of the ball's exit velocity for a shot at [distanceInches], in inches/sec
     * (matching the field's inch units so the turret can compute time-of-flight directly as
     * `distance / horizontalExitSpeed`). Reflects the *achievable* (clamped) flywheel speed.
     */
    fun horizontalExitSpeedInchesPerSec(distanceInches: Double): Double {
        val sol = aim(distanceInches)
        val v = exitSpeedFromTps(sol.targetTps)
        return ProjectileSolver.horizontalSpeed(v, sol.launchAngleRad) / INCHES_TO_METERS
    }
}

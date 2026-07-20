package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import org.firstinspires.ftc.teamcode.control.ShooterModel
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.testfakes.FakeAnalogInput
import org.firstinspires.ftc.teamcode.testfakes.FakeDcMotorEx
import org.firstinspires.ftc.teamcode.testfakes.FakeTimeSource
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.random.Random

/**
 * Tests for the Turret subsystem.
 *
 * Two independent concerns:
 *  1. The two-encoder vernier / Chinese-Remainder absolute-position decode ([fuseAbsoluteAngle],
 *     reached through read()/currentAngle). The forward model is derived purely from the tooth
 *     counts (137t / 12t / 13t), NOT from the decode itself, so `decode(forward(theta)) == theta`
 *     is a genuine self-consistency check.
 *  2. The trapezoidal-profiled feedforward+PID control law. Following the suite philosophy, these
 *     assert *contracts* -- the profiled setpoint stays within its velocity/accel limits and
 *     converges to the target, effort points the right way, output is clamped, a settled turret
 *     commands zero -- rather than pinning tuning gains. Time is driven by a [FakeTimeSource].
 *
 * IMPORTANT SCOPE: this validates decode ALGEBRA and control STRUCTURE only, not hardware
 * calibration -- gear-mesh direction, the ENCODER_*_ZERO_OFFSET_DEG constants, and the shooter
 * physics constants must still be confirmed on the physical robot.
 */
class TurretTest {
    // Physical gear ratios: idler encoder revolutions per turret revolution. Derived from the
    // tooth counts, independent of the production decode.
    private val r12 = 137.0 / 12.0
    private val r13 = 137.0 / 13.0
    private val encoderMaxVoltage = 3.3

    // Drive-motor encoder scale, mirrored from Turret (145.1 ticks/rev through the 137/15 reduction).
    private val ticksPerTurretRev = 145.1 * (137.0 / 15.0)
    private val degPerTick = 360.0 / ticksPerTurretRev
    private val seam = 360.0 * 12.0 / 137.0

    private lateinit var motor: FakeDcMotorEx
    private lateinit var encoder12: FakeAnalogInput
    private lateinit var encoder13: FakeAnalogInput
    private lateinit var clock: FakeTimeSource
    private lateinit var turret: Turret

    @Before
    fun setUp() {
        Robot.telemetry = MultipleTelemetry() // face() writes telemetry

        // Pin the shared companion tunables to their production defaults so test order can't leak.
        Turret.kP = 0.038
        Turret.kV = 0.0012
        Turret.kA = 0.0
        Turret.kD = 0.0
        Turret.kStatic = 0.02
        Turret.MIN_ANGLE = -90.0
        Turret.MAX_ANGLE = 90.0
        Turret.MAX_VELOCITY = 700.0
        Turret.MAX_ACCELERATION = 3600.0
        Turret.ANGLE_TOLERANCE_DEGREES = 0.2
        Turret.POWER_UPDATE_THRESHOLD = 0.01
        Turret.ENCODER_12T_ZERO_OFFSET_DEG = 0.0
        Turret.ENCODER_13T_ZERO_OFFSET_DEG = 0.0
        Turret.ANGLE_FILTER_TAU = 0.02
        Turret.ANGLE_FILTER_SPIKE_GATE = 10.0
        // Motor fusion OFF by default so the existing tests exercise the absolute-only path unchanged.
        Turret.USE_MOTOR_FUSION = false
        Turret.MOTOR_ANGLE_SIGN = 1.0
        Turret.MOTOR_FUSION_TAU = 0.10
        Turret.MOTOR_FUSION_GATE = 15.0
        Turret.MOTION_CLAMP_MARGIN = 2.0
        Turret.MOTOR_HEALTH_MIN_DELTA_DEG = 0.5
        Turret.MOTOR_HEALTH_MAX_DISAGREE = 12

        // face() and the (unused-here) aim path read ShooterModel; pin it to defaults.
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

        motor = FakeDcMotorEx()
        encoder12 = FakeAnalogInput()
        encoder13 = FakeAnalogInput()
        clock = FakeTimeSource()
        turret = Turret(motor, encoder12, encoder13, clock)
        turret.reset()
    }

    private fun wrap360(deg: Double): Double {
        var a = deg % 360.0
        if (a < 0) a += 360.0
        return a
    }

    /** Voltage a single idler encoder would report for a given true turret angle (physics model). */
    private fun voltageFor(theta: Double, ratio: Double, rawOffsetDeg: Double = 0.0): Double =
        wrap360(theta * ratio + rawOffsetDeg) / 360.0 * encoderMaxVoltage

    /** Drive the fake encoders to what they'd read at turret angle [theta], then latch via read(). */
    private fun applyTurretAngle(theta: Double, rawOffset12: Double = 0.0, rawOffset13: Double = 0.0) {
        encoder12.fakeVoltage = voltageFor(theta, r12, rawOffset12)
        encoder13.fakeVoltage = voltageFor(theta, r13, rawOffset13)
        turret.read()
    }

    /**
     * Set the encoders to [theta] and run one decode tick without advancing the clock. `dt == 0`
     * forces the memoryless vernier acquisition path (no filtering), so this returns exactly
     * `fuseAbsoluteAngle(...)` -- the pure decode algebra. (currentAngle now populates in update(),
     * not read().)
     */
    private fun decodedAngleAt(theta: Double, rawOffset12: Double = 0.0, rawOffset13: Double = 0.0): Double {
        applyTurretAngle(theta, rawOffset12, rawOffset13)
        turret.update()
        return turret.currentAngle
    }

    // ---- Chinese Remainder Theorem / vernier absolute-position decode ----

    @Test
    fun crtDecode_recoversTrueAngleAcrossFullRange() {
        // 350 deg physical travel => roughly +/-175 deg about zero.
        var theta = -175.0
        var worstErr = 0.0
        var worstAt = 0.0
        while (theta <= 175.0) {
            val err = abs(decodedAngleAt(theta) - theta)
            if (err > worstErr) { worstErr = err; worstAt = theta }
            theta += 0.05
        }
        assertTrue(
            "CRT decode drifted from true angle by $worstErr deg at theta=$worstAt (a full-revolution " +
                "flip would show ~31.5 deg). Decode ALGEBRA may be wrong -- do not 'fix' on hardware " +
                "assumptions; investigate fuseAbsoluteAngle.",
            worstErr < 1e-4
        )
    }

    @Test
    fun crtDecode_exactAtZero() {
        assertEquals(0.0, decodedAngleAt(0.0), 1e-6)
    }

    @Test
    fun crtDecode_handlesRevolutionBoundaries() {
        // The 12t encoder wraps every 360 * 12/137 ~= 31.53 deg of turret travel; round() picks the
        // revolution there, so check on and around those seams.
        val seam = 360.0 * 12.0 / 137.0
        for (k in -5..5) {
            val theta = k * seam
            if (theta < -175.0 || theta > 175.0) continue
            for (d in listOf(-0.3, -0.05, 0.0, 0.05, 0.3)) {
                val t = theta + d
                if (t < -175.0 || t > 175.0) continue
                assertEquals("decode failed near revolution seam at theta=$t", t, decodedAngleAt(t), 1e-3)
            }
        }
    }

    @Test
    fun crtDecode_honorsCalibrationOffsets() {
        // With a raw mounting offset on each encoder and the matching ZERO_OFFSET_DEG calibration,
        // the decode should still recover the true angle.
        Turret.ENCODER_12T_ZERO_OFFSET_DEG = 37.0
        Turret.ENCODER_13T_ZERO_OFFSET_DEG = 211.0
        for (theta in listOf(-120.0, -45.0, 0.0, 30.0, 90.0, 160.0)) {
            val decoded = decodedAngleAt(theta, rawOffset12 = 37.0, rawOffset13 = 211.0)
            assertEquals("offset-compensated decode failed at theta=$theta", theta, decoded, 1e-3)
        }
    }

    // ---- continuity tracking + fading-memory filter (noise robustness) ----

    @Test
    fun tracking_survivesNoiseAtSeamWithoutRevolutionJump() {
        // A revolution seam is exactly where the memoryless round() is a coin-flip under noise, so a
        // fused-angle-only filter would still let the angle jump a full ~31.5 deg. Continuity tracking
        // pins the revolution to the (physically continuous) prior angle instead.
        val seam = 360.0 * 12.0 / 137.0
        val theta = 2 * seam
        applyTurretAngle(theta)
        turret.update() // clean acquire (dt == 0)

        val rng = Random(7)
        var worst = 0.0
        repeat(200) {
            applyTurretAngle(theta, rawOffset12 = rng.nextDouble(-4.0, 4.0), rawOffset13 = rng.nextDouble(-4.0, 4.0))
            clock.advance(0.005)
            turret.update()
            worst = maxOf(worst, abs(turret.currentAngle - theta))
        }
        assertTrue(
            "noise must not flip the revolution (~31.5 deg jump); worst deviation was $worst deg",
            worst < 2.0
        )
    }

    @Test
    fun tracking_largeDtReacquiresTrueAngleAfterStall() {
        // At theta == 2*seam the 12t encoder reads exactly its zero, so a continuity pick from a stale
        // prior of 0 would wrongly report ~0. A loop stall (dt > MAX_DT) must fall back to the
        // memoryless vernier, which recovers the true angle regardless of the prior.
        val seam = 360.0 * 12.0 / 137.0
        applyTurretAngle(0.0)
        turret.update()
        repeat(5) { applyTurretAngle(0.0); clock.advance(0.01); turret.update() }

        val moved = 2 * seam
        applyTurretAngle(moved)
        clock.advance(0.2) // > MAX_DT (0.1)
        turret.update()
        assertEquals("large-dt stall must re-acquire the true angle", moved, turret.currentAngle, 1e-2)
    }

    @Test
    fun tracking_velocityEstimateFollowsConstantRotationRate() {
        // A clean velocity estimate is what makes the kD term usable; feed a constant-rate sweep and
        // check the filter recovers the rate (and still tracks position with no steady-state lag).
        applyTurretAngle(0.0)
        turret.update()

        val rate = 100.0 // deg/s
        val dt = 0.01
        var t = 0.0
        repeat(150) {
            t += dt
            applyTurretAngle(rate * t)
            clock.advance(dt)
            turret.update()
        }
        assertEquals("velocity estimate should track the true rotation rate", rate, turret.measuredVelocity, 1.0)
        assertEquals("position should track the moving angle", rate * t, turret.currentAngle, 0.5)
    }

    // ---- motor-encoder fusion (ComplementaryFilter) ----

    /** Encoder count the drive motor would report at motor-shaft angle [motorTheta] (sign +1). */
    private fun motorTicksFor(motorTheta: Double): Int = Math.round(motorTheta / degPerTick).toInt()

    /**
     * Drive the fakes for a turret angle [absTheta] (what the analog encoders sense) and an
     * independent motor-shaft angle [motorTheta] (what the quadrature encoder reports); they differ
     * only when modelling backlash. [motorVel] is the tach reading in ticks/s.
     */
    private fun applyFused(absTheta: Double, motorTheta: Double = absTheta, motorVel: Double = 0.0) {
        encoder12.fakeVoltage = voltageFor(absTheta, r12)
        encoder13.fakeVoltage = voltageFor(absTheta, r13)
        motor.fakeCurrentPosition = motorTicksFor(motorTheta)
        motor.fakeVelocity = motorVel
        turret.read()
    }

    /** Backlash as a play/dead-band: the motor moves through a [halfWidth] gap before the turret follows. */
    private class BacklashModel(private val halfWidth: Double) {
        var turret = 0.0
            private set
        fun onMotorAngle(phi: Double): Double {
            if (phi > turret + halfWidth) turret = phi - halfWidth
            else if (phi < turret - halfWidth) turret = phi + halfWidth
            return turret
        }
    }

    @Test
    fun fusion_tracksTrueAngleWhenMotorAndAbsoluteAgree() {
        Turret.USE_MOTOR_FUSION = true
        applyFused(0.0); turret.update() // cold-start vernier acquire

        val rate = 50.0
        val dt = 0.01
        var t = 0.0
        repeat(100) {
            t += dt
            applyFused(rate * t)
            clock.advance(dt)
            turret.update()
        }
        assertEquals("fused angle should track when motor + absolute agree", rate * t, turret.currentAngle, 0.5)
    }

    @Test
    fun fusion_backlashReversalTransientStaysBoundedAndTracksTheTurretNotTheMotor() {
        Turret.USE_MOTOR_FUSION = true
        val backlash = BacklashModel(halfWidth = 2.0)
        val dt = 0.01

        applyFused(backlash.onMotorAngle(0.0), motorTheta = 0.0); turret.update() // acquire

        var worst = 0.0
        var phi = 0.0
        fun drive(delta: Double) {
            phi += delta
            val trueTurret = backlash.onMotorAngle(phi)
            applyFused(trueTurret, motorTheta = phi)
            clock.advance(dt)
            turret.update()
            worst = maxOf(worst, abs(turret.currentAngle - trueTurret))
        }
        repeat(80) { drive(+0.5) } // forward
        repeat(80) { drive(-0.5) } // reverse -> gap traversal

        // The fused angle follows the TRUE turret (which ends a backlash-width from the motor), and the
        // reversal transient stays a small multiple of the backlash width -- never near a ~31.5° seam.
        assertTrue("reversal transient should stay bounded well under a seam (was $worst)", worst < 6.0)
        assertEquals("fused should settle on the true turret angle, not the motor angle",
            backlash.turret, turret.currentAngle, 0.5)
    }

    @Test
    fun fusion_deadReckonsThroughLoopStallWithoutSeamFlip() {
        Turret.USE_MOTOR_FUSION = true
        val start = 2 * seam // 12t reads its zero here -- the case a stale continuity prior mis-decodes
        applyFused(start); turret.update() // acquire
        repeat(3) { applyFused(start); clock.advance(0.01); turret.update() }

        // The turret really moved +20° during a loop stall; motor ticks counted it in hardware.
        val moved = start + 20.0
        applyFused(moved)
        clock.advance(0.2) // > MAX_DT (0.1): motor-carry, not a vernier re-acquire
        turret.update()
        assertEquals("motor-carry must dead-reckon the stall without a seam flip", moved, turret.currentAngle, 0.5)
    }

    @Test
    fun fusion_velocityComesFromTheTachometer() {
        Turret.USE_MOTOR_FUSION = true
        applyFused(0.0); turret.update() // acquire

        val tps = 400.0 // ticks/s
        applyFused(5.0, motorVel = tps)
        clock.advance(0.01)
        turret.update()
        assertEquals("velocity should be the tach reading in turret degrees",
            tps * degPerTick, turret.measuredVelocity, 1e-6)
    }

    @Test
    fun fusion_wrongSignDoesNotWedgeAndHealthMonitorReverts() {
        // A miscalibrated sign makes the motor prediction fight the truth. The estimate must stay
        // bounded (NOT runaway), and the health monitor must latch fusion off and revert to absolute.
        Turret.USE_MOTOR_FUSION = true
        Turret.MOTOR_ANGLE_SIGN = -1.0
        applyFused(0.0); turret.update() // acquire

        val rate = 50.0
        val dt = 0.01
        var t = 0.0
        var maxDev = 0.0
        repeat(40) {
            t += dt
            applyFused(rate * t)
            clock.advance(dt)
            turret.update()
            maxDev = maxOf(maxDev, abs(turret.currentAngle - rate * t))
        }
        assertTrue("wrong sign must not run away (bounded dev, was $maxDev)", maxDev < 15.0)
        assertTrue("health monitor must latch fusion off under a persistent sign disagreement",
            !turret.motorFusionHealthy)
    }

    // ---- trapezoidal-profiled feedforward + PID control law ----

    /** First update after reset seeds the profile at the current angle and integrates nothing. */
    private fun prime(current: Double, target: Double) {
        applyTurretAngle(current)
        turret.setTargetAngle(target)
        turret.update() // dt == 0 -> profiledPosition <- current, motorPower 0
    }

    private fun tick(dt: Double, current: Double? = null) {
        if (current != null) applyTurretAngle(current)
        clock.advance(dt)
        turret.update()
        turret.write()
    }

    @Test
    fun firstUpdateAfterResetSeedsProfileAndHoldsZeroPower() {
        prime(current = 15.0, target = 90.0)
        assertEquals("profile should seed at the measured angle", 15.0, turret.profiledPosition, 1e-9)
        assertEquals("first tick must not command power", 0.0, turret.motorPower, 1e-9)
    }

    @Test
    fun profiledSetpoint_convergesToTargetWithinVelocityLimit() {
        prime(current = 0.0, target = 90.0)
        var maxVel = 0.0
        var prevPos = turret.profiledPosition
        repeat(200) {
            tick(0.02, current = 0.0)
            assertTrue("profiled setpoint went backwards", turret.profiledPosition >= prevPos - 1e-9)
            assertTrue("profiled velocity exceeded MAX_VELOCITY", abs(turret.profiledVelocity) <= 700.0 + 1e-6)
            maxVel = maxOf(maxVel, abs(turret.profiledVelocity))
            prevPos = turret.profiledPosition
        }
        assertEquals("profile did not reach target", 90.0, turret.profiledPosition, 1e-3)
        assertEquals("profile did not settle to rest", 0.0, turret.profiledVelocity, 1e-3)
        assertTrue("profile should have accelerated meaningfully", maxVel > 100.0)
    }

    @Test
    fun positiveTarget_commandsPositivePower_negativeTargetNegative() {
        prime(current = 0.0, target = 60.0)
        tick(0.02, current = 0.0)
        assertTrue("should drive toward a positive target", turret.motorPower > 0.0)

        turret.reset()
        prime(current = 0.0, target = -60.0)
        tick(0.02, current = 0.0)
        assertTrue("should drive toward a negative target", turret.motorPower < 0.0)
    }

    @Test
    fun motorPower_isAlwaysClampedToUnitRange() {
        prime(current = 0.0, target = 90.0)
        repeat(100) {
            tick(0.02, current = 0.0) // current pinned far from the advancing setpoint -> saturates
            assertTrue("motor power escaped [-1,1]", abs(turret.motorPower) <= 1.0)
        }
    }

    @Test
    fun settledOnTarget_commandsZeroPower() {
        prime(current = 30.0, target = 30.0)
        repeat(10) { tick(0.02, current = 30.0) }
        assertEquals(0.0, turret.motorPower, 1e-9)
        assertTrue(turret.isAtTarget())
    }

    @Test
    fun write_pushesCommandedPowerToMotor() {
        prime(current = 0.0, target = 90.0)
        tick(0.02, current = 0.0)
        assertTrue("precondition: some power is being commanded", abs(turret.motorPower) > Turret.POWER_UPDATE_THRESHOLD)
        assertEquals("write() must push the commanded power to the motor", turret.motorPower, motor.getPower(), 1e-9)
    }

    // ---- setTargetAngle clamping ----

    @Test
    fun setTargetAngle_clampsToConfiguredRange() {
        turret.setTargetAngle(200.0)
        assertEquals(90.0, turret.targetAngle, 1e-9)

        turret.setTargetAngle(-200.0)
        assertEquals(-90.0, turret.targetAngle, 1e-9)

        turret.setTargetAngle(45.0)
        assertEquals(45.0, turret.targetAngle, 1e-9)
    }

    // ---- face(): aiming geometry + shoot-on-the-move lead ----

    private fun cartesian(x: Double, y: Double) = Vector(Pose(x, y))

    @Test
    fun face_straightAheadIsZero() {
        turret.face(cartesian(10.0, 0.0), Pose(0.0, 0.0, 0.0), Vector())
        assertEquals(0.0, turret.targetAngle, 1e-6)
    }

    @Test
    fun face_diagonalTargetIs45() {
        turret.face(cartesian(10.0, 10.0), Pose(0.0, 0.0, 0.0), Vector())
        assertEquals(45.0, turret.targetAngle, 1e-6)
    }

    @Test
    fun face_accountsForRobotHeading() {
        // Target dead ahead in field frame, but robot faces +90 deg: turret must swing to -90.
        turret.face(cartesian(10.0, 0.0), Pose(0.0, 0.0, Math.PI / 2.0), Vector())
        assertEquals(-90.0, turret.targetAngle, 1e-6)
    }

    @Test
    fun face_appliesManualOffset() {
        turret.offset = 10.0
        turret.face(cartesian(10.0, 0.0), Pose(0.0, 0.0, 0.0), Vector())
        assertEquals(10.0, turret.targetAngle, 1e-6)
    }

    @Test
    fun face_shootOnTheMoveLeadsTheTargetUsingShooterModelSpeed() {
        val target = cartesian(100.0, 0.0)
        val pose = Pose(0.0, 0.0, 0.0)

        // Stationary: aim straight at the goal (lead term vanishes regardless of ball speed).
        turret.face(target, pose, Vector())
        assertEquals(0.0, turret.targetAngle, 1e-6)
        val stationary = turret.targetAngle

        // Moving +y: the aim leads the target by robotVelocity * timeOfFlight, and time-of-flight
        // comes from the shooter's own horizontal exit speed (unified with the launcher).
        turret.face(target, pose, cartesian(0.0, 90.0))
        val ballSpeed = ShooterModel.horizontalExitSpeedInchesPerSec(100.0)
        val tof = 100.0 / ballSpeed
        val virtualGoalY = 0.0 - 90.0 * tof
        val expectedLead = Math.toDegrees(atan2(virtualGoalY, 100.0))
        assertEquals(expectedLead, turret.targetAngle, 1e-6)
        assertTrue("moving aim should differ from stationary", abs(turret.targetAngle - stationary) > 1.0)
    }
}

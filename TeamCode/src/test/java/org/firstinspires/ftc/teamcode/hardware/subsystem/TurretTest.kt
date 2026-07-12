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

    // ---- Chinese Remainder Theorem / vernier absolute-position decode ----

    @Test
    fun crtDecode_recoversTrueAngleAcrossFullRange() {
        // 350 deg physical travel => roughly +/-175 deg about zero.
        var theta = -175.0
        var worstErr = 0.0
        var worstAt = 0.0
        while (theta <= 175.0) {
            applyTurretAngle(theta)
            val err = abs(turret.currentAngle - theta)
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
        applyTurretAngle(0.0)
        assertEquals(0.0, turret.currentAngle, 1e-6)
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
                applyTurretAngle(t)
                assertEquals("decode failed near revolution seam at theta=$t", t, turret.currentAngle, 1e-3)
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
            applyTurretAngle(theta, rawOffset12 = 37.0, rawOffset13 = 211.0)
            assertEquals("offset-compensated decode failed at theta=$theta", theta, turret.currentAngle, 1e-3)
        }
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

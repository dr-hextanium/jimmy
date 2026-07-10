package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.testfakes.FakeAnalogInput
import org.firstinspires.ftc.teamcode.testfakes.FakeDcMotorEx
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test
import kotlin.math.abs
import kotlin.math.atan2

/**
 * Tests for the Turret subsystem.
 *
 * The centrepiece is a forward-model sweep of the two-encoder Chinese-Remainder / vernier decode
 * ([fuseAbsoluteAngle], reached through read()/currentAngle). The forward model is derived purely
 * from the drivetrain's tooth counts (137t turret gear, 12t and 13t idler encoder gears), NOT from
 * the decode itself, so `decode(forward(theta)) == theta` is a genuine self-consistency check
 * rather than a tautology.
 *
 * IMPORTANT SCOPE: this validates the decode ALGEBRA only. It does NOT validate hardware
 * calibration -- real gear-mesh direction (sign of the ratios) and the ENCODER_*_ZERO_OFFSET_DEG
 * constants must still be confirmed on the physical turret before trusting this for motion.
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
    private lateinit var turret: Turret

    @Before
    fun setUp() {
        Robot.telemetry = MultipleTelemetry() // face() writes telemetry

        // Pin the shared companion tunables to their production defaults so test order can't leak.
        Turret.kP = 0.038
        Turret.kStatic = 0.02
        Turret.MIN_ANGLE = -90.0
        Turret.MAX_ANGLE = 90.0
        Turret.ANGLE_TOLERANCE_DEGREES = 0.2
        Turret.POWER_UPDATE_THRESHOLD = 0.01
        Turret.SHOOT_SPEED = 180.0
        Turret.ENCODER_12T_ZERO_OFFSET_DEG = 0.0
        Turret.ENCODER_13T_ZERO_OFFSET_DEG = 0.0

        motor = FakeDcMotorEx()
        encoder12 = FakeAnalogInput()
        encoder13 = FakeAnalogInput()
        turret = Turret(motor, encoder12, encoder13)
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

    // ---- Proportional + static-feedforward control law ----

    private fun runControl(target: Double, currentTheta: Double) {
        applyTurretAngle(currentTheta)
        turret.setTargetAngle(target)
        turret.update()
        turret.write()
    }

    @Test
    fun control_positiveErrorDrivesPositivePowerWithStaticTerm() {
        runControl(target = 10.0, currentTheta = 0.0)
        // 10*kP + kStatic = 10*0.038 + 0.02 = 0.40
        assertEquals(0.40, turret.motorPower, 1e-9)
        assertEquals(0.40, motor.getPower(), 1e-9)
    }

    @Test
    fun control_negativeErrorDrivesNegativePower() {
        runControl(target = -10.0, currentTheta = 0.0)
        assertEquals(-0.40, turret.motorPower, 1e-9)
        assertEquals(-0.40, motor.getPower(), 1e-9)
    }

    @Test
    fun control_withinToleranceHoldsZeroPower() {
        runControl(target = 0.1, currentTheta = 0.0) // |error| 0.1 < tolerance 0.2
        assertEquals(0.0, turret.motorPower, 1e-9)
        assertEquals(0.0, motor.getPower(), 1e-9)
        assertTrue(turret.isAtTarget())
    }

    @Test
    fun control_largeErrorSaturatesToUnitPower() {
        runControl(target = 200.0, currentTheta = 0.0) // target clips to 90, error huge
        assertEquals(1.0, turret.motorPower, 1e-9)
        assertEquals(1.0, motor.getPower(), 1e-9)
    }

    @Test
    fun control_smallPowerDeltaBelowThresholdIsNotRewritten() {
        runControl(target = 10.0, currentTheta = 0.0) // writes 0.40
        assertEquals(0.40, motor.getPower(), 1e-9)

        // 10.1 -> 0.4038, only 0.0038 above the last write (< POWER_UPDATE_THRESHOLD 0.01): no write.
        runControl(target = 10.1, currentTheta = 0.0)
        assertEquals(0.4038, turret.motorPower, 1e-9) // internal target updated
        assertEquals(0.40, motor.getPower(), 1e-9)    // but the bus write was skipped

        // 20.0 -> 0.78, well over threshold: written.
        runControl(target = 20.0, currentTheta = 0.0)
        assertEquals(0.78, motor.getPower(), 1e-9)
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
    fun face_shootOnTheMoveLeadsTheTarget() {
        val target = cartesian(100.0, 0.0)
        val pose = Pose(0.0, 0.0, 0.0)

        // Stationary: aim straight at the goal.
        turret.face(target, pose, Vector())
        assertEquals(0.0, turret.targetAngle, 1e-6)
        val stationary = turret.targetAngle

        // Moving +y at 90 in/s with SHOOT_SPEED 180: time-of-flight 100/180 s => lead 50 in in -y,
        // virtual goal (100, -50) => aim atan2(-50, 100) ~= -26.565 deg.
        turret.face(target, pose, cartesian(0.0, 90.0))
        val expectedLead = Math.toDegrees(atan2(-50.0, 100.0))
        assertEquals(expectedLead, turret.targetAngle, 1e-3)
        assertTrue("moving aim should differ from stationary", abs(turret.targetAngle - stationary) > 1.0)
    }
}

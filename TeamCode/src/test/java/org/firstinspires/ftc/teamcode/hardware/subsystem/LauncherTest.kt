package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.control.ShooterModel
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.testfakes.FakeDcMotorEx
import org.firstinspires.ftc.teamcode.testfakes.FakeServo
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * Tests for the Launcher (flywheel) subsystem: speed measurement, at-speed detection, the scalar
 * -> TPS / hood conversions, the kinematic aim wiring, the feedforward-first velocity controller,
 * and the NaN-safe write path.
 *
 * The velocity-controller gains are pinned to defaults here so the (feedforward-dominated) power
 * outputs are deterministic; if a gain is re-tuned on the robot and one of these fails, update it
 * on purpose. MAX_TPS is sourced from ShooterModel, so its tunables are reset too.
 */
class LauncherTest {
    private lateinit var left: FakeDcMotorEx
    private lateinit var right: FakeDcMotorEx
    private lateinit var hood: FakeServo
    private lateinit var launcher: Launcher

    @Before
    fun setUp() {
        Robot.telemetry = MultipleTelemetry() // update() writes telemetry

        Launcher.kS = 0.0
        Launcher.kV = 0.0004
        Launcher.kP = 0.0003

        // MAX_TPS (and the aim path) come from ShooterModel; pin it to defaults.
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

        left = FakeDcMotorEx()
        right = FakeDcMotorEx()
        hood = FakeServo()
        launcher = Launcher(left, right, hood)
    }

    private fun setMeasuredVelocities(l: Double, r: Double) {
        left.fakeVelocity = l
        right.fakeVelocity = r
    }

    // ---- averageTPS ----

    @Test
    fun averageTPS_isMeanOfAbsoluteVelocities() {
        setMeasuredVelocities(1000.0, -2000.0) // right runs reversed on the robot
        assertEquals(1500.0, launcher.averageTPS, 1e-9)

        setMeasuredVelocities(-500.0, -1500.0)
        assertEquals(1000.0, launcher.averageTPS, 1e-9)
    }

    // ---- atSpeed / isReady ----

    @Test
    fun atSpeed_falseBelowMinimumTarget() {
        launcher.targetTPSByScalar(0.02) // 0.02 * 2500 = 50 TPS, under the 100 floor
        setMeasuredVelocities(50.0, 50.0)
        assertFalse(launcher.atSpeed)
        assertFalse(launcher.isReady)
    }

    @Test
    fun atSpeed_trueWhenWithinTolerance() {
        launcher.targetTPS = 2000.0
        setMeasuredVelocities(2000.0, 2000.0) // exact
        assertTrue(launcher.atSpeed)
        assertTrue(launcher.isReady)

        setMeasuredVelocities(2100.0, 2100.0) // ~4.9% off, within 5%
        assertTrue(launcher.atSpeed)
    }

    @Test
    fun atSpeed_falseOutsideTolerance() {
        launcher.targetTPS = 2000.0
        setMeasuredVelocities(2200.0, 2200.0) // ~9.5% off
        assertFalse(launcher.atSpeed)

        setMeasuredVelocities(1800.0, 1800.0) // ~10.5% off
        assertFalse(launcher.atSpeed)
    }

    // ---- scalar conversions ----

    @Test
    fun scaleToTPS_scalesByMaxTps() {
        assertEquals(0.5 * Launcher.MAX_TPS, launcher.scaleToTPS(0.5), 1e-9)
        assertEquals(Launcher.MAX_TPS, launcher.scaleToTPS(1.0), 1e-9)
    }

    @Test
    fun targetTPSByScalar_setsTarget() {
        launcher.targetTPSByScalar(0.4)
        assertEquals(0.4 * Launcher.MAX_TPS, launcher.targetTPS, 1e-9)
    }

    @Test
    fun targetHoodByScalar_mapsZeroToOneAcrossHoodTravel() {
        launcher.targetHoodByScalar(0.0)
        assertEquals(Launcher.HOOD_HIGH, launcher.targetHoodPosition, 1e-9)

        launcher.targetHoodByScalar(1.0)
        assertEquals(Launcher.HOOD_LOW, launcher.targetHoodPosition, 1e-9)

        launcher.targetHoodByScalar(0.5)
        assertEquals((Launcher.HOOD_HIGH + Launcher.HOOD_LOW) / 2.0, launcher.targetHoodPosition, 1e-9)
    }

    // ---- kinematic aim wiring ----

    @Test
    fun aimAtDistance_appliesShooterModelSolution() {
        val expected = ShooterModel.aim(120.0)
        launcher.aimAtDistance(120.0)
        assertEquals(expected.targetTps, launcher.targetTPS, 1e-9)
        assertEquals(expected.hoodServoPosition, launcher.targetHoodPosition, 1e-9)
    }

    // ---- update(): feedforward-first velocity controller ----

    @Test
    fun update_belowTarget_saturatesTowardFullPower() {
        launcher.targetTPSByScalar(0.8) // 2000 TPS
        setMeasuredVelocities(1000.0, 1000.0)
        launcher.update()
        // 0.0004*2000 + 0.0003*(2000-1000) = 0.8 + 0.3 = 1.1 -> clamped to 1.0
        assertEquals(1.0, launcher.currentPower, 1e-9)
    }

    @Test
    fun update_atTarget_holdsFeedforwardPower() {
        launcher.targetTPS = 2000.0
        setMeasuredVelocities(2000.0, 2000.0)
        launcher.update()
        // kV*target only (error is zero): 0.0004 * 2000 = 0.8
        assertEquals(0.8, launcher.currentPower, 1e-9)
    }

    @Test
    fun update_targetBelowFloor_commandsZero() {
        launcher.targetTPSByScalar(0.02) // 50 TPS, under MIN_TPS
        setMeasuredVelocities(0.0, 0.0)
        launcher.update()
        assertEquals(0.0, launcher.currentPower, 1e-9)
    }

    @Test
    fun update_aboveTarget_reducesPowerButNeverReverses() {
        launcher.targetTPS = 1000.0
        setMeasuredVelocities(2000.0, 2000.0) // overspeed
        launcher.update()
        // 0.0004*1000 + 0.0003*(1000-2000) = 0.4 - 0.3 = 0.1 (coasts, single-direction)
        assertEquals(0.1, launcher.currentPower, 1e-9)

        setMeasuredVelocities(5000.0, 5000.0) // way overspeed -> would go negative, clamped to 0
        launcher.update()
        assertEquals(0.0, launcher.currentPower, 1e-9)
    }

    @Test
    fun update_powerIsAlwaysWithinUnitRange() {
        launcher.targetTPS = 1500.0
        for (v in listOf(0.0, 750.0, 1500.0, 3000.0, 6000.0)) {
            setMeasuredVelocities(v, v)
            launcher.update()
            assertTrue("power $${launcher.currentPower} out of [0,1] at v=$v", launcher.currentPower in 0.0..1.0)
        }
    }

    // ---- write(): motor + hood output, NaN-safe threshold ----

    @Test
    fun write_pushesPowerAndHoodOnChange() {
        launcher.targetTPSByScalar(0.8)
        setMeasuredVelocities(0.0, 0.0)
        launcher.targetHoodByScalar(0.5)
        launcher.update() // currentPower saturates to 1.0
        launcher.write()

        assertEquals(1.0, left.getPower(), 1e-9)
        assertEquals(1.0, right.getPower(), 1e-9)
        assertEquals((Launcher.HOOD_HIGH + Launcher.HOOD_LOW) / 2.0, hood.getPosition(), 1e-9)
    }

    @Test
    fun write_transitionsPowerOffThenNoOpsWhenAlreadyZero() {
        // Spin up.
        launcher.targetTPSByScalar(0.8)
        setMeasuredVelocities(0.0, 0.0)
        launcher.update()
        launcher.write()
        assertEquals(1.0, left.getPower(), 1e-9)

        // Command off: target below floor -> power 0. percentDifference(1.0, 0.0) is large -> written.
        launcher.targetTPS = 0.0
        launcher.update()
        launcher.write()
        assertEquals(0.0, left.getPower(), 1e-9)
        assertEquals(0.0, right.getPower(), 1e-9)

        // Still off: percentDifference(0.0, 0.0) is NaN; the >0.005 guard is false, so this must be
        // a safe no-op (no exception, motors stay 0) rather than a spurious write.
        launcher.update()
        launcher.write()
        assertEquals(0.0, left.getPower(), 1e-9)
        assertEquals(0.0, right.getPower(), 1e-9)
    }

    @Test
    fun write_alwaysSetsHoodPosition() {
        launcher.targetHoodByScalar(0.0)
        launcher.write()
        assertEquals(Launcher.HOOD_HIGH, hood.getPosition(), 1e-9)

        launcher.targetHoodByScalar(1.0)
        launcher.write()
        assertEquals(Launcher.HOOD_LOW, hood.getPosition(), 1e-9)
    }

    // ---- reset() ----

    @Test
    fun reset_configuresMotorsAndHood() {
        // Dirty everything first to prove reset actually rewrites it.
        left.setDirection(DcMotorSimple.Direction.REVERSE)
        right.setDirection(DcMotorSimple.Direction.FORWARD)
        launcher.targetTPS = 1234.0
        launcher.currentPower = 0.9
        launcher.targetHoodPosition = 0.5

        launcher.reset()

        assertEquals(DcMotorSimple.Direction.FORWARD, left.getDirection())
        assertEquals(DcMotorSimple.Direction.REVERSE, right.getDirection())
        assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, left.getZeroPowerBehavior())
        assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, right.getZeroPowerBehavior())
        assertEquals(15.0, left.currentAlertAmps, 1e-9)
        assertEquals(15.0, right.currentAlertAmps, 1e-9)
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, left.getMode())
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, right.getMode())
        assertEquals(Launcher.HOOD_HIGH, hood.getPosition(), 1e-9)
        assertEquals(0.0, launcher.targetTPS, 1e-9)
        assertEquals(0.0, launcher.currentPower, 1e-9)
        // reset() must also seed the hood *target* to HOOD_HIGH, or the next write() clobbers it.
        assertEquals(Launcher.HOOD_HIGH, launcher.targetHoodPosition, 1e-9)
    }

    @Test
    fun reset_thenWrite_holdsHoodAtHigh_notZero() {
        // Regression: targetHoodPosition defaulted to 0.0 and was not reset, so the first write()
        // after reset() drove the hood to 0.0 -- below HOOD_HIGH (the usable-travel low end) --
        // until the driver first moved it in TeleOp. reset() must leave write() parking at HOOD_HIGH.
        launcher.targetHoodPosition = 0.0 // simulate the old default
        launcher.reset()
        launcher.write()
        assertEquals(Launcher.HOOD_HIGH, hood.getPosition(), 1e-9)
    }
}

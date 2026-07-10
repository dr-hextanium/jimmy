package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
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
 * -> TPS / hood conversions, the bang-bang power controller, and the NaN-safe write path.
 *
 * distanceToScalar is characterized (not contract-checked) because SCALAR_PER_INCH / BASE_SCALAR
 * are still placeholder zeros -- the test pins the current "always 0" behavior so a future tuning
 * change is a visible, deliberate edit.
 */
class LauncherTest {
    private lateinit var left: FakeDcMotorEx
    private lateinit var right: FakeDcMotorEx
    private lateinit var hood: FakeServo
    private lateinit var launcher: Launcher

    @Before
    fun setUp() {
        Robot.telemetry = MultipleTelemetry() // update() writes telemetry
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

    @Test
    fun distanceToScalar_characterization_placeholderReturnsZero() {
        // SCALAR_PER_INCH and BASE_SCALAR are still 0, so every distance maps to 0 (then clamped).
        // Update this test when those constants are tuned on-robot.
        assertEquals(0.0, launcher.distanceToScalar(0.0), 1e-9)
        assertEquals(0.0, launcher.distanceToScalar(50.0), 1e-9)
        assertEquals(0.0, launcher.distanceToScalar(1000.0), 1e-9)
    }

    // ---- update(): bang-bang controller ----

    @Test
    fun update_commandsFullPowerWhenBelowTarget() {
        launcher.targetTPSByScalar(0.8) // 2000 TPS
        setMeasuredVelocities(1000.0, 1000.0)
        launcher.update()
        assertEquals(1.0, launcher.currentPower, 1e-9)
    }

    @Test
    fun update_commandsZeroPowerWhenAtOrAboveTarget() {
        launcher.targetTPS = 1000.0
        setMeasuredVelocities(2000.0, 2000.0) // above target
        launcher.update()
        assertEquals(0.0, launcher.currentPower, 1e-9)

        setMeasuredVelocities(1000.0, 1000.0) // exactly at target -> error 0 -> off
        launcher.update()
        assertEquals(0.0, launcher.currentPower, 1e-9)
    }

    // ---- write(): motor + hood output, NaN-safe threshold ----

    @Test
    fun write_pushesPowerAndHoodOnChange() {
        launcher.targetTPSByScalar(0.8)
        setMeasuredVelocities(0.0, 0.0)
        launcher.targetHoodByScalar(0.5)
        launcher.update() // currentPower -> 1.0
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

        // Command off: percentDifference(1.0, 0.0) is large -> motors written to 0.
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
    }
}

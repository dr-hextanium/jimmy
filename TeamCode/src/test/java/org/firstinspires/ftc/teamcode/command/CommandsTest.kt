package org.firstinspires.ftc.teamcode.command

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import org.firstinspires.ftc.teamcode.command.intake.CloseGate
import org.firstinspires.ftc.teamcode.command.intake.IntakeIn
import org.firstinspires.ftc.teamcode.command.intake.IntakeOut
import org.firstinspires.ftc.teamcode.command.intake.IntakeWithGateClosed
import org.firstinspires.ftc.teamcode.command.intake.OpenGate
import org.firstinspires.ftc.teamcode.command.intake.StopIntake
import org.firstinspires.ftc.teamcode.command.launcher.FeedLauncherArtifacts
import org.firstinspires.ftc.teamcode.command.launcher.LaunchByDistance
import org.firstinspires.ftc.teamcode.command.launcher.ManualHood
import org.firstinspires.ftc.teamcode.command.launcher.ManuallyLaunch
import org.firstinspires.ftc.teamcode.command.launcher.StopLauncher
import org.firstinspires.ftc.teamcode.command.turret.AimAtGoal
import org.firstinspires.ftc.teamcode.command.turret.PointTowards
import org.firstinspires.ftc.teamcode.command.turret.StopAimingAtGoal
import org.firstinspires.ftc.teamcode.control.ShooterModel
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.hardware.subsystem.Intake
import org.firstinspires.ftc.teamcode.hardware.subsystem.Launcher
import org.firstinspires.ftc.teamcode.hardware.subsystem.Turret
import org.firstinspires.ftc.teamcode.testfakes.FakeAnalogInput
import org.firstinspires.ftc.teamcode.testfakes.FakeDcMotorEx
import org.firstinspires.ftc.teamcode.testfakes.FakeServo
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * Tests for the thin command wrappers. Each command's initialize() should nudge exactly one piece
 * of subsystem state; the CommandTemplate-based ones also report isFinished() == true (they are
 * one-shot). Subsystems are backed by fakes and installed into the Robot.Subsystems registry.
 */
class CommandsTest {
    private lateinit var intake: Intake
    private lateinit var launcher: Launcher
    private lateinit var turret: Turret

    @Before
    fun setUp() {
        Robot.telemetry = MultipleTelemetry()

        // Keep turret clamps deterministic for PointTowards.
        Turret.MIN_ANGLE = -90.0
        Turret.MAX_ANGLE = 90.0

        // LaunchByDistance -> aimAtDistance reads ShooterModel; pin it to defaults.
        ShooterModel.TARGET_HEIGHT_DELTA_M = 0.5
        ShooterModel.HOOD_MIN_ANGLE_RAD = Math.toRadians(30.0)
        ShooterModel.HOOD_MAX_ANGLE_RAD = Math.toRadians(60.0)
        ShooterModel.SERVO_AT_MIN_ANGLE = 0.25
        ShooterModel.SERVO_AT_MAX_ANGLE = 0.905
        ShooterModel.SLIP_EFFICIENCY = 0.85
        ShooterModel.LAUNCHER_TICKS_PER_REV = 28.0
        ShooterModel.MAX_TPS = 2500.0

        intake = Intake(
            FakeDcMotorEx(),
            FakeServo(),
        )
        launcher = Launcher(FakeDcMotorEx(), FakeDcMotorEx(), FakeServo())
        turret = Turret(FakeDcMotorEx(), FakeAnalogInput(), FakeAnalogInput())

        Robot.Subsystems.intake = intake
        Robot.Subsystems.launcher = launcher
        Robot.Subsystems.turret = turret
    }

    // ---- intake commands ----

    @Test
    fun intakeIn_defaultPower() {
        val cmd = IntakeIn()
        cmd.initialize()
        assertEquals(Intake.POWER_INTAKE, intake.power, 1e-9)
        assertTrue(cmd.isFinished())
    }

    @Test
    fun intakeIn_customPower() {
        IntakeIn(0.8).initialize()
        assertEquals(0.8, intake.power, 1e-9)
    }

    @Test
    fun intakeOut_reverses() {
        val cmd = IntakeOut()
        cmd.initialize()
        assertEquals(Intake.POWER_REVERSE, intake.power, 1e-9)
        assertTrue(cmd.isFinished())
    }

    @Test
    fun stopIntake_zeroesPower() {
        intake.power = 1.0
        val cmd = StopIntake()
        cmd.initialize()
        assertEquals(0.0, intake.power, 1e-9)
        assertTrue(cmd.isFinished())
    }

    @Test
    fun openGate_thenCloseGate() {
        OpenGate().initialize()
        assertTrue(intake.gateOpened)

        CloseGate().initialize()
        assertFalse(intake.gateOpened)
    }

    // ---- launcher commands ----

    @Test
    fun manuallyLaunch_setsTargetFromScalar() {
        val cmd = ManuallyLaunch { 0.71 }
        cmd.initialize()
        assertEquals(0.71 * Launcher.MAX_TPS, launcher.targetTPS, 1e-9)
        assertTrue(cmd.isFinished())
    }

    @Test
    fun stopLauncher_zeroesTarget() {
        launcher.targetTPS = 2000.0
        StopLauncher().initialize()
        assertEquals(0.0, launcher.targetTPS, 1e-9)
    }

    @Test
    fun manualHood_setsHoodFromScalar() {
        ManualHood { 0.5 }.initialize()
        assertEquals((Launcher.HOOD_HIGH + Launcher.HOOD_LOW) / 2.0, launcher.targetHoodPosition, 1e-9)
    }

    @Test
    fun launchByDistance_appliesKinematicAimingSolution() {
        val expected = ShooterModel.aim(60.0)
        LaunchByDistance(60.0).initialize()
        assertEquals(expected.targetTps, launcher.targetTPS, 1e-9)
        assertEquals(expected.hoodServoPosition, launcher.targetHoodPosition, 1e-9)
    }

    // ---- turret commands ----

    @Test
    fun pointTowards_setsClampedTarget() {
        PointTowards(30.0).initialize()
        assertEquals(30.0, turret.targetAngle, 1e-9)

        PointTowards(200.0).initialize()
        assertEquals(90.0, turret.targetAngle, 1e-9) // clamped to MAX_ANGLE
    }

    @Test
    fun aimAtGoal_togglesFlagOnAndOff() {
        AimAtGoal().initialize()
        assertTrue(turret.aimAtGoal)

        StopAimingAtGoal().initialize()
        assertFalse(turret.aimAtGoal)
    }

    // ---- composite (parallel) commands ----

    @Test
    fun feedLauncherArtifacts_runsIntakeAndOpensGate() {
        FeedLauncherArtifacts().initialize()
        assertEquals(Intake.POWER_INTAKE, intake.power, 1e-9)
        assertTrue(intake.gateOpened)
    }

    @Test
    fun intakeWithGateClosed_runsIntakeAndClosesGate() {
        intake.openGate() // ensure the command actively closes it
        IntakeWithGateClosed().initialize()
        assertEquals(0.8, intake.power, 1e-9)
        assertFalse(intake.gateOpened)
    }
}

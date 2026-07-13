package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.testfakes.FakeDcMotorEx
import org.firstinspires.ftc.teamcode.testfakes.FakeServo
import org.junit.Assert.assertEquals
import org.junit.Before
import org.junit.Test

/**
 * Tests for the Intake subsystem: the gate servo open/close output in write(), motor power
 * passthrough, and reset() configuration.
 */
class IntakeTest {
    private lateinit var motor: FakeDcMotorEx
    private lateinit var gate: FakeServo
    private lateinit var intake: Intake

    @Before
    fun setUp() {
        motor = FakeDcMotorEx()
        gate = FakeServo()
        intake = Intake(motor, gate)
    }

    // ---- gate open/close via write() ----

    @Test
    fun write_gateOpensAndCloses() {
        intake.openGate()
        intake.write()
        assertEquals(Intake.GATE_OPEN_POSITION, gate.getPosition(), 1e-9)

        intake.closeGate()
        intake.write()
        assertEquals(Intake.GATE_CLOSED_POSITION, gate.getPosition(), 1e-9)
    }

    @Test
    fun gateFlagDefaultsClosed() {
        // Fresh intake, no open call: write drives the gate closed.
        intake.write()
        assertEquals(Intake.GATE_CLOSED_POSITION, gate.getPosition(), 1e-9)
    }

    // ---- motor power passthrough ----

    @Test
    fun write_passesPowerThroughToMotor() {
        intake.power = Intake.POWER_INTAKE
        intake.write()
        assertEquals(Intake.POWER_INTAKE, motor.getPower(), 1e-9)

        intake.power = Intake.POWER_REVERSE
        intake.write()
        assertEquals(Intake.POWER_REVERSE, motor.getPower(), 1e-9)

        intake.power = 0.0
        intake.write()
        assertEquals(0.0, motor.getPower(), 1e-9)
    }

    // ---- reset() ----

    @Test
    fun reset_configuresMotorAndClosesGate() {
        motor.setDirection(DcMotorSimple.Direction.FORWARD) // wrong direction first
        gate.setPosition(Intake.GATE_OPEN_POSITION)

        intake.reset()

        assertEquals(DcMotorSimple.Direction.REVERSE, motor.getDirection())
        assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, motor.getZeroPowerBehavior())
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motor.getMode())
        assertEquals(0.0, motor.getPower(), 1e-9)
        assertEquals(Intake.GATE_CLOSED_POSITION, gate.getPosition(), 1e-9)
    }
}

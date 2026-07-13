package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.testfakes.FakeAnalogInput
import org.firstinspires.ftc.teamcode.testfakes.FakeDcMotorEx
import org.firstinspires.ftc.teamcode.testfakes.FakeServo
import org.junit.Assert.assertEquals
import org.junit.Test

/**
 * Proves the unit-test harness itself works before the real suite is written:
 *  - the src/test/java Kotlin source set is compiled and run,
 *  - FTC SDK classes (robotcore) are on the unit-test classpath,
 *  - the hand-rolled fakes construct and round-trip state,
 *  - the concrete-class fakes' null-controller super constructors don't blow up,
 *  - Robot.telemetry can be satisfied with an empty MultipleTelemetry.
 */
class HarnessSpikeTest {
    @Test
    fun sanity() {
        assertEquals(2, 1 + 1)
    }

    @Test
    fun ftcClasspathIsPresent() {
        // Range comes from com.qualcomm.robotcore -- proves the FTC AARs are on the test classpath.
        assertEquals(1.0, Range.clip(5.0, 0.0, 1.0), 0.0)
        assertEquals(-1.0, Range.clip(-5.0, -1.0, 1.0), 0.0)
    }

    @Test
    fun fakeMotorRoundTrips() {
        val m = FakeDcMotorEx()
        m.setPower(-0.25)
        assertEquals(-0.25, m.getPower(), 0.0)

        m.setDirection(DcMotorSimple.Direction.REVERSE)
        assertEquals(DcMotorSimple.Direction.REVERSE, m.getDirection())

        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, m.getMode())

        m.fakeVelocity = 1234.0
        assertEquals(1234.0, m.getVelocity(), 0.0)

        m.fakeCurrentPosition = 42
        assertEquals(42, m.getCurrentPosition())
    }

    @Test
    fun fakeServoRoundTrips() {
        val s = FakeServo()
        s.setPosition(0.3)
        assertEquals(0.3, s.getPosition(), 0.0)
    }

    @Test
    fun fakeAnalogInputReportsVoltage() {
        val a = FakeAnalogInput(1.65)
        assertEquals(1.65, a.voltage, 0.0)
        a.fakeVoltage = 3.3
        assertEquals(3.3, a.voltage, 0.0)
    }

    @Test
    fun robotTelemetryCanBeSatisfied() {
        Robot.telemetry = MultipleTelemetry()
        // Must not throw with zero backing telemetries.
        Robot.telemetry.addData("spike", 1.0)
        Robot.telemetry.addData("spike-str", "ok")
    }
}

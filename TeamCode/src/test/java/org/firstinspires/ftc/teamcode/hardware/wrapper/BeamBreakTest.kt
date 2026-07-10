package org.firstinspires.ftc.teamcode.hardware.wrapper

import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.testfakes.FakeDigitalChannel
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Tests for the BeamBreak wrapper. The polarity here matters: a broken beam (artifact present)
 * reads as digital LOW, so broken() == !state and intact() == state.
 */
class BeamBreakTest {
    @Test
    fun constructorSetsChannelToInputMode() {
        val ch = FakeDigitalChannel()
        ch.setMode(DigitalChannel.Mode.OUTPUT) // start in the wrong mode
        BeamBreak(ch)
        assertEquals(DigitalChannel.Mode.INPUT, ch.getMode())
    }

    @Test
    fun brokenWhenStateLow() {
        val ch = FakeDigitalChannel(fakeState = false)
        val beam = BeamBreak(ch)
        assertTrue(beam.broken())
        assertFalse(beam.intact())
    }

    @Test
    fun intactWhenStateHigh() {
        val ch = FakeDigitalChannel(fakeState = true)
        val beam = BeamBreak(ch)
        assertFalse(beam.broken())
        assertTrue(beam.intact())
    }

    @Test
    fun tracksStateChanges() {
        val ch = FakeDigitalChannel(fakeState = true)
        val beam = BeamBreak(ch)
        assertTrue(beam.intact())

        ch.fakeState = false
        assertTrue(beam.broken())

        ch.fakeState = true
        assertTrue(beam.intact())
    }
}

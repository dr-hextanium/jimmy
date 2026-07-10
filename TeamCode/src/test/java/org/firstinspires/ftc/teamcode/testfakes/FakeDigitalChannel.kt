package org.firstinspires.ftc.teamcode.testfakes

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.DigitalChannelImpl

/**
 * Pure-JVM stand-in for a [DigitalChannelImpl] (the concrete type [BeamBreak] takes). State and
 * mode are backed by real fields; the null controller passed to super is never consulted because
 * every method the code exercises is overridden here.
 */
class FakeDigitalChannel(var fakeState: Boolean = false) : DigitalChannelImpl(null, 0) {
    var fakeMode: DigitalChannel.Mode = DigitalChannel.Mode.INPUT

    override fun getState(): Boolean = fakeState
    override fun setState(state: Boolean) { fakeState = state }
    override fun setMode(mode: DigitalChannel.Mode) { fakeMode = mode }
    override fun getMode(): DigitalChannel.Mode = fakeMode
}

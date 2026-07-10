package org.firstinspires.ftc.teamcode.hardware.wrapper

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.DigitalChannelImpl

class BeamBreak(val channel: DigitalChannelImpl) {
    init {
        // Set the mode to INPUT so we can read the state.
        channel.mode = DigitalChannel.Mode.INPUT
    }

    fun broken() = !channel.state
    fun intact() = channel.state
}
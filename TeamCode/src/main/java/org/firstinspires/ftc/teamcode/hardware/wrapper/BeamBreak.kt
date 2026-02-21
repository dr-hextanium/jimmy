package org.firstinspires.ftc.teamcode.hardware.wrapper

import org.firstinspires.ftc.teamcode.hardware.Robot.hw

class BeamBreak(val name: String) {
    val channel by lazy { hw.digitalChannel[name] }

    fun broken() = channel.state
    fun intact() = !channel.state
}
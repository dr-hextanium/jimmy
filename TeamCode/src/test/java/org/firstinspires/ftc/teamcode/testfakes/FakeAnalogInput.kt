package org.firstinspires.ftc.teamcode.testfakes

import com.qualcomm.robotcore.hardware.AnalogInput

/**
 * Pure-JVM stand-in for an [AnalogInput]. [AnalogInput.getVoltage] normally delegates to a
 * controller; we override it to return a test-settable voltage and pass a null controller to the
 * superclass constructor (which only stores it and is never consulted here).
 */
class FakeAnalogInput(var fakeVoltage: Double = 0.0) : AnalogInput(null, 0) {
    override fun getVoltage(): Double = fakeVoltage
}

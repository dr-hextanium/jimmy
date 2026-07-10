package org.firstinspires.ftc.teamcode.testfakes

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController

/**
 * Pure-JVM stand-in for a [Servo]. Position and direction are backed by real state (subsystems
 * read back [getPosition] to decide whether to re-write it); the rest throws [NotImplementedError].
 * Read state back through [getPosition]/[getDirection].
 */
class FakeServo : Servo {
    private var positionValue = 0.0
    private var directionValue = Servo.Direction.FORWARD

    override fun setPosition(position: Double) { positionValue = position }
    override fun getPosition(): Double = positionValue
    override fun setDirection(direction: Servo.Direction) { directionValue = direction }
    override fun getDirection(): Servo.Direction = directionValue

    override fun getController(): ServoController = throw NotImplementedError("FakeServo.getController")
    override fun getPortNumber(): Int = throw NotImplementedError("FakeServo.getPortNumber")
    override fun scaleRange(min: Double, max: Double) = throw NotImplementedError("FakeServo.scaleRange")

    override fun getManufacturer(): HardwareDevice.Manufacturer = throw NotImplementedError("FakeServo.getManufacturer")
    override fun getDeviceName(): String = throw NotImplementedError("FakeServo.getDeviceName")
    override fun getConnectionInfo(): String = throw NotImplementedError("FakeServo.getConnectionInfo")
    override fun getVersion(): Int = throw NotImplementedError("FakeServo.getVersion")
    override fun resetDeviceConfigurationForOpMode() = throw NotImplementedError("FakeServo.resetDeviceConfigurationForOpMode")
    override fun close() = throw NotImplementedError("FakeServo.close")
}

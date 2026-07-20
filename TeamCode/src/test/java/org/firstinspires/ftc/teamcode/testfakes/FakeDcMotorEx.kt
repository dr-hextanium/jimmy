package org.firstinspires.ftc.teamcode.testfakes

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

/**
 * A pure-JVM stand-in for a [DcMotorEx] used by unit tests. Only the handful of properties the
 * subsystems actually touch are backed by real state (power, velocity, direction, mode,
 * zero-power behavior, encoder position, current-alert). Everything else throws
 * [NotImplementedError] so an unexpected code path fails loudly instead of silently returning a
 * bogus default.
 *
 * Read state back through the standard SDK accessors ([getPower], [getDirection], [getMode],
 * [getZeroPowerBehavior], [getVelocity], [getCurrentPosition]). The `fake*` fields are test-only
 * knobs for state a real motor reports read-only (measured velocity, encoder count): set them
 * directly to simulate hardware.
 */
@Suppress("OVERRIDE_DEPRECATION") // DcMotorEx declares deprecated PID-coefficient accessors
class FakeDcMotorEx : DcMotorEx {
    private var powerValue = 0.0
    private var directionValue = DcMotorSimple.Direction.FORWARD
    private var modeValue = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    private var zeroPowerBehaviorValue = DcMotor.ZeroPowerBehavior.UNKNOWN

    /** Simulated measured velocity (ticks/s). Also what [setVelocity] writes. */
    var fakeVelocity = 0.0
    /** Simulated encoder count. A real motor reports this read-only; tests set it directly. */
    var fakeCurrentPosition = 0
    /** Last current-alert threshold written via [setCurrentAlert], in amps. */
    var currentAlertAmps = 0.0
    /** Simulated motor current (amps). A real motor reports this read-only; tests set it directly. */
    var fakeCurrent = 0.0

    // --- DcMotorSimple ---
    override fun setPower(power: Double) { powerValue = power }
    override fun getPower(): Double = powerValue
    override fun setDirection(direction: DcMotorSimple.Direction) { directionValue = direction }
    override fun getDirection(): DcMotorSimple.Direction = directionValue

    // --- DcMotor ---
    override fun setMode(mode: DcMotor.RunMode) { modeValue = mode }
    override fun getMode(): DcMotor.RunMode = modeValue
    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        zeroPowerBehaviorValue = zeroPowerBehavior
    }
    override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior = zeroPowerBehaviorValue
    override fun getCurrentPosition(): Int = fakeCurrentPosition

    override fun getMotorType(): MotorConfigurationType = throw NotImplementedError("FakeDcMotorEx.getMotorType")
    override fun setMotorType(motorType: MotorConfigurationType?) = throw NotImplementedError("FakeDcMotorEx.setMotorType")
    override fun getController(): DcMotorController = throw NotImplementedError("FakeDcMotorEx.getController")
    override fun getPortNumber(): Int = throw NotImplementedError("FakeDcMotorEx.getPortNumber")
    override fun setPowerFloat() = throw NotImplementedError("FakeDcMotorEx.setPowerFloat")
    override fun getPowerFloat(): Boolean = throw NotImplementedError("FakeDcMotorEx.getPowerFloat")
    override fun setTargetPosition(position: Int) = throw NotImplementedError("FakeDcMotorEx.setTargetPosition")
    override fun getTargetPosition(): Int = throw NotImplementedError("FakeDcMotorEx.getTargetPosition")
    override fun isBusy(): Boolean = throw NotImplementedError("FakeDcMotorEx.isBusy")

    // --- DcMotorEx ---
    override fun setMotorEnable() = throw NotImplementedError("FakeDcMotorEx.setMotorEnable")
    override fun setMotorDisable() = throw NotImplementedError("FakeDcMotorEx.setMotorDisable")
    override fun isMotorEnabled(): Boolean = throw NotImplementedError("FakeDcMotorEx.isMotorEnabled")
    override fun setVelocity(angularRate: Double) { fakeVelocity = angularRate }
    override fun setVelocity(angularRate: Double, unit: AngleUnit?) = throw NotImplementedError("FakeDcMotorEx.setVelocity(unit)")
    override fun getVelocity(): Double = fakeVelocity
    override fun getVelocity(unit: AngleUnit?): Double = throw NotImplementedError("FakeDcMotorEx.getVelocity(unit)")
    override fun setPIDCoefficients(mode: DcMotor.RunMode?, pidCoefficients: PIDCoefficients?) = throw NotImplementedError("FakeDcMotorEx.setPIDCoefficients")
    override fun setPIDFCoefficients(mode: DcMotor.RunMode?, pidfCoefficients: PIDFCoefficients?) = throw NotImplementedError("FakeDcMotorEx.setPIDFCoefficients")
    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) = throw NotImplementedError("FakeDcMotorEx.setVelocityPIDFCoefficients")
    override fun setPositionPIDFCoefficients(p: Double) = throw NotImplementedError("FakeDcMotorEx.setPositionPIDFCoefficients")
    override fun getPIDCoefficients(mode: DcMotor.RunMode?): PIDCoefficients = throw NotImplementedError("FakeDcMotorEx.getPIDCoefficients")
    override fun getPIDFCoefficients(mode: DcMotor.RunMode?): PIDFCoefficients = throw NotImplementedError("FakeDcMotorEx.getPIDFCoefficients")
    override fun setTargetPositionTolerance(tolerance: Int) = throw NotImplementedError("FakeDcMotorEx.setTargetPositionTolerance")
    override fun getTargetPositionTolerance(): Int = throw NotImplementedError("FakeDcMotorEx.getTargetPositionTolerance")
    override fun getCurrent(unit: CurrentUnit?): Double = fakeCurrent // test knob is in amps; unit ignored
    override fun getCurrentAlert(unit: CurrentUnit?): Double = throw NotImplementedError("FakeDcMotorEx.getCurrentAlert")
    override fun setCurrentAlert(current: Double, unit: CurrentUnit?) { currentAlertAmps = current }
    override fun isOverCurrent(): Boolean = throw NotImplementedError("FakeDcMotorEx.isOverCurrent")

    // --- HardwareDevice ---
    override fun getManufacturer(): HardwareDevice.Manufacturer = throw NotImplementedError("FakeDcMotorEx.getManufacturer")
    override fun getDeviceName(): String = throw NotImplementedError("FakeDcMotorEx.getDeviceName")
    override fun getConnectionInfo(): String = throw NotImplementedError("FakeDcMotorEx.getConnectionInfo")
    override fun getVersion(): Int = throw NotImplementedError("FakeDcMotorEx.getVersion")
    override fun resetDeviceConfigurationForOpMode() = throw NotImplementedError("FakeDcMotorEx.resetDeviceConfigurationForOpMode")
    override fun close() = throw NotImplementedError("FakeDcMotorEx.close")
}

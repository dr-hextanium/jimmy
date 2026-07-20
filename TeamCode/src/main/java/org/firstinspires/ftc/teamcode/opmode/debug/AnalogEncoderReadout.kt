package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.hardware.Names
import org.firstinspires.ftc.teamcode.hardware.subsystem.Turret

/**
 * Pure sensor-only readout of the turret's two absolute analog encoders.
 *
 * Reads NOTHING except the two [AnalogInput] devices, looked up directly from the hardware map by the
 * names in [Names.AnalogDevices.Turret]. No motor, no [Turret] subsystem, no closed-loop control -- so
 * it runs on a robot that has only the encoders wired (the turret motor need not be present or
 * configured). Nothing is ever written to hardware.
 *
 * Per encoder it shows the raw voltage and the equivalent absolute degrees (voltage / 3.3V * 360).
 * Use it to sanity-check that each encoder is wired, powered, and sweeping 0..3.3V as its shaft turns.
 * (For the fused CRT/vernier angle and turret jog, use "Turret Encoder Debug" instead.)
 */
@TeleOp(name = "Analog Encoder Readout", group = "Debug")
class AnalogEncoderReadout : OpMode() {
    private val enc12 by lazy { hardwareMap.get(AnalogInput::class.java, Names.AnalogDevices.Turret.encoder12Tooth) }
    private val enc13 by lazy { hardwareMap.get(AnalogInput::class.java, Names.AnalogDevices.Turret.encoder13Tooth) }

    override fun init() {
        telemetry.addLine("Pure analog-encoder readout. Turn each encoder shaft and watch")
        telemetry.addLine("voltage sweep 0..${Turret.ENCODER_MAX_VOLTAGE}V (0..360 deg). No motor is used.")
    }

    override fun loop() {
        val v12 = enc12.voltage
        val v13 = enc13.voltage

        telemetry.addData("enc12 (${Names.AnalogDevices.Turret.encoder12Tooth}) voltage", "%.3f V", v12)
        telemetry.addData("enc12 raw deg", "%.1f", voltageToDegrees(v12))
        telemetry.addLine()
        telemetry.addData("enc13 (${Names.AnalogDevices.Turret.encoder13Tooth}) voltage", "%.3f V", v13)
        telemetry.addData("enc13 raw deg", "%.1f", voltageToDegrees(v13))
        telemetry.addLine()
        telemetry.addData("max voltage (ref)", "%.2f V", Turret.ENCODER_MAX_VOLTAGE)
    }

    private fun voltageToDegrees(voltage: Double): Double = (voltage / Turret.ENCODER_MAX_VOLTAGE) * 360.0
}

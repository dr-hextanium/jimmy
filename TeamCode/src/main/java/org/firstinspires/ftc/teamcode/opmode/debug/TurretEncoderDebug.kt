package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.hardware.Names
import org.firstinspires.ftc.teamcode.hardware.subsystem.Turret

/**
 * Readout for calibrating the turret's two-encoder CRT / vernier absolute-angle decode.
 *
 * Rotate the turret (by hand, or with the triggers) and watch:
 *  - the raw voltage and degrees of each idler encoder,
 *  - the FUSED absolute angle the decode produces,
 *  - the motor's own encoder ticks.
 *
 * To calibrate: move the turret to its true mechanical zero and read the two "raw deg" values --
 * those are your ENCODER_12T_ZERO_OFFSET_DEG / ENCODER_13T_ZERO_OFFSET_DEG. Also confirm the fused
 * angle increases when the turret rotates the direction you consider positive; if it decreases, the
 * gear-mesh sign assumption is wrong.
 *
 * Surgical + low-risk: constructs only the turret hardware, never runs closed-loop PID. The jog is
 * open-loop, power-capped, and dead-man (zero the instant you release the trigger).
 *
 * Controls (gamepad1): right trigger = jog one way, left trigger = jog the other (capped).
 */
@TeleOp(name = "Turret Encoder Debug", group = "Debug")
class TurretEncoderDebug : OpMode() {
    private val motor by lazy { hardwareMap.get(DcMotorEx::class.java, Names.Motors.Turret.motor) }
    private val enc12 by lazy { hardwareMap.get(AnalogInput::class.java, Names.AnalogDevices.Turret.encoder12Tooth) }
    private val enc13 by lazy { hardwareMap.get(AnalogInput::class.java, Names.AnalogDevices.Turret.encoder13Tooth) }
    private val turret by lazy { Turret(motor, enc12, enc13) }

    override fun init() {
        turret.reset()
        telemetry.addLine("Rotate the turret and watch FUSED angle. At true zero, the raw per-encoder")
        telemetry.addLine("degrees are your ENCODER_*_ZERO_OFFSET_DEG values.")
    }

    override fun loop() {
        turret.read() // updates the fused currentAngle from the two encoders (issues no motor command)

        // Open-loop manual jog: capped and dead-man. Never closed-loop here.
        val jog = Range.clip((gamepad1.right_trigger - gamepad1.left_trigger).toDouble(), -1.0, 1.0) * JOG_POWER
        motor.power = jog

        val deg12 = enc12.voltage / Turret.ENCODER_MAX_VOLTAGE * 360.0
        val deg13 = enc13.voltage / Turret.ENCODER_MAX_VOLTAGE * 360.0

        telemetry.addData("enc12 voltage", "%.3f", enc12.voltage)
        telemetry.addData("enc13 voltage", "%.3f", enc13.voltage)
        telemetry.addData("enc12 raw deg", "%.1f", deg12)
        telemetry.addData("enc13 raw deg", "%.1f", deg13)
        telemetry.addData("FUSED angle (deg)", "%.2f", turret.currentAngle)
        telemetry.addData("motor ticks", motor.currentPosition)
        telemetry.addData("jog power", "%.2f", jog)
        telemetry.addLine("--- current calibration constants ---")
        telemetry.addData("ENCODER_12T_ZERO_OFFSET_DEG", Turret.ENCODER_12T_ZERO_OFFSET_DEG)
        telemetry.addData("ENCODER_13T_ZERO_OFFSET_DEG", Turret.ENCODER_13T_ZERO_OFFSET_DEG)
        telemetry.addLine("triggers: jog +/- (dead-man, capped at $JOG_POWER)")
    }

    override fun stop() {
        motor.power = 0.0
    }

    companion object {
        private const val JOG_POWER = 0.2
    }
}

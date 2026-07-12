package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.Names

/**
 * Spins one drivetrain motor at a time (while its button is held) so you can confirm each motor's
 * wiring and forward direction. Motors coast to zero the instant the button is released -- nothing
 * latches on.
 *
 * Controls (gamepad1): triangle=front-right, square=front-left, circle=back-right, cross=back-left.
 */
@TeleOp(group = "Debug", name = "Motor Direction Debugger")
class MotorDirectionDebugger : OpMode() {
    private val fr by lazy { hardwareMap.get(DcMotor::class.java, Names.Motors.Drivetrain.frontRight) }
    private val fl by lazy { hardwareMap.get(DcMotor::class.java, Names.Motors.Drivetrain.frontLeft) }
    private val br by lazy { hardwareMap.get(DcMotor::class.java, Names.Motors.Drivetrain.backRight) }
    private val bl by lazy { hardwareMap.get(DcMotor::class.java, Names.Motors.Drivetrain.backLeft) }

    override fun init() { fr; fl; br; bl }

    override fun loop() {
        // Power reflects the current button state every loop -> released button means zero power.
        fr.power = if (gamepad1.triangle) 1.0 else 0.0
        fl.power = if (gamepad1.square) 1.0 else 0.0
        br.power = if (gamepad1.circle) 1.0 else 0.0
        bl.power = if (gamepad1.cross) 1.0 else 0.0

        telemetry.addData("front right (triangle)", fr.power)
        telemetry.addData("front left (square)", fl.power)
        telemetry.addData("back right (circle)", br.power)
        telemetry.addData("back left (cross)", bl.power)
    }

    override fun stop() {
        fr.power = 0.0
        fl.power = 0.0
        br.power = 0.0
        bl.power = 0.0
    }
}

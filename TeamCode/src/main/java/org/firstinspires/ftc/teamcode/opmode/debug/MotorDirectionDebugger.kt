package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(group = "Debug", name = "Motor Direction Debugger")
class MotorDirectionDebugger : OpMode() {
    val fr by lazy { hardwareMap["fr"] as DcMotor }
    val fl by lazy { hardwareMap["fl"] as DcMotor }
    val br by lazy { hardwareMap["br"] as DcMotor }
    val bl by lazy { hardwareMap["bl"] as DcMotor }

    override fun init() { fr; fl; br; bl }

    override fun loop() {
        when {
            gamepad1.triangle -> fr.power = 1.0
            gamepad1.square -> fl.power = 1.0
            gamepad1.circle -> br.power = 1.0
            gamepad1.cross -> bl.power = 1.0
        }

        telemetry.addData("front right", fr.power)
        telemetry.addData("front left", fl.power)
        telemetry.addData("back right", br.power)
        telemetry.addData("back left", bl.power)
    }
}
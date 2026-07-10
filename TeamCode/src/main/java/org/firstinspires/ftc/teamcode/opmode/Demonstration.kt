package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.roundToInt
import kotlin.math.sin

@TeleOp
class Demonstration : OpMode() {
    override fun init() {}

    override fun loop() {
        telemetry.addData("sine of runtime", (sin(runtime) * 1000.0).roundToInt() / 1000.0)
    }
}
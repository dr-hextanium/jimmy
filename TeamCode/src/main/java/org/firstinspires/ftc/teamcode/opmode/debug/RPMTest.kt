package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.command.launcher.Manual
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.opmode.template.BaseTemplate

@TeleOp
@Disabled
class RPMTest : BaseTemplate() {
    var launcherPower = 0.0
    var increment = 0.01

    val launcher by lazy { Robot.Subsystems.launcher }

    override fun initialize() {
        launcher
    }

    override fun cycle() {
        launcherPower += when {
            gamepad1.dpad_right -> increment
            gamepad1.dpad_left -> -increment
            else -> 0.0
        }.coerceIn(0.0, 1.0)

        if(gamepad1.right_bumper) Manual({ launcherPower })

        Robot.telemetry.addData("launcher power", launcherPower)
    }
}
package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.Names
import org.firstinspires.ftc.teamcode.hardware.subsystem.Intake

/**
 * Intake bring-up tool: lets you run the intake and open/close the gate to check motor direction
 * and gate travel. Surgical -- constructs only the Intake subsystem's hardware.
 *
 * The intake motor is dead-man: it only runs while a trigger is held.
 *
 * Controls (gamepad1): right trigger = intake in, left trigger = eject, A = open gate, B = close gate.
 */
@TeleOp(name = "Intake Debug", group = "Debug")
class IntakeDebug : OpMode() {
    private lateinit var intake: Intake

    override fun init() {
        val motor = hardwareMap.get(DcMotorEx::class.java, Names.Motors.Intake.motor)
        val gate = hardwareMap.get(Servo::class.java, Names.Servos.Intake.servo)
        intake = Intake(motor, gate)
        intake.reset()
    }

    override fun loop() {
        intake.read()

        intake.power = when {
            gamepad1.right_trigger > 0.3f -> Intake.POWER_INTAKE
            gamepad1.left_trigger > 0.3f -> Intake.POWER_REVERSE
            else -> 0.0
        }
        if (gamepad1.a) intake.openGate()
        if (gamepad1.b) intake.closeGate()

        intake.write()

        telemetry.addData("gate", if (intake.gateOpened) "OPEN" else "CLOSED")
        telemetry.addData("intake power", intake.power)
        telemetry.addLine("RT: intake  LT: eject  A: open gate  B: close gate")
    }

    override fun stop() {
        intake.power = 0.0
        intake.write()
    }
}

package org.firstinspires.ftc.teamcode.hardware.subsystem

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.ISubsystem
import org.firstinspires.ftc.teamcode.hardware.wrapper.BeamBreak

class Intake(
    val motor: DcMotorEx,
    val gate: Servo,

    val bottomBeamBreak: BeamBreak,
    val middleBeamBreak: BeamBreak,
    val topBeamBreak: BeamBreak,
) : ISubsystem {
    var gateOpened = false
	var power = 0.0

    var bottomHasArtifact = false
    var middleHasArtifact = false
    var topHasArtifact = false

	override fun reset() {
		motor.direction = REVERSE
		motor.zeroPowerBehavior = BRAKE
		motor.power = 0.0
		motor.mode = RunMode.RUN_WITHOUT_ENCODER

        gate.position = GATE_CLOSED_POSITION
	}

    fun openGate() { gateOpened = true }
    fun closeGate() { gateOpened = false }

	override fun read() {
        bottomHasArtifact = bottomBeamBreak.broken()
        middleHasArtifact = middleBeamBreak.broken()
        topHasArtifact = topBeamBreak.broken()
    }

	override fun update() {
//		Robot.telemetry.addData("intake power", power)
//        Robot.telemetry.addData("gate position", gate.position)
//        Robot.telemetry.addData("bottomBeamBreak broken", bottomHasArtifact)
//        Robot.telemetry.addData("middleBeamBreak broken", middleHasArtifact)
//        Robot.telemetry.addData("topBeamBreak broken", topHasArtifact)
	}

	override fun write() {
        val ideal = when {
            gateOpened -> GATE_OPEN_POSITION
            else -> GATE_CLOSED_POSITION
        }

        if (gate.position != ideal) {
            gate.position = ideal
        }

        if (motor.power != power) {
            motor.power = power
        }
	}

	companion object {
		const val POWER_INTAKE = 1.0
		const val POWER_REVERSE = -1.0

        const val GATE_OPEN_POSITION = 0.700
        const val GATE_CLOSED_POSITION = 0.445
	}
}
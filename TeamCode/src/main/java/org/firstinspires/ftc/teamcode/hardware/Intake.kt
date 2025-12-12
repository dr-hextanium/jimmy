package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Actors
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels

class Intake(hw: HardwareMap) : Subsystem() {
    val motor = hw[MOTOR_NAME] as DcMotorEx

    fun reset() {
        motor.direction = FORWARD
        motor.zeroPowerBehavior = FLOAT
        motor.power = 0.0
        motor.mode = RunMode.RUN_WITHOUT_ENCODER
    }

    init { reset() }

    val intake: Closure = Channels.send<Message>(
        { In },
        { actor.tx }
    )

    val outtake: Closure = Channels.send<Message>(
        { Out },
        { actor.tx }
    )

    val stop: Closure = Channels.send<Message>(
        { Stop },
        { actor.tx }
    )

    val actor = Actors.actor<State, Message>(
        { State.STOPPED },

        { _, message ->
            when (message) {
                is Stop -> State.STOPPED
                is In -> State.INTAKING
                is Out -> State.OUTTAKING
            }
        },

        { stateRegister ->
            match(stateRegister)
                .branch(State.STOPPED, exec {
                    motor.power = 0.0
                })
                .branch(State.INTAKING, exec {
                    motor.power = POWER_IN
                })
                .branch(State.OUTTAKING, exec {
                    motor.power = POWER_OUT
                })
                .assertExhaustive()
        }
    )

    sealed interface Message
    data object Stop : Message
    data object In : Message
    data object Out : Message

    enum class State {
        STOPPED,
        INTAKING,
        OUTTAKING
    }

    companion object {
        const val MOTOR_NAME = "intake"

        const val POWER_IN = 1.0
        const val POWER_OUT = -1.0
    }
}
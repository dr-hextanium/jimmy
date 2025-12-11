package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Actors
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels

class Transfer(hw: HardwareMap) : Subsystem() {
    val motor = hw[MOTOR_NAME] as DcMotorEx

    fun reset() {
        motor.direction = FORWARD
        motor.zeroPowerBehavior = BRAKE
        motor.power = 0.0
        motor.mode = RunMode.RUN_WITHOUT_ENCODER
    }

    init { reset() }

    val feed: Closure = Channels.send<Message>(
        { Feed },
        { actor.tx }
    )

    val reverse: Closure = Channels.send<Message>(
        { Reverse },
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
                is Feed -> State.FEEDING
                is Reverse -> State.REVERSING
            }
        },

        { stateRegister ->
            match(stateRegister)
                .branch(State.STOPPED, exec {
                    motor.power = 0.0
                })
                .branch(State.FEEDING, exec {
                    motor.power = POWER_FEED
                })
                .branch(State.REVERSING, exec {
                    motor.power = POWER_REVERSE
                })
                .assertExhaustive()
        }
    )

    sealed interface Message
    data object Stop : Message
    data object Feed : Message
    data object Reverse : Message

    enum class State {
        STOPPED,
        FEEDING,
        REVERSING
    }

    companion object {
        const val MOTOR_NAME = "transfer"

        const val POWER_FEED = 1.0
        const val POWER_REVERSE = -1.0
    }
}
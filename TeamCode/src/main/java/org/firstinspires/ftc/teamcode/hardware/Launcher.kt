package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Actors
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.utility.absPercentDifference

class Launcher(hw: HardwareMap) : Subsystem() {
    val left = hw[LEFT_MOTOR_NAME] as DcMotorEx
    val right = hw[RIGHT_MOTOR_NAME] as DcMotorEx
    val motors = listOf(left, right)

    var targetRPM = 0.0 // ticks per second

    val averageRPM: Double
        get() = (left.velocity + right.velocity) / 2.0

    val atSpeed: Boolean
        get() = absPercentDifference(averageRPM, targetRPM) <= AT_SPEED_TOLERANCE

    val withinSafetyMargins: Boolean
        get() = absPercentDifference(left.velocity, right.velocity) <= MAXIMUM_DEVIANCE

    val isReady: Boolean
        get() = atSpeed && withinSafetyMargins

    fun reset() {
        targetRPM = 0.0

        right.direction = FORWARD
        left.direction = REVERSE

        motors.forEach {
            it.zeroPowerBehavior = FLOAT
            it.power = 0.0
            it.velocity = 0.0

            it.setCurrentAlert(15.0, CurrentUnit.AMPS)

            it.mode = RunMode.STOP_AND_RESET_ENCODER
            it.mode = RunMode.RUN_USING_ENCODER
        }
    }

    init { reset() }

    // distance: inches
    fun distanceToRPM(distance: Double): Double {
        val calculated = (distance * RPM_PER_INCH) + BASE_RPM
        return calculated.coerceIn(0.0, MAX_RPM)
    }

    fun spinUp(distanceSupplier: () -> Double): Closure {
        return Channels.send<Message>(
            { SpinUp(distanceSupplier()) },
            { actor.tx }
        )
    }

    fun setRpm(rpmSupplier: () -> Double): Closure {
        return Channels.send<Message>(
            { OverrideRPM(rpmSupplier()) },
            { actor.tx }
        )
    }

    val stop: Closure = Channels.send<Message>(
        { Stop },
        { actor.tx }
    )

    val actor = Actors.actor<State, Message>(
        { State.STOPPED },

        { _, message ->
            when (message) {
                is Stop -> {
                    targetRPM = 0.0
                    State.STOPPED
                }

                is SpinUp -> {
                    targetRPM = distanceToRPM(message.distance)

                    if (isReady) State.AT_SPEED else State.SPINNING_UP
                }

                is OverrideRPM -> {
                    targetRPM = message.rpm

                    if (isReady) State.AT_SPEED else State.SPINNING_UP
                }
            }
        },

        { stateRegister ->
            match(stateRegister)
                .branch(State.STOPPED, exec {
                    motors.forEach { it.velocity = 0.0 }
                })
                .branch(State.SPINNING_UP, exec {
                    motors.forEach { it.velocity = targetRPM }
                })
                .branch(State.AT_SPEED, exec {
                    motors.forEach { it.velocity = targetRPM }
                })
                .assertExhaustive()
        }
    )

    sealed interface Message
    data object Stop : Message
    data class SpinUp(val distance: Double) : Message
    data class OverrideRPM(val rpm: Double) : Message

    enum class State {
        STOPPED,
        SPINNING_UP,
        AT_SPEED
    }

    companion object {
        const val MAX_RPM = 0.0 // ticks per second

        const val LEFT_MOTOR_NAME = "sl"
        const val RIGHT_MOTOR_NAME = "sr"

        // the maximum amount that the left and right motors can deviate at any given time
        const val MAXIMUM_DEVIANCE = 0.1 // 10%

        // the tolerance to certify a AT_SPEED state
        const val AT_SPEED_TOLERANCE = 0.05 // 5%

        const val BASE_RPM = 0.0
        const val RPM_PER_INCH = 0.0
    }
}
package org.firstinspires.ftc.teamcode.hardware

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Actors
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels
import kotlin.math.tan

class Limelight(hw: HardwareMap) : Subsystem() {
    val device = hw.get(Limelight3A::class.java, NAME)

    fun reset() {
        device.pipelineSwitch(PIPELINE_TAGS)
        device.start()
    }

    init { reset() }

    fun getRelocalizationPose(): Pose? {
        val result = device.latestResult
        if (result == null || !result.isValid) return null

        val botpose = result.botpose

        // Convert Meters to Inches for PedroPathing
        val xInches = botpose.position.x * 39.3701
        val yInches = botpose.position.y * 39.3701

        // Limelight Yaw is degrees, Pedro uses Radians
        val headingRad = Math.toRadians(botpose.orientation.yaw)

        return Pose(xInches, yInches, headingRad)
    }

    fun getDistanceFromGoal(): Double {
        val result = device.latestResult
        if (result == null || !result.isValid) return 0.0

        val ty = result.ty
        val angleToGoalRad = Math.toRadians(MOUNT_ANGLE + ty)

        val heightDiff = GOAL_HEIGHT - LENS_HEIGHT

        return heightDiff / tan(angleToGoalRad)
    }

    fun getAngleToGoal(): Double {
        val result = device.latestResult
        if (result == null || !result.isValid) return 0.0
        return result.tx
    }

    fun setPipeline(index: Int): Closure {
        return Channels.send<Message>(
            { SetPipeline(index) },
            { actor.tx }
        )
    }

    val stop: Closure = Channels.send<Message>(
        { Stop },
        { actor.tx }
    )

    val actor = Actors.actor<State, Message>(
        { State.TAGS },

        { _, message ->
            when (message) {
                is Stop -> {
                    State.STOPPED
                }
                is SetPipeline -> {
                    if (message.index == PIPELINE_TAGS) State.TAGS else State.SCORING
                }
            }
        },

        { stateRegister ->
            match(stateRegister)
                .branch(State.STOPPED, exec {
                    device.stop()
                })
                .branch(State.TAGS, exec {
                    device.start()
                    device.pipelineSwitch(PIPELINE_TAGS)
                })
                .branch(State.SCORING, exec {
                    device.start()
                    device.pipelineSwitch(PIPELINE_SCORING)
                })
                .assertExhaustive()
        }
    )

    sealed interface Message
    data object Stop : Message
    data class SetPipeline(val index: Int) : Message

    enum class State {
        STOPPED,
        TAGS,
        SCORING
    }

    companion object {
        const val NAME = "limelight"

        const val PIPELINE_TAGS = 0
        const val PIPELINE_SCORING = 1

        const val LENS_HEIGHT = 10.0 // Inches
        const val GOAL_HEIGHT = 30.0 // Inches
        const val MOUNT_ANGLE = 20.0 // Degrees
    }
}
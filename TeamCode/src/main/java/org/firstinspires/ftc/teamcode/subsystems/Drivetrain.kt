package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierPoint
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Actors
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.math.PI

class Drivetrain(hw: HardwareMap, private val telemetry: TelemetryManager) : Subsystem() {
    val follower: Follower = Constants.createFollower(hw)

    private var driveForward = 0.0
    private var driveStrafe = 0.0
    private var driveTurn = 0.0

    private var targetHeading = 0.0

    fun reset() {
        follower.startTeleopDrive()
        driveForward = 0.0
        driveStrafe = 0.0
        driveTurn = 0.0
    }

    init {
        follower.update()
        reset()
    }

    private fun calculateHeadingPower(currentHeading: Double, targetHeading: Double): Double {
        var error = targetHeading - currentHeading

        while (error > PI) error -= 2 * PI
        while (error < -PI) error += 2 * PI

        return (error * HEADING_P).coerceIn(-1.0, 1.0)
    }

    fun drive(forward: () -> Double, strafe: () -> Double, turn: () -> Double): Closure {
        return Channels.send<Message>(
            { SetDrive(forward(), strafe(), turn()) },
            { actor.tx }
        )
    }

    fun driveHeadingLock(forward: () -> Double, strafe: () -> Double, targetHeadingRad: () -> Double): Closure {
        return Channels.send<Message>(
            { SetHeadingLock(forward(), strafe(), targetHeadingRad()) },
            { actor.tx }
        )
    }

    fun followPath(pathChain: () -> PathChain): Closure {
        return Channels.send<Message>(
            { FollowPath(pathChain()) },
            { actor.tx }
        )
    }

    val holdPosition: Closure = Channels.send<Message>(
        { HoldPosition },
        { actor.tx }
    )

    fun relocalize(pose: () -> Pose): Closure {
        return Channels.send<Message>(
            { Relocalize(pose()) },
            { actor.tx }
        )
    }

    val stop: Closure = Channels.send<Message>(
        { Stop },
        { actor.tx }
    )

    val actor = Actors.actor<State, Message>(
        { State.TELEOP },

        { _, message ->
            when (message) {
                is Stop -> {
                    driveForward = 0.0; driveStrafe = 0.0; driveTurn = 0.0
                    follower.startTeleopDrive()
                    State.TELEOP
                }

                is SetDrive -> {
                    driveForward = message.forward
                    driveStrafe = message.strafe
                    driveTurn = message.turn

                    follower.startTeleopDrive()
                    State.TELEOP
                }

                is SetHeadingLock -> {
                    driveForward = message.forward
                    driveStrafe = message.strafe
                    targetHeading = message.targetHeading

                    follower.startTeleopDrive()
                    State.HEADING_LOCK
                }

                is HoldPosition -> {
                    val currentPose = follower.pose
                    val holdPoint =
                        BezierPoint(Pose(currentPose.x, currentPose.y))
                    val path = Path(holdPoint)
                    path.setConstantHeadingInterpolation(currentPose.heading)

                    follower.followPath(path, true)
                    State.HOLD_POSITION
                }

                is Relocalize -> {
                    follower.pose = message.pose
                    State.TELEOP
                }

                is FollowPath -> {
                    follower.followPath(message.path)
                    State.AUTO
                }
            }
        },

        { stateRegister ->
            match(stateRegister)
                .branch(State.TELEOP, exec {
                    follower.update()
                    follower.setTeleOpDrive(driveForward, driveStrafe, driveTurn, false)
                })
                .branch(State.HEADING_LOCK, exec {
                    follower.update()
                    val turnPower = calculateHeadingPower(follower.pose.heading, targetHeading)
                    follower.setTeleOpDrive(driveForward, driveStrafe, turnPower, false)
                })
                .branch(State.HOLD_POSITION, exec {
                    follower.update()
                })
                .branch(State.AUTO, exec {
                    follower.update()
                })
                .assertExhaustive()
        }
    )

    sealed interface Message
    data object Stop : Message
    data object HoldPosition : Message
    data class SetDrive(val forward: Double, val strafe: Double, val turn: Double) : Message
    data class SetHeadingLock(val forward: Double, val strafe: Double, val targetHeading: Double) : Message
    data class Relocalize(val pose: Pose) : Message
    data class FollowPath(val path: PathChain) : Message

    enum class State {
        TELEOP,
        HEADING_LOCK,
        HOLD_POSITION,
        AUTO
    }

    companion object {
        const val HEADING_P = 1.0
    }
}
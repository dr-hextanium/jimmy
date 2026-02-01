package org.firstinspires.ftc.teamcode.command.auto

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.command.CommandTemplate

open class PedroCommand(
    val path: PathChain,
    val follower: Follower,
    val holdEnd: Boolean = true,
    val maxPower: Double = 1.0
) : CommandTemplate() {
    override fun initialize() {
        follower.setMaxPower(maxPower)
        follower.followPath(path, holdEnd)
    }

    override fun execute() = follower.update()

    override fun isFinished() = !follower.isBusy
}
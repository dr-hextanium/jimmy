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

    // The follower is advanced exactly once per loop in Robot.read(); this command only needs to
    // start the path (initialize) and report completion (isFinished). Calling follower.update() here
    // too would re-read the Pinpoint (extra I2C) and re-run path PIDF against a barely-changed pose.
    override fun execute() {}

    override fun isFinished() = !follower.isBusy
}
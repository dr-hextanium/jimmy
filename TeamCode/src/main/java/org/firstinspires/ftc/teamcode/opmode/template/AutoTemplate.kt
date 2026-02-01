package org.firstinspires.ftc.teamcode.opmode.template

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.hardware.Globals
import org.firstinspires.ftc.teamcode.hardware.Robot

open class AutoTemplate(val start: Pose) : BaseTemplate() {
    val follower
        get() = Robot.follower

    override fun init() {
        Globals.AUTO = true
        super.init()
    }

    override fun initialize() {
        follower.setStartingPose(start)
    }

    override fun cycle() {
        follower.update()

        telemetry.addData("x", follower.pose.x)
        telemetry.addData("y", follower.pose.y)
        telemetry.addData("heading", follower.pose.heading)
    }
}
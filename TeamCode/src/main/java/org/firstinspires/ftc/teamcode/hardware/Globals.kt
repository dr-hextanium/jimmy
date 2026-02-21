package org.firstinspires.ftc.teamcode.hardware

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector

@Configurable
object Globals {
    @JvmField
	var AUTO = false

    @JvmField
    var AUTO_RECOVERY_POSITION: Pose? = null

    @JvmField
    var isRed: Boolean? = null

    @JvmField
    val RED_GOAL_POSE = Vector(144.0, 144.0)

    @JvmField
    val BLUE_GOAL_POSE = Vector(0.0, 144.0)

    @JvmField
    val RED_RESET_POSE = Vector(136.0, 9.25)

    @JvmField
    val BLUE_RESET_POSE = Vector(8.0, 9.25)
}
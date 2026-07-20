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

    var globalHeadingOffset: Double = 0.0

    @JvmField
    val RED_GOAL_POSE = Vector(Pose(144.0, 144.0))

    @JvmField
    val BLUE_GOAL_POSE = Vector(Pose(0.0, 144.0))

    @JvmField
    val BLUE_RESET_POSE = Pose(27.0, 131.5, Math.toRadians(143.0))

    @JvmField
    val RED_RESET_POSE = Pose(117.0, 131.5, Math.toRadians(37.0))

    // Base-zone relocalization poses: where the robot physically sits, and the field-frame heading it
    // faces, when placed in its own base zone. x/y mirror the base-zone reference points in Zones.kt;
    // heading is the base-zone facing (RED faces +x = 0, BLUE faces -x = 180deg). The LEFT_STICK
    // relocalize snaps the localizer here. CONFIRM x/y/heading against the real field (TODO.md).
    @JvmField
    val RED_BASE_POSE = Pose(38.5, 33.5, 0.0)

    @JvmField
    val BLUE_BASE_POSE = Pose(105.5, 33.5, Math.toRadians(180.0))
}
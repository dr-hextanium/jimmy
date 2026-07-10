package org.firstinspires.ftc.teamcode.hardware

import com.skeletonarmy.marrow.zones.Point
import com.skeletonarmy.marrow.zones.PolygonZone

object Zones {
    val CLOSE_LAUNCH_ZONE = PolygonZone(Point(144.0, 144.0), Point(72.0, 72.0), Point(0.0, 144.0))
    val FAR_LAUNCH_ZONE = PolygonZone(Point(48.0, 0.0), Point(72.0, 24.0), Point(96.0, 0.0))

    val RED_BASE_ZONE = PolygonZone(Point(38.5, 33.5), 20.0, 20.0)
    val BLUE_BASE_ZONE = PolygonZone(Point(105.5, 33.5), 20.0, 20.0)

    val RED_GOAL_ZONE = PolygonZone(Point(137.0, 120.0), Point(137.0, 143.0), Point(120.0, 143.0))
    val BLUE_GOAL_ZONE = PolygonZone(Point(7.0, 120.0), Point(7.0, 143.0), Point(24.0, 143.0))
}
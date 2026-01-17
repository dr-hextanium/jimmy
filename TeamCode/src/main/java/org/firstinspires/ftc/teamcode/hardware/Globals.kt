package org.firstinspires.ftc.teamcode.hardware

import com.bylazar.configurables.annotations.Configurable

data class PowerDelayPair(val power: Double, val delay: Long)

@Configurable
object Globals {
    @JvmField
	var AUTO = false

    @JvmField
    val CLOSE_DEPOT = Triple(
        PowerDelayPair(0.875, 150L),
        PowerDelayPair(0.9, 150L),
        PowerDelayPair(0.885, 50L),
    )

    @JvmField
    val CLOSE_APEX = Triple(
        PowerDelayPair(0.71, 150L),
        PowerDelayPair(0.71, 150L),
        PowerDelayPair(0.71, 1300L),
    )

    @JvmField
    val FAR = Triple(
        PowerDelayPair(0.875, 150L),
        PowerDelayPair(0.9, 150L),
        PowerDelayPair(0.885, 1300L),
    )
}
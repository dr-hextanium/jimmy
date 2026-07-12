package org.firstinspires.ftc.teamcode.testfakes

import org.firstinspires.ftc.teamcode.control.TimeSource

/**
 * A hand-advanced [TimeSource] for unit tests. Starts at 0.0; [advance] steps the clock forward by
 * a fixed dt so profiled/integral control can be exercised one deterministic tick at a time.
 */
class FakeTimeSource(var now: Double = 0.0) : TimeSource {
    override fun seconds(): Double = now

    /** Advance the clock by [dt] seconds and return the new time. */
    fun advance(dt: Double): Double {
        now += dt
        return now
    }
}

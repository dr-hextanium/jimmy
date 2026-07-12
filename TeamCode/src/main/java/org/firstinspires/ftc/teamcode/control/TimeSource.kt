package org.firstinspires.ftc.teamcode.control

/**
 * A monotonic clock in seconds.
 *
 * Control code that integrates over time (motion profiles, derivative/integral terms) needs a
 * notion of "now". Taking that time through this injectable interface -- rather than calling
 * [System.nanoTime] directly inside a subsystem -- lets JVM unit tests drive the loop with a
 * deterministic, hand-advanced clock instead of real wall-clock time.
 *
 * On the robot, use [SYSTEM]. In tests, supply a fake that returns whatever value the test sets.
 */
fun interface TimeSource {
    /** The current time, in seconds. Only differences between successive calls are meaningful. */
    fun seconds(): Double

    companion object {
        /** Real monotonic clock backed by [System.nanoTime]. */
        val SYSTEM = TimeSource { System.nanoTime() / 1e9 }
    }
}

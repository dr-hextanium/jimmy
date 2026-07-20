package org.firstinspires.ftc.teamcode.control

import kotlin.math.abs

/**
 * Finds the "knee" (point of diminishing returns) of a trade-off curve — for the launcher tuner, the
 * curve of spin-up time (y) versus current limit (x). Raising the current budget buys a big time
 * reduction at first, then flattens out; the knee is where the extra current stops meaningfully
 * shortening the spin-up, and is a sensible default operating point.
 *
 * Method: normalise both axes to `[0,1]` (so the two units — amps and seconds — are comparable), then
 * return the point farthest from the straight chord joining the first and last points. That is the
 * corner of the "L", i.e. the maximum-curvature point, and works whether the curve bows below the
 * chord (convex, the usual time-vs-current shape) or above it.
 *
 * Pure and unit-tested. Degenerate inputs (fewer than 3 points, or a zero range on either axis where
 * no interior knee is defined) fall back to the last point — for this curve the highest current limit,
 * i.e. the fastest spin-up, which is the safe "no useful knee found" default.
 */
object KneeFinder {
    data class Point(val x: Double, val y: Double)

    data class Knee(val x: Double, val y: Double, val index: Int, val welldefined: Boolean)

    fun findKnee(points: List<Point>): Knee {
        require(points.isNotEmpty()) { "need at least one point" }

        val sorted = points.withIndex().sortedBy { it.value.x }
        val last = sorted.last()
        val fallback = Knee(last.value.x, last.value.y, last.index, welldefined = false)
        if (sorted.size < 3) return fallback

        val xs = sorted.map { it.value.x }
        val ys = sorted.map { it.value.y }
        val xMin = xs.first()
        val xMax = xs.last()
        val yMin = ys.min()
        val yMax = ys.max()
        val xRange = xMax - xMin
        val yRange = yMax - yMin
        if (xRange <= 0.0 || yRange <= 0.0) return fallback

        // Normalised endpoints of the chord.
        val x1 = 0.0
        val y1 = (ys.first() - yMin) / yRange
        val x2 = 1.0
        val y2 = (ys.last() - yMin) / yRange
        val dxChord = x2 - x1
        val dyChord = y2 - y1

        var bestDist = -1.0
        var best = sorted[1] // some interior point
        // Only interior points can be a knee (endpoints are on the chord).
        for (i in 1 until sorted.size - 1) {
            val xn = (xs[i] - xMin) / xRange
            val yn = (ys[i] - yMin) / yRange
            // |cross product| of the chord vector with (point - chordStart); chord length is constant
            // across candidates, so the numerator alone ranks distance.
            val dist = abs(dxChord * (y1 - yn) - (x1 - xn) * dyChord)
            if (dist > bestDist) {
                bestDist = dist
                best = sorted[i]
            }
        }
        return Knee(best.value.x, best.value.y, best.index, welldefined = true)
    }
}

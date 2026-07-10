package org.firstinspires.ftc.teamcode.utility

class InterpLUT {
    // Stores the x,y pairs
    private val points = mutableListOf<Point>()
    // Tracks if the list has been sorted
    private var isBuilt = false

    private data class Point(val x: Double, val y: Double) : Comparable<Point> {
        // Compare points based on their X value to keep them sorted horizontally
        override fun compareTo(other: Point): Int = this.x.compareTo(other.x)
    }

    /**
     * Adds an x, y point to the LUT.
     * Returns the LUT instance so you can chain add() calls.
     */
    fun add(x: Double, y: Double): InterpLUT {
        points.add(Point(x, y))
        isBuilt = false // Reset build status since we modified the table
        return this
    }

    /**
     * Sorts the array. Must be called before querying the table.
     */
    fun createLUT() {
        points.sort()
        isBuilt = true
    }

    /**
     * Gets the interpolated Y value for the given X.
     * The `operator` keyword allows you to use array syntax: lut[x]
     */
    operator fun get(x: Double): Double {
        require(points.isNotEmpty()) { "LUT must have at least one point added before querying." }

        // Auto-build if the user forgot to call createLUT()
        if (!isBuilt) {
            createLUT()
        }

        // --- FAILSAFES (Out of bounds) ---
        // If x is lower than our lowest point, return the lowest point's y
        if (x <= points.first().x) return points.first().y
        // If x is higher than our highest point, return the highest point's y
        if (x >= points.last().x) return points.last().y

        // --- INTERPOLATION ---
        // Binary search to efficiently find the two closest points (p1 and p2)
        var low = 0
        var high = points.size - 1

        while (low <= high) {
            val mid = (low + high) / 2
            val midX = points[mid].x

            if (midX == x) {
                return points[mid].y // Exact match found
            } else if (midX < x) {
                low = mid + 1
            } else {
                high = mid - 1
            }
        }

        // 'high' is the index of the closest point strictly less than x (p1)
        // 'low' is the index of the closest point strictly greater than x (p2)
        val p1 = points[high]
        val p2 = points[low]

        // Edge case prevention: if x values are exactly the same somehow, prevent divide-by-zero
        if (p1.x == p2.x) return p1.y

        // Linear interpolation formula: y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
        val slope = (p2.y - p1.y) / (p2.x - p1.x)
        return p1.y + slope * (x - p1.x)
    }
}
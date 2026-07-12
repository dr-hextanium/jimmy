package org.firstinspires.ftc.teamcode.pedroPathing

import com.pedropathing.geometry.Pose
import org.hipparchus.linear.Array2DRowRealMatrix
import org.hipparchus.linear.ArrayRealVector
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresBuilder
import org.hipparchus.optim.nonlinear.vector.leastsquares.LevenbergMarquardtOptimizer
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

// Internal helper for math
data class Vec2(val x: Double, val y: Double)

fun getPedroPathCode(
    startPose: Pose,
    artifactPoses: List<Pose>,
    intakeOffset: Double
): String {
    val optimizer = LevenbergMarquardtOptimizer()

    // 3 segments, each needing 2 control points (P1, P2) = 12 variables (x,y for each)
    val initialGuess = DoubleArray(12)
    val target = DoubleArray(3)

    val problem = LeastSquaresBuilder()
        .maxIterations(100)
        .start(initialGuess)
        .target(target)
        .model { params ->
            val residuals = DoubleArray(3)
            for (i in 0 until 3) {
                val p1 = Vec2(params.getEntry(i * 4), params.getEntry(i * 4 + 1))
                val p2 = Vec2(params.getEntry(i * 4 + 2), params.getEntry(i * 4 + 3))
                val art = Vec2(artifactPoses[i].x, artifactPoses[i].y)

                // Math to find required robot center based on heading
                val angle = atan2(art.y - p2.y, art.x - p2.x)
                val targetX = art.x - intakeOffset * cos(angle)
                val targetY = art.y - intakeOffset * sin(angle)

                // The residual is the distance error
                residuals[i] = hypot(targetX - p2.x, targetY - p2.y)
            }
            org.hipparchus.util.Pair(ArrayRealVector(residuals), Array2DRowRealMatrix(3, 12))
        }
        .build()

    val result = optimizer.optimize(problem).point.toArray()

    val sb = StringBuilder()
    sb.append("\n// --- COPY PASTE INTO YOUR PATHCHAIN ---\n")

    var lastPose = "new Pose(${startPose.x}, ${startPose.y}, ${startPose.heading})"

    for (i in 0 until 3) {
        val p1x = result[i * 4]
        val p1y = result[i * 4 + 1]
        val p2x = result[i * 4 + 2]
        val p2y = result[i * 4 + 3]

        val angle = atan2(artifactPoses[i].y - p2y, artifactPoses[i].x - p2x)
        val p3x = artifactPoses[i].x - intakeOffset * cos(angle)
        val p3y = artifactPoses[i].y - intakeOffset * sin(angle)

        sb.append(".addPath(\n")
        sb.append("    new BackendCurve(\n") // Pedro uses Pose for Bezier handles in some versions
        sb.append(String.format("        new Pose(%.2f, %.2f, %.2f),\n", startPose.x, startPose.y, startPose.heading))
        sb.append(String.format("        new Pose(%.2f, %.2f),\n", p1x, p1y))
        sb.append(String.format("        new Pose(%.2f, %.2f),\n", p2x, p2y))
        sb.append(String.format("        new Pose(%.2f, %.2f, %.2f)\n", p3x, p3y, angle))
        sb.append("    )\n")
        sb.append(").setConstantHeadingInterpolation(${angle})\n")
    }

    return sb.toString()
}
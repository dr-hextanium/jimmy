package org.firstinspires.ftc.teamcode.utility

import kotlin.math.absoluteValue

fun percentDifference(a: Double, b: Double): Double {
    return (a - b) / ((a + b) / 2)
}

fun absPercentDifference(a: Double, b: Double): Double {
    return percentDifference(a, b).absoluteValue
}
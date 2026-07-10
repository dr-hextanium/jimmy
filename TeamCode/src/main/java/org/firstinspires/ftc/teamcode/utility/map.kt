package org.firstinspires.ftc.teamcode.utility

fun map(
    input: Double,
    inputMin: Double,
    inputMax: Double,
    outputMin: Double,
    outputMax: Double
): Double {
    // Prevent divide-by-zero if the input min and max are exactly the same
    if (inputMin == inputMax) return outputMin

    // Calculate the scaled proportion and apply it to the output range
    return outputMin + (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin)
}

fun mapClamped(
    input: Double,
    inputMin: Double,
    inputMax: Double,
    outputMin: Double,
    outputMax: Double
): Double {
    if (inputMin == inputMax) return outputMin

    val mappedValue = outputMin + (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin)

    // Clamp the result to ensure it stays strictly within the output bounds
    val minOut = minOf(outputMin, outputMax)
    val maxOut = maxOf(outputMin, outputMax)

    return mappedValue.coerceIn(minOut, maxOut)
}

//fun Double.map(inMin: Double, inMax: Double, outMin: Double, outMax: Double): Double {
//    if (inMin == inMax) return outMin
//    return outMin + (this - inMin) * (outMax - outMin) / (inMax - inMin)
//}
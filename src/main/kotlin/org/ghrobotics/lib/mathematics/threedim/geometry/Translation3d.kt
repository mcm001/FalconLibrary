package org.ghrobotics.lib.mathematics.threedim.geometry

import org.ghrobotics.lib.mathematics.epsilonEquals

/**
 * +X to the right
 * +Y straight up
 * +Z axis toward viewer
 */
data class Translation3d(
    val x: Double,
    val y: Double,
    val z: Double
) {

    val magnitude get() = Math.sqrt(sqrMagnitude)
    val sqrMagnitude get() = x * x + y * y + z * z

    operator fun unaryMinus() = Translation3d(-x, -y, -z)

    operator fun get(componentId: Int) =
        when (componentId) {
            0 -> x
            1 -> y
            2 -> z
            else -> throw IndexOutOfBoundsException()
        }

    operator fun plus(other: Translation3d) =
        Translation3d(
            x + other.x,
            y + other.y,
            z + other.z
        )

    operator fun minus(other: Translation3d) = plus(-other)

    operator fun times(quaternion: Quaternion) = quaternion.transform(this)

    operator fun times(scalar: Double) = Translation3d(x * scalar, y * scalar, z * scalar)

    operator fun div(scalar: Double) = times(1/scalar)

    infix fun epsilonEquals(other: Translation3d) = x epsilonEquals other.x &&
        y epsilonEquals other.y && z epsilonEquals other.z

    companion object {
        val kZero = Translation3d(0.0, 0.0, 0.0)
    }
}
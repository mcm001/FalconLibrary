package org.ghrobotics.lib.mathematics.units

import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.kEpsilon

val Number.radian get() = Rotation2d(toDouble())
val Number.degree get() = Math.toRadians(toDouble()).radian

class Rotation2d : SIUnit<Rotation2d> {

    override val value: Double
    val cos: Double
    val sin: Double

    constructor(value: Double) {

        val x = Math.cos(value)
        val y = Math.sin(value)

        val magnitude = Math.hypot(x, y)
        if (magnitude > kEpsilon) {
            sin = y / magnitude
            cos = x / magnitude
        } else {
            sin = 0.0
            cos = 1.0
        }

        this.value = value

    }

    constructor(x: Double, y: Double, normalize: Boolean) {
        if (normalize) {
            val magnitude = Math.hypot(x, y)
            if (magnitude > kEpsilon) {
                sin = y / magnitude
                cos = x / magnitude
            } else {
                sin = 0.0
                cos = 1.0
            }
        } else {
            cos = x
            sin = y
        }
        value = Math.atan2(sin, cos)
    }

    val radian get() = value % (Math.PI * 2)
    val degree get() = Math.toDegrees(radian)

    val unboundedRadian get() = value
    val unboundedDegree get() = Math.toDegrees(unboundedRadian)

    fun distance(other: Rotation2d): Double {
        return inverse().rotateBy(other).radian
    }

    /**
     * @return The pole nearest to this rotation.
     */
    fun nearestPole(): Rotation2d {
        var pole_sin = 0.0
        var pole_cos = 0.0
        if (Math.abs(cos) > Math.abs(sin)) {
            pole_cos = Math.signum(cos)
            pole_sin = 0.0
        } else {
            pole_cos = 0.0
            pole_sin = Math.signum(sin)
        }
        return Rotation2d(pole_cos, pole_sin, false)
    }

    /**
     * We can rotate this Rotation2d by adding together the effects of it and another rotation.
     *
     * @param other The other rotation. See: https://en.wikipedia.org/wiki/Rotation_matrix
     * @return This rotation rotated by other.
     */
    fun rotateBy(other: Rotation2d): Rotation2d {
        return Rotation2d(cos * other.cos - sin * other.sin,
                cos * other.sin + sin * other.cos, true)
    }

    fun isParallel(rotation: Rotation2d) = (this - rotation).radian epsilonEquals 0.0

    override fun plus(other: Rotation2d): Rotation2d {
        return Rotation2d(
            cos * other.cos - sin * other.sin,
            cos * other.sin + sin * other.cos,
            true
        )
    }

    override fun minus(other: Rotation2d) = plus(-other)

    override fun createNew(newValue: Double) = Rotation2d(newValue)

    override fun equals(other: Any?) = other is Rotation2d && this.value epsilonEquals other.value

    override fun hashCode() = this.value.hashCode()

    fun inverse() = Rotation2d(cos, -sin, false)

    companion object {
        val kZero = Rotation2d(0.0)
        val kRotation = 360.degree
    }
}
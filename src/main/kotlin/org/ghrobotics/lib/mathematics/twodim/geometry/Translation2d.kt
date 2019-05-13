/*
 * FRC Team 5190
 * Green Hope Falcons
 */

/*
 * Some implementations and algorithms borrowed from:
 * NASA Ames Robotics "The Cheesy Poofs"
 * Team 254
 */

@file:Suppress("KDocUnresolvedReference", "EqualsOrHashCode")

package org.ghrobotics.lib.mathematics.twodim.geometry

import com.team1323.lib.util.Util
import org.ghrobotics.lib.mathematics.lerp
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.types.VaryInterpolatable
import com.team254.lib.geometry.Translation2d.dot
import org.apache.commons.math3.geometry.euclidean.threed.Rotation
import org.ghrobotics.lib.mathematics.kEpsilon


fun Rotation2d.toTranslation() = Translation2d(cos, sin)

data class Translation2d constructor(
    val x: Double,
    val y: Double
) : VaryInterpolatable<Translation2d> {

    constructor() : this(0.0, 0.0)

    constructor(start: Translation2d, end: Translation2d) : this(
            end.x - start.x,
            end.y - start.y
    )

    constructor(
        x: Length = 0.meter,
        y: Length = 0.meter
    ) : this(x.value, y.value)

    // Vector to Translation3d
    constructor(
        distance: Length = 0.meter,
        rotation: Rotation2d = 0.degree
    ) : this(distance * rotation.cos, distance * rotation.sin)

    val norm get() = Math.hypot(x, y)

    override fun interpolate(endValue: Translation2d, t: Double) = when {
        t <= 0 -> this
        t >= 1 -> endValue
        else -> Translation2d(
            x.lerp(endValue.x, t),
            y.lerp(endValue.y, t)
        )
    }

    override fun distance(other: Translation2d): Double {
        val x = this.x - other.x
        val y = this.y - other.y
        return Math.hypot(x, y)
    }

    operator fun plus(other: Translation2d) = Translation2d(x + other.x, y + other.y)
    operator fun minus(other: Translation2d) = Translation2d(x - other.x, y - other.y)

    operator fun times(other: Rotation2d) = Translation2d(
        x * other.cos - y * other.sin,
        x * other.sin + y * other.cos
    )

    operator fun times(other: Number): Translation2d {
        val factor = other.toDouble()
        return Translation2d(x * factor, y * factor)
    }

    operator fun div(other: Number): Translation2d {
        val factor = other.toDouble()
        return Translation2d(x / factor, y / factor)
    }

    operator fun unaryMinus() = Translation2d(-x, -y)

    val inverse
        get() = Translation2d(-x, -y)

    companion object {
        fun dot(a: Translation2d, b: Translation2d): Double {
            return a.x * b.x + a.y * b.y
        }
        fun fromRotation2d(angle: Double) : Translation2d = Translation2d(Math.cos(angle), Math.sin(angle))
        fun fromRotation2d(angle: Double, magnitude: Double) : Translation2d = fromRotation2d(angle) * magnitude
    }

    /**
     * https://stackoverflow.com/a/1167047/6627273
     * A point D is considered "within" an angle ABC when
     * cos(DBM) > cos(ABM)
     * where M is the midpoint of AC, so ABM is half the angle ABC.
     * The cosine of an angle can be computed as the dot product of two normalized
     * vectors in the directions of its sides.
     * Note that this definition of "within" does not include points that lie on
     * the sides of the given angle.
     * If `vertical` is true, then check not within the given angle, but within the
     * image of that angle rotated by pi about its vertex.
     *
     * @param Translation2d A
     * A point on one side of the angle.
     * @param Translation2d B
     * The vertex of the angle.
     * @param Translation2d C
     * A point on the other side of the angle.
     * @param boolean vertical
     * Whether to check in the angle vertical to the one given
     * @return Whether this translation is within the given angle.
     * @author Joseph Reed
     */
    fun isWithinAngle(A: Translation2d, B: Translation2d, C: Translation2d, vertical: Boolean): Boolean {
        val M = A.interpolate(C, 0.5) // midpoint
        var m = Translation2d(B, M).normalize() // mid-vector
        var a = Translation2d(B, A).normalize() // side vector
        val d = Translation2d(B, this).normalize() // vector to here
        if (vertical) {
            m = m.inverse
            a = a.inverse
        }
        return Translation2d.dot(d, m) > Translation2d.dot(a, m)
    }

    /** Assumes an angle centered at the origin.  */
    fun isWithinAngle(A: Translation2d, C: Translation2d, vertical: Boolean): Boolean {
        return isWithinAngle(A, Translation2d(), C, vertical)
    }
    fun isWithinAngle(A: Translation2d, C: Translation2d): Boolean {
        return isWithinAngle(A, C, false)
    }

    fun normalize(): Translation2d {
        return if (epsilonEquals(Translation2d(), kEpsilon)) this else this * (1.0 / norm)
    }

    fun epsilonEquals(other: Translation2d, epsilon: Double): Boolean {
        return Util.epsilonEquals(x, other.x, epsilon) && Util.epsilonEquals(y, other.y, epsilon)
    }

    infix fun equals(other: Translation2d): Boolean {
        return if (other == null || other !is Translation2d) false else distance(other) < kEpsilon
    }

    val direction: Rotation2d = Rotation2d(x, y, true)

    infix fun rotateBy(rotation: Rotation2d) = Translation2d(x * rotation.cos - y * rotation.sin, x * rotation.sin + y * rotation.cos)

}
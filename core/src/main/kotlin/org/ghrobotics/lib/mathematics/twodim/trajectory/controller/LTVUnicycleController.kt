package org.ghrobotics.lib.mathematics.twodim.trajectory.controller

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.trajectory.Trajectory
import org.ejml.simple.SimpleMatrix
import kotlin.math.*

/**
 * A cascaded Linear time-varying unicycle controller. See Theorem 8.7.2
 * from https://github.com/calcmogul/controls-engineering-in-frc
 */
class LTVUnicycleController(
    private val kX: Double,
    private val kY_0: Double,
    private val kY_1: Double,
    private val kTheta: Double
) {

    private var poseError = Pose2d()
    var poseTolerance = Pose2d()

//    fun reset() {
//
//    }

//    fun atReference(): Boolean {
//        val eTranslate = this.poseError.translation
//        val eRotate = this.poseError.rotation
//        val tolTranslate = this.poseTolerance.translation
//        val tolRotate = this.poseTolerance.rotation
//        return abs(eTranslate.x) < tolTranslate.x && abs(eTranslate.y) < tolTranslate.y && abs(eRotate.radians) < tolRotate.radians
//    }

    fun calculate(currentPose: Pose2d, poseRef: Pose2d, linearVelocityRefMetersPerSec: Double, angularVelocityRefRadiansPerSecond: Double): ChassisSpeeds {
        this.poseError = poseRef.relativeTo(currentPose)
        val eX = this.poseError.translation.x
        val eY = this.poseError.translation.y
        val eTheta = this.poseError.rotation.radians
        val heading = currentPose.rotation.radians

//        // Rotate error into robot reference frame
//        val rotationMatrix = SimpleMatrix(3, 3, true, doubleArrayOf(
//            cos(heading), sin(heading), 0.0,
//            -sin(heading), cos(heading), 0.0,
//            0.0, 0.0, 1.0
//        ))

        val error = SimpleMatrix(3, 1, false, doubleArrayOf(
            eX, eY, eTheta
        ))

        val rotatedError = error// rotationMatrix.mult(error)


        val u = K(linearVelocityRefMetersPerSec).mult(rotatedError)

        val string = K(linearVelocityRefMetersPerSec).toString()

        val u0 = u[0]
        val u1 = u[1]

        val x = rotatedError[0]
        val y = rotatedError[1]
        val t = rotatedError[2]

        return ChassisSpeeds(u[0] + linearVelocityRefMetersPerSec,
            0.0, u[1] + angularVelocityRefRadiansPerSecond)
    }

    fun K(velocity: Double) = SimpleMatrix(2, 3, true, doubleArrayOf(
        kX, 0.0, 0.0,
        0.0, kY(velocity).withSign(velocity), kTheta * sqrt(velocity.absoluteValue)
    ))

    fun kY(velocity: Double): Double {
        return kY_0 + (kY_1 - kY_0) * sqrt(velocity.absoluteValue)
    }

    fun calculate(currentPose: Pose2d, desiredState: Trajectory.State) =
        calculate(currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond,
            desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter)

}
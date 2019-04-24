package org.ghrobotics.lib.components

import org.ghrobotics.lib.mathematics.threedim.geometry.Pose3d
import org.ghrobotics.lib.mathematics.threedim.geometry.Quaternion
import org.ghrobotics.lib.mathematics.threedim.geometry.Transform
import org.ghrobotics.lib.mathematics.threedim.geometry.Translation3d
import org.ghrobotics.lib.mathematics.units.Rotation2d

abstract class ArmComponent(
        // the position of the axle relative to the *local* coordinate system
        // this is a cartesian Translation3d
        val armAxleOffset: Translation3d,
        // the rotational axis of the arm relative to the *local* coordiante system
        val armRotationAxis: Translation3d
) : MotorComponent<Rotation2d>() {

    abstract val armKg: Double

    override fun updateState() {

        localTransform = Pose3d(
            armAxleOffset,
            Quaternion.fromAxisAngle(position, armRotationAxis)
        )

        localVelocityTransform = Transform(
            Translation3d.kZero,
            Quaternion.fromAxisAngle(velocity, armRotationAxis)
        )

        super.updateState()
    }

    override fun useState() {

        var experiencedAcceleration = 9.80665
        val parent = this.parent
        if (parent != null) {
            // Add the acceleration of all the components leading up to the arm
            experiencedAcceleration += (parent.worldAccelerationTransform.translation * worldTransform.rotation).y
        }

        arbitraryFeedForward = armKg * Math.cos(position) * experiencedAcceleration

        super.useState()
    }

}


package org.ghrobotics.lib.physics

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.degree
import java.util.*

@Suppress("unused")
class SwerveDriveKinematics(
        val kModulePositions: List<Translation2d>
) {

    var moduleRelativePositions: List<Translation2d> = kModulePositions


    init {
        setCenterOfRotation(Translation2d())
    }

    fun SwerveInverseKinematics() {
        setCenterOfRotation(Translation2d())
    }

    private val kNumberOfModules = 4

    //  private var moduleRelativePositions = Constants.kModulePositions
    private var moduleRotationDirections = updateRotationDirections()

    private fun updateRotationDirections(): List<Translation2d> {
        val directions = ArrayList<Translation2d>(kNumberOfModules)
        for (i in 0 until kNumberOfModules) {
            directions.add(moduleRelativePositions[i].rotateBy(90.degree))
        }
        return directions
    }

    fun setCenterOfRotation(center: Translation2d) {
        val positions = ArrayList<Translation2d>(kNumberOfModules)
        var maxMagnitude = 0.0
        for (i in 0 until kNumberOfModules) {
            val position = kModulePositions[i] + center.inverse
            positions.add(position)
            val magnitude = position.norm
            if (magnitude > maxMagnitude) {
                maxMagnitude = magnitude
            }
        }
        for (i in 0 until kNumberOfModules) {
            val position = positions[i]
            positions[i] = position * (1.0 / maxMagnitude)
        }
        moduleRelativePositions = positions
        moduleRotationDirections = updateRotationDirections()
    }

    fun updateDriveVectors(translationalVector: Translation2d, rotationalMagnitude: Double,
                           robotPose: Pose2d, robotCentric: Boolean): List<Translation2d> {
        var translationalVector = translationalVector
        SmartDashboard.putNumber("Vector Direction", translationalVector.direction.degree)
        //SmartDashboard.putNumber("Vector Magnitude", translationalVector.norm());
        SmartDashboard.putNumber("Robot Velocity", translationalVector.norm)

        if (!robotCentric)
            translationalVector = translationalVector.rotateBy(robotPose.rotation.inverse())
        val driveVectors = ArrayList<Translation2d>(kNumberOfModules)
        for (i in 0 until kNumberOfModules) {
            driveVectors.add(translationalVector + (moduleRotationDirections[i] * rotationalMagnitude))
        }
        var maxMagnitude = 1.0
        for (t in driveVectors) {
            val magnitude = t.norm
            if (magnitude > maxMagnitude) {
                maxMagnitude = magnitude
            }
        }
        for (i in 0 until kNumberOfModules) {
            val driveVector = driveVectors[i]
            driveVectors[i] = driveVector * (1.0 / maxMagnitude)
        }
        return driveVectors
    }

}
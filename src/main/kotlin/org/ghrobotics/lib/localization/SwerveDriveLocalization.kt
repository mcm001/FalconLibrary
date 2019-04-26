package org.ghrobotics.lib.localization

import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.components.SwerveModuleComponent
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import java.util.ArrayList



@Suppress("unused")
class SwerveDriveLocalization(
        robotHeading: Source<Rotation2d>,
        private val modules: List<SwerveModuleComponent>,
        localizationBuffer: TimeInterpolatableBuffer<Pose2d> = TimeInterpolatableBuffer()
) : Localization(robotHeading, localizationBuffer) {

    var pose = Pose2d()
    var distanceTraveled: Double = 0.0
    var currentVelocity = 0.0
    var lastUpdateTimestamp = 0.0

    override fun resetInternal(newPosition: Pose2d) {
        super.resetInternal(newPosition)
//        prevModuleHeading = moduleHeading().radian
//        prevModuleDistance = moduleDistance()
    }

    // Thank you to team 1323 for implementation!
    override fun update(deltaHeading: Rotation2d): Pose2d {
        val timestamp = Timer.getFPGATimestamp()
        var x = 0.0
        var y = 0.0
        val heading = robotHeading()

        var averageDistance = 0.0
        val distances = DoubleArray(4)
        for (m in modules) {
            m.updatePose(heading)
            val distance = (m.estimatedRobotPose.translation + pose.translation.inverse).norm
            distances[m.moduleID] = distance
            averageDistance += distance
        }
        averageDistance /= modules.size

        var minDevianceIndex = 0
        var minDeviance = 100.0
        val modulesToUse = ArrayList<SwerveModuleComponent>()
        for (m in modules) {
            val deviance = Math.abs(distances[m.moduleID] - averageDistance)
            if (deviance < minDeviance) {
                minDeviance = deviance
                minDevianceIndex = m.moduleID
            }
            if (deviance <= 0.01) {
                modulesToUse.add(m)
            }
        }

        if (modulesToUse.isEmpty()) {
            modulesToUse.add(modules.get(minDevianceIndex))
        }

        //SmartDashboard.putNumber("Modules Used", modulesToUse.size());

        for (m in modulesToUse) {
            x += m.estimatedRobotPose.translation.x
            y += m.estimatedRobotPose.translation.y
        }
        val updatedPose = Pose2d(Translation2d(x / modulesToUse.size, y / modulesToUse.size), heading)
        val deltaPos = (updatedPose.translation + pose.translation.inverse).norm
        distanceTraveled += deltaPos
        currentVelocity = deltaPos / (timestamp - lastUpdateTimestamp)
        pose = updatedPose
        modules.forEach { m -> m.resetPose(pose) }

        return pose
    }


}
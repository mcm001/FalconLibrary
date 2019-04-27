package org.ghrobotics.lib.components

import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.motors.FalconMotor
import com.team1323.lib.util.Util
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.mathematics.kEpsilon
import org.ghrobotics.lib.mathematics.threedim.geometry.Pose3d
import org.ghrobotics.lib.mathematics.threedim.geometry.Quaternion
import org.ghrobotics.lib.mathematics.threedim.geometry.Transform
import org.ghrobotics.lib.mathematics.threedim.geometry.Translation3d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import org.ghrobotics.lib.mathematics.units.radian
import org.ghrobotics.lib.physics.OblargianMotorModel
import org.ghrobotics.lib.utils.Source


class SwerveModuleComponent(
        val azumithMotor: FalconMotor<Rotation2d>,
        val driveMotor: FalconMotor<Length>,
        val azumithModel: OblargianMotorModel,
        val driveModel: OblargianMotorModel,
        val startingPosition: Translation2d,
        val moduleID: Int,
        val kWheelScrubFactors: Double,
        val kXScrubFactor: Double,
        val kYScrubFactor: Double,
        val kSimulateReversedCarpet: Boolean,
        val standardCarpetDirection: Boolean,
        val moduleAxis: Translation3d = Translation3d(0.0, 0.0, 1.0)
) : RobotComponent() {


    var angle : Rotation2d
        get() = azumithMotor.encoder.position.radian
        set(value) = azumithMotor.encoder.resetPosition(value.radian)

    val angularVelocity : Double // in rad/sec
        get() = azumithMotor.encoder.velocity

    val linearVelocity
        get() = driveMotor.encoder.velocity

    var driveDistance : Double
        get() = driveMotor.encoder.position
        set(value) = driveMotor.encoder.resetPosition(value)

    fun getFieldCentricAngle(robotHeading: Rotation2d): Rotation2d {
        val normalizedAngle = angle
        return normalizedAngle + (robotHeading)
    }

    var previousEncDistance: Double = 0.0
    var position = Translation2d()
    var estimatedRobotPose = Pose2d()

    @Synchronized
    fun updatePose(robotHeading: Rotation2d) {
        val currentEncDistance = driveDistance / SILengthConstants.kInchToMeter
        val deltaEncDistance = (currentEncDistance - previousEncDistance) * kWheelScrubFactors
        val currentWheelAngle = getFieldCentricAngle(robotHeading)
        var deltaPosition = Translation2d(currentWheelAngle.cos * deltaEncDistance,
                currentWheelAngle.sin * deltaEncDistance)

        var xScrubFactor = kXScrubFactor
        var yScrubFactor = kYScrubFactor
        if (kSimulateReversedCarpet) {
            if (Util.epsilonEquals(Math.signum(deltaPosition.x), 1.0, kEpsilon)) {
                if (standardCarpetDirection) {
                    xScrubFactor = 1.0 / kXScrubFactor
                } else {
                    xScrubFactor = 1.0
                }
            } else {
                if (standardCarpetDirection) {
                    xScrubFactor = kXScrubFactor * kXScrubFactor
                } else {

                }
            }
            if (Util.epsilonEquals(Math.signum(deltaPosition.y), 1.0, kEpsilon)) {
                if (standardCarpetDirection) {
                    yScrubFactor = 1.0 / kYScrubFactor
                } else {
                    yScrubFactor = 1.0
                }
            } else {
                if (standardCarpetDirection) {
                    yScrubFactor = kYScrubFactor * kYScrubFactor
                } else {

                }
            }
        } else {
            if (Util.epsilonEquals(Math.signum(deltaPosition.x), 1.0, kEpsilon)) {
                if (standardCarpetDirection) {
                    xScrubFactor = 1.0
                } else {

                }
            } else {
                if (standardCarpetDirection) {

                } else {
                    xScrubFactor = 1.0
                }
            }
            if (Util.epsilonEquals(Math.signum(deltaPosition.y), 1.0, kEpsilon)) {
                if (standardCarpetDirection) {
                    yScrubFactor = 1.0
                } else {

                }
            } else {
                if (standardCarpetDirection) {

                } else {
                    yScrubFactor = 1.0
                }
            }
        }

        deltaPosition = Translation2d(deltaPosition.x * xScrubFactor,
                deltaPosition.y * yScrubFactor)
        val updatedPosition = position + deltaPosition
        val staticWheelPose = Pose2d(updatedPosition, robotHeading)
        val robotPose = staticWheelPose.transformBy(Pose2d(startingPosition).inverse)
        position = updatedPosition
        estimatedRobotPose = robotPose
        previousEncDistance = currentEncDistance
    }

    @Synchronized
    fun resetPose(robotPose: Pose2d) {
        val modulePosition = robotPose.transformBy(Pose2d(startingPosition)).translation
        position = modulePosition
    }

    override val localTransformSource = {
        Pose3d(
                Translation3d.kZero,
                Quaternion.fromAxisAngle(angle.radian, moduleAxis)
        )
    }

    override var localVelocityTransformSource = {

        val moduleVelocityVector = Translation2d(linearVelocity, angle.radian)

        val toReturn = Pose3d(
                Translation3d(moduleVelocityVector.x, 0.0, moduleVelocityVector.y),
                Quaternion.fromAxisAngle(angle.radian, moduleAxis)
        )

        toReturn
    }

    open fun customizeWantedState(wantedState: State): State = wantedState

    var wantedState: State = State.Nothing
    var currentState: State = State.Nothing
        private set

    override fun useState() {

        val wantedState = customizeWantedState(wantedState)

        currentState = wantedState

        when (wantedState) {
            is State.Nothing -> {
                driveMotor.setNeutral()
            }
            is State.PercentOutput -> {
                azumithMotor.setPosition(wantedState.wantedState.angle, azumithFeedForward)
                driveMotor.setDutyCycle(wantedState.wantedState.demand, wantedState.wantedState.arbFF)
            }
            is State.Velocity -> {
                azumithMotor.setPosition(wantedState.wantedState.angle, azumithFeedForward)
                driveMotor.setVelocity(wantedState.wantedState.demand, wantedState.wantedState.arbFF)
            }
            is State.CustomState -> wantedState.update()
        }

        super.useState()
    }

    sealed class State {

        object Nothing: State()

        class PercentOutput(val wantedState: ModuleState) : State()

        class Velocity(val wantedState: ModuleState) : State()

        abstract class CustomState : State() {
            abstract fun update()
        }

    }

    val azumithFeedForward = azumithModel.getFeedForward(azumithMotor.encoder.velocity, localAccelerationTransform.rotation.eulerAngles.y)

    data class ModuleState(
            val angle: Double,
            val demand: Double,
            val arbFF: Double = 0.0
    )

}

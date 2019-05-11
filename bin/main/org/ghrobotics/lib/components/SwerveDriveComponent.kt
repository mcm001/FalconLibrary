package org.ghrobotics.lib.components

import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.localization.SwerveDriveLocalization
import org.ghrobotics.lib.mathematics.threedim.geometry.Pose3d
import org.ghrobotics.lib.mathematics.threedim.geometry.Quaternion
import org.ghrobotics.lib.mathematics.threedim.geometry.Translation3d
import org.ghrobotics.lib.physics.SwerveDriveKinematics
import org.ghrobotics.lib.utils.DeltaTime

abstract class SwerveDriveComponent(
        private val drivetrainHeightFromGround: Double
) : RobotComponent() {

    abstract val modules: List<SwerveModuleComponent>

    abstract val kinematics: SwerveDriveKinematics
    abstract val localization: SwerveDriveLocalization

    init {
        modules.forEach{
            this.addComponent(it)
        }
    }

    var wantedState: State = State.Nothing
    var currentState: State = State.Nothing
        private set

    open fun customizeWantedState(wantedState: State): State = wantedState

    private val loopDeltaTime = DeltaTime()

    override fun updateState() {

        val dt = loopDeltaTime.updateTime(Timer.getFPGATimestamp())

        val robotPose = localization.robotPosition

        val lastLocalTransform = localTransform

        localTransform = Pose3d(
            Translation3d(robotPose.translation.x, robotPose.translation.y, drivetrainHeightFromGround),
            Quaternion.fromEulerAngles(robotPose.rotation.radian, 0.0, 0.0)
        )

        localVelocityTransform = (localTransform - lastLocalTransform) / dt

        super.updateState()
    }

    override fun useState() {

        val wantedState = customizeWantedState(wantedState)

        currentState = wantedState
        when (wantedState) {
            is State.Nothing -> {
                modules.forEach{
                    it.wantedState = SwerveModuleComponent.State.Nothing
                }
            }
            is State.Velocity -> {
                modules[0].wantedState = SwerveModuleComponent.State.Velocity(wantedState.demandedInput.module0State)
                modules[1].wantedState = SwerveModuleComponent.State.Velocity(wantedState.demandedInput.module1State)
                modules[2].wantedState = SwerveModuleComponent.State.Velocity(wantedState.demandedInput.module2State)
                modules[3].wantedState = SwerveModuleComponent.State.Velocity(wantedState.demandedInput.module3State)
            }
            is State.PercentOutput -> {
                modules[0].wantedState = SwerveModuleComponent.State.PercentOutput(wantedState.demandedInput.module0State)
                modules[1].wantedState = SwerveModuleComponent.State.PercentOutput(wantedState.demandedInput.module1State)
                modules[2].wantedState = SwerveModuleComponent.State.PercentOutput(wantedState.demandedInput.module2State)
                modules[3].wantedState = SwerveModuleComponent.State.PercentOutput(wantedState.demandedInput.module3State)
            }
            is State.CustomState -> wantedState.update()
        }

        super.useState()
    }

    sealed class State {
        object Nothing : State()

        class Velocity(val demandedInput : SwerveDriveModulesState) : State()

        class PercentOutput(val demandedInput: SwerveDriveModulesState) : State()

        abstract class CustomState : State() {
            abstract fun update()
        }
    }

    data class SwerveDriveModulesState(
            val module0State: SwerveModuleComponent.ModuleState,
            val module1State: SwerveModuleComponent.ModuleState,
            val module2State: SwerveModuleComponent.ModuleState,
            val module3State: SwerveModuleComponent.ModuleState
    )

}
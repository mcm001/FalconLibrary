package org.ghrobotics.lib.components

import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.physics.OblargianMotorModel
import org.ghrobotics.lib.simulation.SimFalconMotor
import org.junit.Test

class SwerveModuleComponentTest {

    @Test
    fun testModuleTranforms() {

        val module = SwerveModuleComponent(
                SimFalconMotor<Rotation2d>(),
                SimFalconMotor<Length>(),
                OblargianMotorModel(1.0, 1.0, 1.0),
                OblargianMotorModel(1.0, 1.0, 1.0),
                Translation2d(0.5, 0.5),
                0,
                1.0,
                1.0,
                1.0,
                kSimulateReversedCarpet = true,
                standardCarpetDirection = true
        )

        println("module position ON INIT EULER ANGLES ${module.localTransform.rotation.eulerAngles}")

        module.updateState()

        module.useState()

        module.wantedState = SwerveModuleComponent.State.Velocity(SwerveModuleComponent.ModuleState(10.degree.radian, 1.toDouble(), 0.toDouble()))

        module.updateState()

        module.useState()

        module.updateState()

        module.useState()

        println("angle in degrees ${module.angle.degree}")

        println("module position EULER ANGLES ${module.localTransform.rotation.eulerAngles}")
        println("module velocity linear ${module.localVelocityTransform.translation} angular ${module.localVelocityTransform.rotation.eulerAngles}")
        println("module accel ${module.localAccelerationTransform}")

    }

}
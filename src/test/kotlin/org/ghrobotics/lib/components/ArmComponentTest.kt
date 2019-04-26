//package org.ghrobotics.lib.components
//
//import org.ghrobotics.lib.mathematics.threedim.geometry.Quaternion
//import org.ghrobotics.lib.mathematics.threedim.geometry.Translation3d
//import org.ghrobotics.lib.mathematics.units.Rotation2d
//import org.ghrobotics.lib.motors.FalconMotor
//import org.ghrobotics.lib.simulation.SimFalconMotor
//import org.junit.Test
//import kotlin.math.PI
//
//class ArmComponentTest {
//
//    @Test
//    fun testCoordinateSystem() {
//
//        val arm = SimArm(0.0, SimFalconMotor<Rotation2d>())
//
//        arm.wantedState = MotorComponent.State.Position(45.0 / 360.0 * 2.0 * PI)
//
//        arm.updateState()
//
//        arm.useState()
//
//        arm.updateState()
//
//        arm.useState()
//
//        arm.updateState()
//
////        println("arm local angle: " + arm.localTransform.rotation.eulerAngles)
////        println("expected angle: " + Quaternion.fromEulerAngles(0.0, 45.0 / 360.0 * 2.0 * PI, 0.0).eulerAngles)
//
//        assert( Quaternion.fromEulerAngles(0.0, 45.0 / 360.0 * 2.0 * PI, 0.0).eulerAngles epsilonEquals  arm.localTransform.rotation.eulerAngles)
//
//    }
//
//    class SimArm(
//            override val armKg: Double, override val motor: FalconMotor<Rotation2d>)
//        : ArmComponent(Translation3d.kZero, Translation3d(0.0, 0.0, 1.0)) {
//
//
//
//    }
//
//}
package org.ghrobotics.lib.physics

class OblargianMotorModel(val kV: Double, val kA: Double, val vIntercept: Double) {

    fun getFeedForward(currentVel: Double, currentAccel: Double) = kV * currentVel + kA * currentAccel + vIntercept * Math.signum(currentVel)

}
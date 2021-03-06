/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.motors

import edu.wpi.first.hal.SimDevice
import edu.wpi.first.hal.SimDouble
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel

abstract class AbstractFalconEncoder<K : SIKey>(
    val model: NativeUnitModel<K>,
    units: K,
    simName: String
) : FalconEncoder<K> {

    private val simDevice: SimDevice? = SimDevice.create(simName)
    private val simPositionHandle: SimDouble? = simDevice?.createDouble("Position, ${units::class.java.simpleName}s", false, 0.0)
    private val simVelocityHandle: SimDouble? = simDevice?.createDouble("Velocity, ${units::class.java.simpleName}s/s", false, 0.0)

    override fun setSimulatedPosition(simPosition: SIUnit<K>) {
        simPositionHandle?.set(simPosition.value)
    }

    override fun setSimulatedVelocity(simVelocity: SIUnit<Velocity<K>>) {
        simVelocityHandle?.set(simVelocity.value)
    }

    override val position: SIUnit<K>
        get() = if (simPositionHandle != null) SIUnit(simPositionHandle.get())
        else model.fromNativeUnitPosition(rawPosition)
    override val velocity: SIUnit<Velocity<K>>
        get() = if (simVelocityHandle != null) SIUnit(simVelocityHandle.get())
        else model.fromNativeUnitVelocity(rawVelocity)

    override fun resetPosition(newPosition: SIUnit<K>) {
        resetPositionRaw(model.toNativeUnitPosition(newPosition))
    }
}

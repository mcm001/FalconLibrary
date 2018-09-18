/*
 * FRC Team 5190
 * Green Hope Falcons
 */

/*
 * Some implementations and algorithms borrowed from:
 * NASA Ames Robotics "The Cheesy Poofs"
 * Team 254
 */

@file:Suppress("unused")

package org.ghrobotics.lib.mathematics.twodim.trajectory

import org.ghrobotics.lib.mathematics.State
import org.ghrobotics.lib.mathematics.twodim.trajectory.view.TrajectoryView

class TrajectoryIterator<S : State<S>>(private val view: TrajectoryView<S>) {

    private var progress = 0.0
    private var sample: TrajectorySamplePoint<S>


    val isDone: Boolean
        get() = remainingProgress == 0.0

    val remainingProgress: Double
        get() = Math.max(0.0, view.lastInterpolant - progress)

    val state: S
        get() = sample.state

    init {
        sample = view.sample(view.firstInterpolant)
        progress = view.firstInterpolant
    }

    fun advance(additional_progress: Double): TrajectorySamplePoint<S> {
        progress = Math.max(view.firstInterpolant,
                Math.min(view.lastInterpolant, progress + additional_progress))
        sample = view.sample(progress)

        return sample
    }

    fun preview(additional_progress: Double): TrajectorySamplePoint<S> {
        val progress = Math.max(view.firstInterpolant,
                Math.min(view.lastInterpolant, this.progress + additional_progress))
        return view.sample(progress)
    }

    fun trajectory(): Trajectory<S> {
        return view.trajectory
    }
}
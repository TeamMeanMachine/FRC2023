package org.team2471.frc2022

import org.jetbrains.kotlin.com.google.common.graph.Traverser
import org.team2471.frc.lib.coroutines.delay

data class Pose(val height: Double, val angle: Double) {

    companion object {
        val current: Pose
            get() = Pose(Climb.height, Climb.angle)

        val CLIMB_PREP = Pose(Climb.HEIGHT_VERTICAL_TOP, 0.0)
        val PULL_UP = Pose(Climb.HEIGHT_BOTTOM, Climb.ANGLE_BOTTOM)
        val PULL_UP_LATCH = Pose(Climb.HEIGHT_BOTTOM, 13.0)
        val PULL_UP_LATCH_LIFT = Pose(Climb.HEIGHT_BOTTOM_DETACH, 13.0)
        val PULL_UP_LATCH_RELEASE = Pose(Climb.HEIGHT_BOTTOM_DETACH, 20.0)
        val EXTEND_HOOKS = Pose(Climb.HEIGHT_TOP, Climb.ANGLE_TOP)
        val TRAVERSE_ENGAGE = Pose(Climb.HEIGHT_TOP, 15.0)
        val TRAVERSE_PULL_MID = Pose(Climb.HEIGHT_PARTIAL_PULL, 10.0)
        val TRAVERSE_PULL_UP = Pose(Climb.HEIGHT_PARTIAL_PULL - 2.0, -2.0)
    }
}

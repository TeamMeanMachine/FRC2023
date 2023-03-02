package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.inches

object NodeDeckHub {
    private val nodeDeckTable = NetworkTableInstance.getDefault().getTable("NodeDeck")
    private val chargeInAutoEntry = nodeDeckTable.getEntry("ChargeInAuto")
    private val isStartingInsideEntry = nodeDeckTable.getEntry("IsStartingInside")
    private val selectedNodeEntry = nodeDeckTable.getEntry("Selected Node")
    private val shoulderCoastModeEntry = nodeDeckTable.getEntry("setShoulderCoastMode")
    private val shoulderBrakeModeEntry = nodeDeckTable.getEntry("setShoulderBrakeMode")
    private val armPoseEntry = nodeDeckTable.getEntry("armPose")
    val shoulderBrakeMode: Boolean
        get() = shoulderBrakeModeEntry.getBoolean(false)
    val shoulderCoastMode: Boolean
        get() = shoulderCoastModeEntry.getBoolean(false)
    val chargeInAuto: Boolean
        get() = chargeInAutoEntry.getBoolean(false)
    val isStartingInside: Boolean
        get() = isStartingInsideEntry.getBoolean(false)
    val selectedNode
        get() = selectedNodeEntry.getInteger(0)

    init {
        GlobalScope.launch {
            armPoseEntry.setDoubleArray(doubleArrayOf(0.0, 0.0, 7.0.inches.asMeters, 0.0))
            periodic {
                if (shoulderCoastMode) {
                    Arm.shoulderCoastMode()
                    shoulderCoastModeEntry.setBoolean(false)
                    println("shoulderCoastMode: $shoulderCoastMode")
                }
                if (shoulderBrakeMode) {
                    Arm.shoulderBrakeMode()
                    shoulderBrakeModeEntry.setBoolean(false)
                    println("shoulderBrakeMode: $shoulderBrakeMode")
                }
            }
        }
    }
}
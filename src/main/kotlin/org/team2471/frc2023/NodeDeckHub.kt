package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.math.Vector2

object NodeDeckHub {
    private val nodeDeckTable = NetworkTableInstance.getDefault().getTable("NodeDeck")
    private val chargeInAutoEntry = nodeDeckTable.getEntry("ChargeInAuto")
    private val isStartingLeftEntry = nodeDeckTable.getEntry("IsStartingLeft")
    private val selectedNodeEntry = nodeDeckTable.getEntry("Selected Node")
    private val shoulderCoastModeEntry = nodeDeckTable.getEntry("setShoulderCoastMode")
    private val shoulderBrakeModeEntry = nodeDeckTable.getEntry("setShoulderBrakeMode")

    val shoulderBrakeMode: Boolean
        get() = shoulderBrakeModeEntry.getBoolean(false)
    val shoulderCoastMode: Boolean
        get() = shoulderCoastModeEntry.getBoolean(false)
    val chargeInAuto: Boolean
        get() = chargeInAutoEntry.getBoolean(false)
    val isStartingLeft: Boolean
        get() = isStartingLeftEntry.getBoolean(false)
    val selectedNode
        get() = selectedNodeEntry.getInteger(0)

    init {
        GlobalScope.launch {
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
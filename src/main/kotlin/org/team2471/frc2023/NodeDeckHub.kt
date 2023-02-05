package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableInstance

object NodeDeckHub {
    private val nodeDeckTable = NetworkTableInstance.getDefault().getTable("NodeDeck")
    private val chargeInAutoEntry = nodeDeckTable.getEntry("ChargeInAuto")
    private val isStartingLeftEntry = nodeDeckTable.getEntry("IsStartingLeft")
    private val selectedNodeEntry = nodeDeckTable.getEntry("Selected Node")
    val chargeInAuto: Boolean
        get() = chargeInAutoEntry.getBoolean(false)
    val isStartingLeft: Boolean
        get() = isStartingLeftEntry.getBoolean(false)
    val selectedNode
        get() = selectedNodeEntry.getInteger(0)
}
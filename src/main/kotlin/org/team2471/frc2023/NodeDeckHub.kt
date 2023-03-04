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
    private val isFloorConeEntry  = nodeDeckTable.getEntry("isFloorCone")
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
    val isFloorCone: Boolean
        get() = isFloorConeEntry.getBoolean(false)

    private var lastNode: Long = 0
    private var lastFloorState = false

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

                if (selectedNode != lastNode || lastFloorState != isFloorCone) {
                    // NodeDeck has updated! Call Things!
                    val color: GamePiece? = FieldManager.nodeList[selectedNode.toInt()]?.coneOrCube

                    when (color) {
                        GamePiece.CONE -> SignalLights.flashYellow()        // YELLOW
                        GamePiece.CUBE -> SignalLights.flashPurple()        // PURPLE
                        GamePiece.BOTH -> {     // OH NO ITS A FLOOR REQUEST
                            if (isFloorCone) SignalLights.flashYellow()     // YELLOW
                            else SignalLights.flashPurple()                 // PURPLE
                        }
                        else -> {}
                    }
                }
                lastNode = selectedNode;
                lastFloorState = isFloorCone;
            }
        }
    }
}
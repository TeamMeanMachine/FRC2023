package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.inches

object NodeDeckHub {
    private val nodeDeckTable = NetworkTableInstance.getDefault().getTable("NodeDeck")

    private val selectedNodeEntry = nodeDeckTable.getEntry("Selected Node")
    private val isFloorConeEntry  = nodeDeckTable.getEntry("isFloorCone")
    private val shoulderCoastModeEntry = nodeDeckTable.getEntry("setShoulderCoastMode")
    private val shoulderBrakeModeEntry = nodeDeckTable.getEntry("setShoulderBrakeMode")

    //auto entry's
    private val startingPointEntry = nodeDeckTable.getEntry("Starting Point")
    private val chargeInAutoEntry = nodeDeckTable.getEntry("ChargeInAuto")
    private val amountOfAutoPiecesEntry = nodeDeckTable.getEntry("AmountOfAutoPieces")
    private val autoOneEntry = nodeDeckTable.getEntry("1")
    private val autoTwoEntry = nodeDeckTable.getEntry("2")
    private val autoThreeEntry = nodeDeckTable.getEntry("3")
    private val autoFourEntry = nodeDeckTable.getEntry("4")
    private val autoFiveEntry = nodeDeckTable.getEntry("5")


    private val armPoseEntry = nodeDeckTable.getEntry("armPose") //why is this in the NodeDeck table?


    val selectedNode
        get() = selectedNodeEntry.getInteger(0)
    val shoulderBrakeMode: Boolean
        get() = shoulderBrakeModeEntry.getBoolean(false)
    val shoulderCoastMode: Boolean
        get() = shoulderCoastModeEntry.getBoolean(false)
    val isFloorCone: Boolean
        get() = isFloorConeEntry.getBoolean(false)

    //auto variables
    val chargeInAuto: Boolean
        get() = chargeInAutoEntry.getBoolean(false)
    val startingPoint: StartingPoint
        get() = StartingPoint.valueOf(startingPointEntry.getString("INSIDE"))
    val amountOfAutoPieces: Int
        get() = amountOfAutoPiecesEntry.getInteger(0).toInt()
    val firstAutoPiece: Int
        get() = autoOneEntry.getInteger(0).toInt()
    val secondAutoPiece: Int
        get() = autoTwoEntry.getInteger(0).toInt()
    val thirdAutoPiece: Int
        get() = autoThreeEntry.getInteger(0).toInt()
    val fourthAutoPiece: Int
        get() = autoFourEntry.getInteger(0).toInt()
    val fifthAutoPiece: Int //5 piece auto?
        get() = autoFiveEntry.getInteger(0).toInt()

    val isCone: Boolean
        get() = FieldManager.getSelectedNode()?.coneOrCube == GamePiece.CONE

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
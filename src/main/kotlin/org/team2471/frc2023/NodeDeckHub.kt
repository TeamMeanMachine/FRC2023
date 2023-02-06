package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableInstance
import org.team2471.frc.lib.math.Vector2

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
class ScoringNode (
    var coneOrCube: GamePiece,
    var level: Level,
    var pos: Vector2
)
enum class GamePiece {
    CUBE,
    CONE,
    BOTH
}
enum class Level {
    HIGH,
    MID,
    LOW
}
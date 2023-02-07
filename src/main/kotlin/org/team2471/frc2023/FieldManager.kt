package org.team2471.frc2023

import edu.wpi.first.math.geometry.Translation2d
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.meters

object FieldManager {
    val fieldDimensions = Vector2(26.9375.feet.asMeters,54.0.feet.asMeters)
    val fieldCenterOffset = fieldDimensions/2.0
    val nodeList: HashMap<Int, ScoringNode> = HashMap<Int, ScoringNode> ()


    init {
        for (n in 0 until 53) {
            //relying on Int uses Floor
            val column = n / 3
            val row = n.mod(3)
            val isCubeColumn = column.mod(3) == 1
            val isBoth = row == 2
            var scoringType = if (isBoth) GamePiece.BOTH else if (isCubeColumn) GamePiece.CUBE else GamePiece.CONE
            val level = Level.values()[row]
            val gamePiece = GamePiece.values()[0]
            var pos = Vector2(0.0, 0.0)

            if (n > 26) {
                //x cord of node 0 - the space between each node * column #, y cord of blue top node - space between each node * row #
                val newRow = (53 - n).mod(3)
                pos = Vector2((36.0 - 22.0 * column)/12.0, (308.5 - 17 * newRow + if (newRow == 2) 4.0 else 0.0)/12.0)
            } else {
                //x cord of node 0 - the space between each node * column #, y cord of red top node + space between each node * row #
                pos = Vector2((36.0 - 22.0 * column)/12.0, (-308.5 + 17 * row - if (row == 2) 4.0 else 0.0)/12.0)
            }

            nodeList.put(n, ScoringNode(scoringType, level, pos))
        }
    }

    fun convertWPIToTMM(wpiDimens: Translation2d): Vector2{
        val modX = wpiDimens.y + fieldCenterOffset.y
        val modY = -wpiDimens.x + fieldCenterOffset.x
        return Vector2(modX.meters.asFeet, modY.meters.asFeet)
    }
}

fun Translation2d.toTMMField():Vector2 {
    return FieldManager.convertWPIToTMM(this)
}
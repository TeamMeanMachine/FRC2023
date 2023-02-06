package org.team2471.frc2023

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.meters

object FieldManager {
    val fieldDimensions = Vector2(26.9375.feet.asMeters,54.0.feet.asMeters)
    val fieldCenterOffset = fieldDimensions/2.0
//    val nodeList: HashMap<Int, ScoringNode>


    init {
        // X layout
        val outerY = Units.inchesToMeters(54.25)
        val lowY = outerY - Units.inchesToMeters(14.25) / 2.0 // Centered when under cube nodes
        val midY = outerY - Units.inchesToMeters(22.75)
        val highY = outerY - Units.inchesToMeters(39.75)
        // Y layout
        val nodeFirstX = Units.inchesToMeters(20.19)
        val nodeSeparationX = Units.inchesToMeters(22.0)

        for (n in 0 until 53) {
            //relying on Int math uses the floor
            val column = n / 3
            val row = n.mod(3)
            val isCubeColumn = column.mod(3) == 1
            val isBoth = row == 2
            var scoringType = if (isBoth) GamePiece.BOTH else if (isCubeColumn) GamePiece.CUBE else GamePiece.CONE
            val level = Level.values()[row]
            val gamePiece = GamePiece.values()[0]

            val pos = Vector2((36.0 - 22.0 * column)/12.0, (-309.0 + row * 17.0 - if (row == 2) - 4.0 else 0.0)/12.0)




//            nodeList.put(n, ScoringNode(scoringType, level, pos)
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
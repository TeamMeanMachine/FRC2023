package org.team2471.frc2023

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.units.*
import java.lang.reflect.Field

object FieldManager {
    val fieldDimensions = Vector2(26.9375.feet.asMeters,54.0.feet.asMeters)
    val fieldCenterOffset = fieldDimensions/2.0
    val nodeList: HashMap<Int, ScoringNode> = HashMap()
    val scoringNodeYPosition = (fieldCenterOffset.y.meters.asInches - 55.0 - Drive.robotHalfWidth.asInches).inches.asFeet


    init {
        for (n in 0 until 54) {
            //relying on Int uses Floor
            val column = n / 3
            val row = n.mod(3)
            val isCubeColumn = column.mod(3) == 1
            val isBoth = row == 2
            val scoringType = if (isBoth) GamePiece.BOTH else if (isCubeColumn) GamePiece.CUBE else GamePiece.CONE
            val level = Level.values()[row]
            val pos = if (n > 26) {
                //x cord of node 0 - the space between each node * column #, y cord of blue top node - space between each node * row #
                val newRow = (53 - n).mod(3)
                Vector2((36.0 - 22.0 * column)/12.0, (308.5 - 17 * newRow + if (newRow == 2) 4.0 else 0.0)/12.0)
            } else {
                //x cord of node 0 - the space between each node * column #, y cord of red top node + space between each node * row #
                Vector2((36.0 - 22.0 * column)/12.0, (-308.5 + 17 * row - if (row == 2) 4.0 else 0.0)/12.0)
            }

            nodeList[n] = ScoringNode(scoringType, level, pos)
        }
    }
    fun convertTMMtoWPI(x:Length, y:Length, heading: Angle):Pose2d{
        val modX = -y.asMeters + fieldCenterOffset.y
        val modY = x.asMeters + fieldCenterOffset.x
        return Pose2d(modX,modY, Rotation2d((-heading+180.0.degrees).wrap().asRadians))
    }

    fun convertWPIToTMM(wpiDimens: Translation2d): Vector2{
        val modX = wpiDimens.y - fieldCenterOffset.x
        val modY = -(wpiDimens.x - fieldCenterOffset.y)
        return Vector2(modX.meters.asFeet, modY.meters.asFeet)
    }

    fun getSelectedNode() : ScoringNode? {
        return nodeList[NodeDeckHub.selectedNode.toInt()]
    }
}


fun Translation2d.toTMMField():Vector2 {
    return FieldManager.convertWPIToTMM(this)
}
fun Pose2d.toTMMField():Pose2d {
    val TMMVector = this.translation.toTMMField()
    val TMMHeading = (-this.rotation.degrees-180.0).degrees.wrap()
    return Pose2d(TMMVector.x,TMMVector.y,Rotation2d(TMMHeading.asRadians))
}

data class ScoringNode (
    var coneOrCube: GamePiece,
    var level: Level,
    var position: Vector2
)
val ScoringNode.alliance
    get() = if (this.position.y < 0.0) DriverStation.Alliance.Red else DriverStation.Alliance.Blue

val ScoringNode.alignPosition : Vector2
    get() {
        val fieldSideMultiplier = if (this.alliance == DriverStation.Alliance.Red) -1.0 else 1.0
        return Vector2(this.position.x, FieldManager.scoringNodeYPosition * fieldSideMultiplier)
    }
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


package org.team2471.frc2023

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.units.*

object FieldManager {

    val fieldDimensions = Vector2(26.29.feet.asMeters,54.27.feet.asMeters) // field diagram & json is 26.29, 54.27 but includes side walls and barriers
    val fieldCenterOffset = fieldDimensions/2.0
    val gridYOffset = 55.0.inches
    val chargingStationYOffset = gridYOffset + 60.0.inches
    val chargingStationXOffset = 0.0.feet
    val chargingStationWidth = 8.0.feet
    val chargingStationDepth = 76.125.inches
    val nodeList: HashMap<Int, ScoringNode> = HashMap()
    val gamePieceStartingPos = ArrayList<Vector2>(8)
    var allianceSidePieces : MutableList<Vector2>? = null
    val scoringNodeYPosition = (fieldCenterOffset.y.meters - gridYOffset - Drive.robotHalfWidth).asFeet
    val avoidanceZones = ArrayList<AvoidanceZone>()
    val gamePieceOnFieldFromCenterY = 47.36.inches
    val gamePieceOnFieldFromCenterX = 22.39.inches
    val gamePieceOnFieldOffsetX = 48.0.inches // count from positive X and offset in negative direction
    val singleSubstationOffsetX = 157.75.inches
    val singleSubstationOffsetY = 238.inches
    val chargeFromCenterY = 85.inches + gamePieceOnFieldFromCenterY
    val gridFromCenterY = 224.0.inches + gamePieceOnFieldFromCenterY
    val chargeFromWall = 59.39.inches

    val barrierTip = Vector2(58.0.inches.asFeet, 16.0)

    val insideSafePointClose: Vector2
        get() = reflectFieldByAlliance( Vector2(barrierTip.x/2.0, (gridFromCenterY - 66.0.inches + Drive.robotHalfWidth).asFeet))
    val insideSafePointFar: Vector2
        get() = reflectFieldByAlliance( Vector2(insideSafePointClose.x, (chargeFromCenterY - 30.0.inches).asFeet))
    val insideSafePointCharge: Vector2
        get() = reflectFieldByAlliance( Vector2(centerOfChargeX, insideSafePointClose.y))
    val outsideSafePointClose: Vector2
        get() = reflectFieldByAlliance( Vector2(-fieldCenterOffset.x.meters.asFeet + (chargeFromWall/2.0).asFeet, insideSafePointClose.y))
    val outsideSafePointFar: Vector2
        get() = reflectFieldByAlliance( Vector2(outsideSafePointClose.x, insideSafePointFar.y))
    val outsideSafePointCharge: Vector2
        get() = reflectFieldByAlliance( Vector2(centerOfChargeX, insideSafePointFar.y - 1.0))
    val centerOfChargeX: Double
        get() = (chargingStationXOffset - chargingStationWidth/2.0).asFeet

    val startingPosition: Vector2
        get() = Vector2 ( if (NodeDeckHub.startingPoint == StartingPoint.INSIDE) insideStartingPosition.x else outsideStartingPosition.x, if (isRedAlliance) -insideStartingPosition.y else insideStartingPosition.y)
    val insideStartingPosition = Vector2(3.0, scoringNodeYPosition)
    val outsideStartingPosition = Vector2(-11.5, scoringNodeYPosition)

    val closeDoubleSubstationOffsetX = 67.64.inches
    val farDoubleSubstationOffsetX = 149.inches

    val doubleSubstationOffsetY = 311.35.inches
    val doubleSubstationHeight = 37.375.inches

    val isRedAlliance: Boolean
        get() = DriverStation.getAlliance() == Alliance.Red
    val isBlueAlliance: Boolean
        get() = !isRedAlliance

    val singleSubstationPosition : Vector2
        get() = Vector2(singleSubstationOffsetX.asFeet, (if (isRedAlliance) singleSubstationOffsetY else -singleSubstationOffsetY).asFeet)
    val closeDoubleSubstationPosition: Vector2
        get() = Vector2(closeDoubleSubstationOffsetX.asFeet, (if (isRedAlliance) doubleSubstationOffsetY else  -doubleSubstationOffsetY).asFeet)
    val farDoubleSubstationPosition: Vector2
        get() = Vector2(farDoubleSubstationOffsetX.asFeet, (if (isRedAlliance) doubleSubstationOffsetY else  -doubleSubstationOffsetY).asFeet)

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
                //x cord of node 54 + the space between each node * column #, y cord of blue top node - space between each node * row #
                val newRow = (53 - n).mod(3)
                Vector2((-140.0 + 22.0 * (column - 9))/12.0, (308.5 - 17 * newRow + if (newRow == 2) 4.0 else 0.0)/12.0)
            } else {
                //x cord of node 0 - the space between each node * column #, y cord of red top node + space between each node * row #
                Vector2((36.0 - 22.0 * column)/12.0, (-308.5 + 17 * row - if (row == 2) 4.0 else 0.0)/12.0)
            }
            nodeList[n] = ScoringNode(scoringType, level, pos)
//            nodeList[n]?.position?.let { println(it.x) }
        }
//        avoidanceZones.add(AvoidanceZone("RedChargeStation", Vector2(-10.5, -6.0), Vector2(0.0, -12.0)))
        for (p in 0 until 8) {
            gamePieceStartingPos.add(Vector2((gamePieceOnFieldFromCenterX - gamePieceOnFieldOffsetX * p.toDouble().mod(4.0)).asFeet, if (p > 3) -gamePieceOnFieldFromCenterY.asFeet else gamePieceOnFieldFromCenterY.asFeet))
            println(gamePieceStartingPos[p])
        }
    }
    fun reflectFieldByAlliance(y: Double): Double {
        return if (isRedAlliance) -y else y
    }
    fun reflectFieldByAlliance(p: Vector2): Vector2 {
        return Vector2(p.x,  if (isRedAlliance) -p.y else p.y)
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

    fun getNode(nodeID: Int) : ScoringNode? {
        return nodeList[nodeID]
    }

    fun getSelectedNode() : ScoringNode? {
        return nodeList[NodeDeckHub.selectedNode.toInt()]
    }
    fun getClosestGamePieceOnField(): Vector2 {
        val currPose = PoseEstimator.currentPose
        if (allianceSidePieces.isNullOrEmpty()) {
            allianceSidePieces = gamePieceStartingPos.filter { (isBlueAlliance && it.y > 0.0 || isRedAlliance && it.y < 0.0) }.sortedBy { it.distance(currPose) }.toMutableList()
        }
        return allianceSidePieces!!.removeFirst()
    }
    fun resetClosestGamePieceOnField() {
        allianceSidePieces = null
    }
}

fun Vector2.toWPIField():Translation2d {
    return FieldManager.convertTMMtoWPI(this.x.feet, this.y.feet, 0.0.degrees).translation
}
fun Translation2d.toTMMField():Vector2 {
    return FieldManager.convertWPIToTMM(this)
}

val Translation2d.toVector2:Vector2
    get() = Vector2(this.x, this.y)

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
    get() = if (this.position.y < 0.0) Alliance.Red else Alliance.Blue

val ScoringNode.alignPosition : Vector2
    get() = Vector2(this.position.x, FieldManager.reflectFieldByAlliance(FieldManager.scoringNodeYPosition - if (this.level == Level.HIGH) 1.0 else 0.0))
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
enum class SafeSide {
    INSIDE,
    OUTSIDE,
    CHARGE,
    DYNAMIC
}
enum class StartingPoint {
    INSIDE,
    OUTSIDE,
    MIDDLE
}
data class AvoidanceZone (
    val name : String,
    val topLeft : Vector2,
    val bottomRight : Vector2
    )


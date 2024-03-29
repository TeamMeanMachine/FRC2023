package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.parallel
//import org.team2471.bunnybots2022.Drive
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion.following.drive
import org.team2471.frc.lib.motion.following.driveAlongPath
import org.team2471.frc.lib.motion_profiling.Autonomi
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.measureTimeFPGA
import java.io.File
import java.util.*

private lateinit var autonomi: Autonomi


enum class Side {
    LEFT,
    RIGHT;

    operator fun not(): Side = when (this) {
        LEFT -> RIGHT
        RIGHT -> LEFT
    }
}

private var startingSide = Side.RIGHT
val selAuto
    get() = SmartDashboard.getString("Autos/selected", "no auto selected")

object AutoChooser {
    private val isRedAllianceEntry = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("isRedAlliance")
    private var autonomiEntryTopicSub =
        NetworkTableInstance.getDefault().getTable("PathVisualizer").getStringTopic("Autonomi").subscribe("")

    var cacheFile: File? = null
    var redSide: Boolean = true
        get() = isRedAllianceEntry.getBoolean(true)
        set(value) {
            field = value
            isRedAllianceEntry.setBoolean(value)
        }

    private val lyricsChooser = SendableChooser<String?>().apply {
        setDefaultOption("Country roads", "Country roads")
        addOption("take me home", "take me home")
    }

    private val testAutoChooser = SendableChooser<String?>().apply {
        addOption("None", null)
        addOption("20 Foot Test", "20 Foot Test")
        addOption("8 Foot Straight", "8 Foot Straight")
        addOption("2 Foot Circle", "2 Foot Circle")
        addOption("4 Foot Circle", "4 Foot Circle")
        addOption("8 Foot Circle", "8 Foot Circle")
        addOption("Hook Path", "Hook Path")
        setDefaultOption("90 Degree Turn", "90 Degree Turn")


    }

    private val autonomousChooser = SendableChooser<String?>().apply {
        setDefaultOption("Tests", "testAuto")
        addOption("Outer Three Auto", "outerThreeAuto")
        addOption("Outer Two Auto", "outerTwoAuto")
        addOption("Inner Three Auto", "innerThreeAuto")
        addOption("NodeDeck", "nodeDeck")

    }

    var numberOfObjects = 3
    var chargeStation = false


    init {
//        DriverStation.reportWarning("Starting auto init warning", false)
//        DriverStation.reportError("Starting auto init error", false)         //            trying to get individual message in event log to get timestamp -- untested

        SmartDashboard.putData("Best Song Lyrics", lyricsChooser)
        SmartDashboard.putData("Tests", testAutoChooser)
        SmartDashboard.putData("Autos", autonomousChooser)

        try {
            cacheFile = File("/home/lvuser/autonomi.json")
            if (cacheFile != null) {
                autonomi = Autonomi.fromJsonString(cacheFile?.readText())!!
                println("Autonomi cache loaded.")
            } else {
                println("Autonomi failed to load!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! RESTART ROBOT!!!!!!")
            }
        } catch (_: Throwable) {
            DriverStation.reportError("Autonomi cache could not be found", false)
            autonomi = Autonomi()
        }
        println("In Auto Init. Before AddListener. Hi.")
        NetworkTableInstance.getDefault().addListener(
            autonomiEntryTopicSub,
            EnumSet.of(
                NetworkTableEvent.Kind.kImmediate,
                NetworkTableEvent.Kind.kPublish,
                NetworkTableEvent.Kind.kValueAll
            )
        ) { event ->
            println("Automous change detected")
            val json = event.valueData.value.string
            if (json.isNotEmpty()) {
                val t = measureTimeFPGA {
                    autonomi = Autonomi.fromJsonString(json) ?: Autonomi()
                }
                println("Loaded autonomi in $t seconds")
                if (cacheFile != null) {
                    println("CacheFile != null. Hi.")
                    cacheFile!!.writeText(json)
                } else {
                    println("cacheFile == null. Hi.")
                }
                println("New autonomi written to cache")
            } else {
                autonomi = Autonomi()
                DriverStation.reportWarning("Empty autonomi received from network tables", false)
            }
        }
    }

    suspend fun autonomous() = use(Drive, name = "Autonomous") {
        println("Got into Auto fun autonomous. Hi. 888888888888888 ${Robot.recentTimeTaken()}")
        SmartDashboard.putString("autoStatus", "init")
        println("Selected Auto = *****************   $selAuto ****************************  ${Robot.recentTimeTaken()}")
        when (selAuto) {
            "Tests" -> testAuto()
            "Outer Three Auto" -> outerThreeAuto()
            "Outer Two Auto" -> outerTwoAuto()
            "Inner Three Auto" -> innerThreeAuto()
            "NodeDeck" -> nodeDeckAuto()
            else -> println("No function found for ---->$selAuto<-----  ${Robot.recentTimeTaken()}")
        }
        SmartDashboard.putString("autoStatus", "complete")
        println("finished autonomous  ${Robot.recentTimeTaken()}")
    }

    suspend fun outerThreeAuto() = use(Intake, Arm, Drive) {
        val auto = if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) autonomi["Outer Red Auto"] else autonomi["Outer Blue Auto"]
        println("$numberOfObjects number of objects, and charging? $chargeStation")
        if (auto != null) {
            if (numberOfObjects > 1) {
                Drive.driveAlongPath(auto["01 GetCube1"], true)
                Drive.driveAlongPath(auto["02 DropCube1"])
                if (numberOfObjects > 2) {
                    Drive.driveAlongPath(auto["03 GetCube2"])
                    Drive.driveAlongPath(auto["04 DropCube2"])
                    if (chargeStation) Drive.driveAlongPath(auto["05 ToCharge"])
                } else if (chargeStation) Drive.driveAlongPath(auto["06 ToCharge2Piece"])
            } else if (chargeStation) Drive.driveAlongPath(auto["07 ToCharge1Piece"])
        }
    }
    suspend fun outerTwoAuto() = use(Intake, Arm, Drive) {
        val auto = if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) autonomi["Outer Red Auto"] else autonomi["Outer Blue Auto"]
        if (auto != null) {
            Drive.driveAlongPath(auto["01 GetCube1"], true)
            Drive.driveAlongPath(auto["02 DropCube1"])
            Drive.driveAlongPath(auto["06 ToCharge2Piece"])
        }
    }

    suspend fun innerThreeAuto() = use(Drive, Intake, Arm) {
        val auto = if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) autonomi["Inner Red Auto"] else autonomi["Inner Blue Auto"]
        if (auto != null) {
            Drive.driveAlongPath(auto["01 GetCube1"], true)
            Drive.driveAlongPath(auto["02 DropCube1"])
            Drive.driveAlongPath(auto["03 GetCube2"])
            Drive.driveAlongPath(auto["04 DropCube2"])
            Drive.driveAlongPath(auto["05 ToCharge"])
        }
    }
    suspend fun nodeDeckAuto() = use(Drive, Intake, Arm) {
        Intake.coneToward = true
//        AprilTag.resetCameras()
        if (FieldManager.nodeList[NodeDeckHub.firstAutoPiece]?.level?.equals(Level.LOW) == true) Intake.intakeMotor.setPercentOutput(Intake.CUBE_SPIT) else if (FieldManager.nodeList[NodeDeckHub.firstAutoPiece]?.coneOrCube == GamePiece.CONE) Intake.intakeMotor.setPercentOutput(Intake.HOLD_CONE) else Intake.intakeMotor.setPercentOutput(Intake.HOLD_CUBE)
        var gamePieceAngles = when (NodeDeckHub.startingPoint) {
            StartingPoint.INSIDE -> doubleArrayOf(0.0, -30.0, -45.0)
            StartingPoint.MIDDLE -> doubleArrayOf(0.0, 30.0, -30.0)
            StartingPoint.OUTSIDE -> doubleArrayOf(0.0, 30.0, 45.0)
        }
        FieldManager.resetClosestGamePieceOnField()
        if ((Drive.position - FieldManager.startingPosition).length > 0.2) {
            println("forcing position .. we are off by more than 0.2 feet")
            Drive.position = FieldManager.startingPosition
            println("position: ${Drive.position}")
            PoseEstimator.zeroOffset()
        }
        Drive.zeroGyro()
        println("position: ${Drive.position}, ${Drive.combinedPosition}")
        if (NodeDeckHub.amountOfAutoPieces > 0) {
            if (FieldManager.nodeList[NodeDeckHub.firstAutoPiece]?.level?.equals(Level.LOW) == true) { //spit and run--special case
                Intake.intakeMotor.setPercentOutput(Intake.CUBE_SPIT)
                Drive.drive(Vector2(0.0,0.0), 0.0)
                delay(0.8)
                Intake.intakeMotor.setPercentOutput(0.0)
                if (!NodeDeckHub.chargeInAuto) Drive.dynamicGoToGamePieceOnFloor(FieldManager.getClosestGamePieceOnField(), gamePieceAngles[0].degrees, isCone = false)
            } else {
                backScoreToward(FieldManager.nodeList[NodeDeckHub.firstAutoPiece]?.coneOrCube == GamePiece.CONE, NodeDeckHub.firstAutoPiece)
                if (NodeDeckHub.amountOfAutoPieces == 1) scoreObject(NodeDeckHub.firstAutoPiece)
                if (NodeDeckHub.amountOfAutoPieces > 1) {
                    nodeDeckPiece(gamePieceAngles[0].degrees, NodeDeckHub.secondAutoPiece, NodeDeckHub.amountOfAutoPieces == 2 && NodeDeckHub.finishWithPiece, NodeDeckHub.firstAutoPiece)
                    if (NodeDeckHub.amountOfAutoPieces == 2) scoreObject(NodeDeckHub.secondAutoPiece)
                    if (NodeDeckHub.amountOfAutoPieces > 2) {
                        nodeDeckPiece(gamePieceAngles[1].degrees, NodeDeckHub.thirdAutoPiece,  NodeDeckHub.amountOfAutoPieces == 3 && NodeDeckHub.finishWithPiece, NodeDeckHub.secondAutoPiece)
                        if (NodeDeckHub.amountOfAutoPieces == 3) scoreObject(NodeDeckHub.thirdAutoPiece)
                        if (NodeDeckHub.amountOfAutoPieces > 3) {
                            nodeDeckPiece(gamePieceAngles[2].degrees, NodeDeckHub.fourthAutoPiece,  NodeDeckHub.amountOfAutoPieces == 4 && NodeDeckHub.finishWithPiece, NodeDeckHub.thirdAutoPiece)
                        }
                    }
                }
            }
        }
        println("charge: ${NodeDeckHub.chargeInAuto}")
        if (NodeDeckHub.chargeInAuto) {
            parallel({
                toBackDrivePose()
            },{
                val chargeX = when(NodeDeckHub.startingPoint) {
                    StartingPoint.OUTSIDE -> -1
                    StartingPoint.INSIDE -> 1
                    StartingPoint.MIDDLE -> 0
                }
                val chargeDestination = Vector2(FieldManager.centerOfChargeX + (Drive.robotHalfWidth.asFeet * 1.7 * chargeX), FieldManager.reflectFieldByAlliance(15.25))
                Drive.driveToPointsPercentSpeed(0.5, Drive.combinedPosition, Vector2(chargeDestination.x, FieldManager.reflectFieldByAlliance(7.0 + Drive.robotHalfWidth.asFeet)))
                delay(0.05)
                Drive.driveToPointsPercentSpeed(0.5, Drive.combinedPosition, chargeDestination)
                Drive.autoBalance()
            })
        }
    }
    suspend fun nodeDeckPiece(pickupHeading: Angle, nodeID: Int, finishWithPiece: Boolean, prevPiece: Int) = use(Intake, Drive, Arm) {
        val nextGamePiece = FieldManager.getClosestGamePieceOnField()
        println("nodedeck auto path to game piece: $nextGamePiece")
        val isCone = FieldManager.nodeList[nodeID]?.coneOrCube == GamePiece.CONE
        parallel({
            delay(0.9)
            Drive.dynamicGoToGamePieceOnFloor(nextGamePiece, pickupHeading, isCone = isCone)
        }, {
            scoreObject(prevPiece)
            println("before toFrontDrivePose")
            delay(0.6) // To front
            println("before intakeFromGround")
            intakeFromGroundAuto(isCone)
        })
        println("finished goToGamePiece")
        val scoringNode = FieldManager.getNode(nodeID)
        println("finishWithPiece: $finishWithPiece")
        if (!finishWithPiece) {
            if (scoringNode == null) {
                println("Scoring Node is Null")
            } else {
                println("Nodedeck auto path to node: ${scoringNode.alignPosition}")
                val safeSide = when (NodeDeckHub.startingPoint) {
                    StartingPoint.INSIDE -> SafeSide.INSIDE
                    StartingPoint.MIDDLE -> SafeSide.CHARGE
                    StartingPoint.OUTSIDE -> SafeSide.OUTSIDE
                }
                parallel({
                    groundBackToDrive(FieldManager.getNodeIsCone(nodeID))
                    backScoreToward(FieldManager.nodeList[nodeID]?.coneOrCube == GamePiece.CONE, nodeID)
                },{
                    Drive.dynamicGoToScore(scoringNode.alignPosition, safeSide)
                })
            }
        } else {

//            Drive.dynamicGoToChargeCenter()
//            println("Right before charge. pos: ${Drive.combinedPosition}   8888888888888")
//            val destination = Vector2(FieldManager.centerOfChargeX + if (Drive.combinedPosition.x < FieldManager.centerOfChargeX) - Drive.robotHalfWidth.asFeet else + Drive.robotHalfWidth.asFeet * 2.0,
//                FieldManager.reflectFieldByAlliance((FieldManager.chargeFromCenterY + FieldManager.chargingStationDepth / 2.00).asFeet))
//            Drive.driveToPoints(Drive.combinedPosition, Vector2(destination.x, Drive.combinedPosition.y), destination)
//            Drive.rampTest()
//            Drive.autoBalance()
        }
    }


    private suspend fun testAuto() {
        val testPath = SmartDashboard.getString("Tests/selected", "no test selected") // testAutoChooser.selected
        if (testPath != null) {
            val testAutonomous = autonomi["Tests"]
            val path = testAutonomous?.get(testPath)
            if (path != null) {
                println("Path isnt null")
                Drive.driveAlongPath(path, true)
            } else {
                println("Path is null")
            }
        }
    }

    private suspend fun spitAndRunAuto() {
        Intake.intakeMotor.setPercentOutput(Intake.CUBE_SPIT)
        Drive.driveToPoints()
//        Drive.dynamicGoToGamePieceOnFloor(FieldManager.getClosestGamePieceOnField(), if (FieldManager.isRedAlliance) 0.0.degrees else 180.0.degrees)
    }

//    suspend fun highConeAndCubesAuto() = use(Drive, Intake, Arm){
//        Intake.coneToward = true
//        AprilTag.resetCameras()
//        if (FieldManager.nodeList[NodeDeckHub.firstAutoPiece]?.level?.equals(Level.LOW) == true) Intake.intakeMotor.setPercentOutput(Intake.CUBE_SPIT) else Intake.intakeMotor.setPercentOutput(Intake.HOLD_CONE)
//        var gamePieceAngles = when (NodeDeckHub.startingPoint) {
//            StartingPoint.INSIDE -> doubleArrayOf(0.0, -30.0, -45.0)
//            StartingPoint.MIDDLE -> doubleArrayOf(0.0, 30.0, -30.0)
//            StartingPoint.OUTSIDE -> doubleArrayOf(0.0, 30.0, 45.0)
//        }
//        FieldManager.resetClosestGamePieceOnField()
//        if ((Drive.position - FieldManager.startingPosition).length > 0.2) {
//            println("forcing position .. we are off by more than 0.2 feet")
//            Drive.position = FieldManager.startingPosition
//            println("position: ${Drive.position}")
//            PoseEstimator.zeroOffset()
//        }
//        Drive.zeroGyro()
//        println("position: ${Drive.position}, ${Drive.combinedPosition}")
//        if (NodeDeckHub.amountOfAutoPieces > 0) {
//            if (FieldManager.nodeList[NodeDeckHub.firstAutoPiece]?.level?.equals(Level.LOW) == true) { //spit and run--special case
//                Intake.intakeMotor.setPercentOutput(Intake.CUBE_SPIT)
//                delay(0.8)
//                parallel ({
//                    if (!NodeDeckHub.chargeInAuto) Drive.dynamicGoToGamePieceOnFloor(FieldManager.getClosestGamePieceOnField(), gamePieceAngles[0].degrees, isCone = false)
//                }, {
//                    Intake.intakeMotor.setPercentOutput(0.0)
//                    flip()
//                })
//            } else {
//                backScoreToward(true, NodeDeckHub.firstAutoPiece)
//                if (NodeDeckHub.amountOfAutoPieces == 1) scoreObject(NodeDeckHub.firstAutoPiece)
//                if (NodeDeckHub.amountOfAutoPieces > 1) {
//                    nodeDeckPiece(gamePieceAngles[0].degrees, NodeDeckHub.secondAutoPiece, NodeDeckHub.amountOfAutoPieces == 2 && NodeDeckHub.finishWithPiece, NodeDeckHub.firstAutoPiece)
//                    if (NodeDeckHub.amountOfAutoPieces == 2) scoreObject(NodeDeckHub.secondAutoPiece)
//                    if (NodeDeckHub.amountOfAutoPieces > 2) {
//                        nodeDeckPiece(gamePieceAngles[1].degrees, NodeDeckHub.thirdAutoPiece,  NodeDeckHub.amountOfAutoPieces == 3 && NodeDeckHub.finishWithPiece, NodeDeckHub.secondAutoPiece)
//                        if (NodeDeckHub.amountOfAutoPieces == 3) scoreObject(NodeDeckHub.thirdAutoPiece)
//                        if (NodeDeckHub.amountOfAutoPieces > 3) {
//                            nodeDeckPiece(gamePieceAngles[2].degrees, NodeDeckHub.fourthAutoPiece,  NodeDeckHub.amountOfAutoPieces == 4 && NodeDeckHub.finishWithPiece, NodeDeckHub.thirdAutoPiece)
//                        } else {
//                            afterScoreFlip(FieldManager.nodeList[NodeDeckHub.thirdAutoPiece]?.level)
//                        }
//                    } else {
//                        afterScoreFlip(FieldManager.nodeList[NodeDeckHub.secondAutoPiece]?.level)
//                    }
//                } else {
//                    afterScoreFlip(FieldManager.nodeList[NodeDeckHub.firstAutoPiece]?.level)
//                }
//            }
//        } else {
////            flip()
//        }
//        println("charge: ${NodeDeckHub.chargeInAuto}")
//        if (NodeDeckHub.chargeInAuto) {
//            if (NodeDeckHub.amountOfAutoPieces == 0) {
//                Drive.driveToPoints(Drive.combinedPosition, Drive.combinedPosition + Vector2(0.0, FieldManager.reflectFieldByAlliance(-0.25)))
//                delay(2.0)
//            }
//            parallel({
////                Drive.dynamicGoToChargeCenter()
//                val chargeX = when(NodeDeckHub.startingPoint) {
//                    StartingPoint.OUTSIDE -> -1
//                    StartingPoint.INSIDE -> 1
//                    StartingPoint.MIDDLE -> 0
//                }
//                val chargeDestination = Vector2(FieldManager.centerOfChargeX + (Drive.robotHalfWidth.asFeet * 1.7 * chargeX), FieldManager.reflectFieldByAlliance(15.25))
//                Drive.driveToPointsPercentSpeed(0.5, Drive.combinedPosition, Vector2(chargeDestination.x, FieldManager.reflectFieldByAlliance(8.0 + Drive.robotHalfWidth.asFeet)))
//                Drive.driveToPointsPercentSpeed(0.5, Drive.combinedPosition, chargeDestination)
//                Drive.autoBalance()
//            }, {
////                toDrivePose()
//            })
//        } else {
////            toDrivePose()
//        }
//    }
}

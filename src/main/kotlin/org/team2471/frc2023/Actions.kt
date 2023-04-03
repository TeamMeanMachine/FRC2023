package org.team2471.frc2023

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.DriverStation
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.parallel
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.util.Timer
import kotlin.math.absoluteValue

//suspend fun scoreIfReady() {
//    var startGoScore = false
//    periodic {
//
////        println("Checking if driving... ${OI.driveRotation}  ${OI.driveTranslation.length}")
//
//        if (!Drive.isHumanDriving) {
//            startGoScore = true
//            stop()
//        }
//    }
//    if (startGoScore) {
//        goScore()
//    }
//}
//
//
//suspend fun goScore() = use(Drive, Arm, Intake, name = "goScore") {
//    periodic {
//        if (!OI.operatorController.x || Drive.isHumanDriving) {
//            stop()
//        }
////        val scoringPos = FieldConstants.Grids.high3dTranslations[NodeDeckHub.selectedNode.toInt()]
//        println()
//    }
//}
//suspend fun scoreIfReady(){
//    var startGoScore = false
//    periodic {
//
//        println("Checking if driving... ${OI.driveRotation}  ${OI.driveTranslation.length}")
//
//        if (!Drive.isHumanDriving) {
//            startGoScore = true
//            stop()
//        }
//    }
//    if (startGoScore) {
//        goScore()
//    }
//}
//
//
//suspend fun goScore() = use(Drive, Arm, Intake, name = "goScore") {
//    periodic {
//        if (!OI.operatorController.x || Drive.isHumanDriving) {
//            stop()
//        }
////        val scoringPos = FieldConstants.Grids.high3dTranslations[NodeDeckHub.selectedNode.toInt()]
//        println()
//    }
//}

suspend fun intakeCurrentLogic() {
    val t = Timer()
    var isTimerStarted = false
    var intakeDetectTime = 0.0
    var linearFilter = LinearFilter.movingAverage(5)
    periodic {
        //-1.0
        linearFilter.calculate(Intake.intakeMotor.current) //intake bad
        if (!isTimerStarted) {
            t.start()
            isTimerStarted = true
            intakeDetectTime = 10000.0
            Intake.intakeMotor.setPercentOutput(Intake.INTAKE_CONE) //intake bad
            println("timer is started")
        } else if (t.get() > 2.0) {
            if (linearFilter.calculate(Intake.intakeMotor.current) > Intake.DETECT_CONE && intakeDetectTime == 10000.0) { //intake bad
                intakeDetectTime = t.get() + 0.5
                println("detected = ${intakeDetectTime}")
            }
            if (t.get() > intakeDetectTime) {
                Intake.holdingObject = true
                Intake.holdDetectedTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp()
                println("t_get = ${t.get()}")
            }
            if (t.get() > intakeDetectTime + 2.0) Intake.intakeMotor.setPercentOutput(Intake.HOLD_CONE) //intake bad
            if (OI.operatorRightTrigger < 0.05) {
                Intake.intakeMotor.setPercentOutput(if (Intake.holdingObject) Intake.HOLD_CONE else 0.0) //intake bad
                this.stop()
            }
        }
    }
}
suspend fun intakeFromGroundAuto(isCone: Boolean) = use(Arm, Intake) {
    Intake.intakeMotor.setPercentOutput(if (isCone) Intake.INTAKE_CONE else Intake.INTAKE_CUBE)
    if  (isCone) {
        animateThroughPoses(Pose.GROUND_INTAKE_FRONT_CONE, Pose.GROUND_INTAKE_CONE_NEAR, Pose.GROUND_INTAKE_CONE_FAR)
    } else {
        animateThroughPoses(Pose.GROUND_INTAKE_FRONT_CUBE, Pose.GROUND_INTAKE_CUBE_NEAR, Pose.GROUND_INTAKE_CUBE_FAR)
    }
    val timer2 = Timer()
    timer2.start()
    var holdingTime = 25.0
    periodic {
        if (timer2.get() > 2.0 || timer2.get() - holdingTime > 0.4) {
            println("totalTime: ${timer2.get() > 1.0}  holdTime ${timer2.get() - holdingTime > 0.4}")
            this.stop()
        }
        if (Intake.holdingObject && holdingTime == 25.0 && timer2.get() > 0.15) {
            holdingTime = timer2.get()
        }
        if (!Intake.holdingObject) holdingTime = 25.0
    }
    Intake.intakeMotor.setPercentOutput(if (Intake.holdingObject) (if (isCone) Intake.HOLD_CONE else Intake.HOLD_CUBE) else 0.0)
}

suspend fun intakeFromGround(isCone: Boolean = NodeDeckHub.isCone) = use(Arm, Intake) {
    println("inside intakeFromGround")
    if (Intake.wristAngle.asDegrees > 70.0) {
        try {
            println("in intakeFromGround")
            val path = Path2D("newPath")
            if (isCone) {
                path.addVector2(Pose.GROUND_INTAKE_CONE_NEAR.wristPosition)
                path.addVector2(Pose.GROUND_INTAKE_CONE_FAR.wristPosition)
            } else {
                path.addVector2(Pose.GROUND_INTAKE_CUBE_NEAR.wristPosition)
                path.addVector2(Pose.GROUND_INTAKE_CUBE_FAR.wristPosition)
            }
            val distance = path.length
            val rate = 150.0  //  inches per second
            val time = distance / rate
            path.addEasePoint(0.0, 0.0)
            path.addEasePoint(time, 1.0)

            Drive.maxTranslation = 0.5
            //go to close intake
            if  (isCone) {
                animateThroughPoses(true, Pose.GROUND_INTAKE_FRONT_CONE, Pose.GROUND_INTAKE_CONE_NEAR)
            } else {
                animateThroughPoses(true, Pose.GROUND_INTAKE_FRONT_CUBE, Pose.GROUND_INTAKE_CUBE_NEAR)
            }
            println("after, wristPos: ${Arm.shoulderSetpoint} ${Arm.elbowSetpoint}    actual: ${Arm.shoulderAngle} ${Arm.elbowAngle}")
//            if (OI.operatorLeftTrigger < 0.05 && DriverStation.isTeleop()) {  // todo: should be able to early exit the animations above
//                this.stop()
//            }
            val slewRateLimiter = SlewRateLimiter(10.0, -10.0, 0.0)

            val timer = Timer()
            timer.start()

            Intake.intakeMotor.setPercentOutput(if (isCone) Intake.INTAKE_CONE else Intake.INTAKE_CUBE) //intake bad
            if (OI.operatorLeftTrigger > 0.05) { //animate through close and far pos
                var tInitialHold = -1.0
                var tHold = -1.0
                var rumbleTimer = -1.0
                parallel({
                    periodic {
                        tHold = if (tInitialHold != -1.0) timer.get() - tInitialHold else -1.0
                        val alpha2 =
                            if (DriverStation.isTeleop()) slewRateLimiter.calculate(linearMap(0.5, 1.0, 0.0, 1.0, OI.operatorLeftTrigger).coerceIn(0.0, 1.0)) else slewRateLimiter.calculate(1.0)
                        val tPath = linearMap(0.0, 1.0, 0.0, time, alpha2)
                        Arm.wristPosition =
                            path.getPosition(tPath) + if (tHold > 0.25) Vector2(-4.0, 4.0) else Vector2(0.0, 0.0) //test intake motor doesn't turn on as much, controlling intakeFromGround curved, rumble upon intake
                        Intake.intakeMotor.setPercentOutput(if (isCone) (if (tHold > 3.0) Intake.HOLD_CONE else Intake.INTAKE_CONE) else (if (tHold > 2.0) Intake.HOLD_CUBE else Intake.INTAKE_CUBE))
                        if (DriverStation.isTeleop() && OI.operatorLeftTrigger < 0.01) {
                            this.stop()
                        }
                    }

                }, {
                    periodic {//rumble if holding object
                        if (Intake.holdingObject && tInitialHold == -1.0) {
                            tInitialHold = timer.get()
                            rumbleTimer = timer.get() + 1.0
                            if (!DriverStation.isAutonomous()) {
                                OI.driverController.rumble = 0.3
                                OI.operatorController.rumble = 0.3
                            }
                        }
                        if (!Intake.holdingObject) tInitialHold = -1.0
                        if (timer.get() > rumbleTimer) {
                            OI.operatorController.rumble = 0.0
                            OI.driverController.rumble = 0.0
                        }
                        if (OI.operatorLeftTrigger < 0.1) {
                            this.stop()
                        }
                    }
                })
            }
        } finally {//move back to drive pos
            Drive.maxTranslation = 1.0
            groundBackToDrive(isCone)
        }
    } else {
        println("Wrong side--flip first!!")
    }
}
suspend fun groundBackToDrive(isCone: Boolean) {
    println("Holding = ${Intake.holdingObject}")
    OI.driverController.rumble = 0.0
    OI.operatorController.rumble = 0.0
    Intake.intakeMotor.setPercentOutput(if (Intake.holdingObject) (if (isCone) Intake.HOLD_CONE else Intake.HOLD_CUBE) else 0.0)
    if (!DriverStation.isAutonomous()) {
        Arm.isFlipping = true
    }
    if (Intake.holdingObject) {
        if (isCone) {
            animateThroughPoses(Pose.GROUND_TO_DRIVE_SAFE_CONE, Pose.GROUND_TO_DRIVE_SAFE, Pose.BACK_DRIVE_POSE)
        } else {
            animateThroughPoses(Pose.GROUND_TO_DRIVE_SAFE_CUBE, Pose.GROUND_TO_DRIVE_SAFE, Pose.BACK_DRIVE_POSE)
        }
    } else {
        animateThroughPoses(Pose.FRONT_DRIVE_POSE)
    }
    Arm.isFlipping = false
}

suspend fun backScoreToward(isCone: Boolean = NodeDeckHub.isCone, pieceNumber: Int = NodeDeckHub.selectedNode.toInt()) = use(Arm, Intake) {
    if (Arm.wristPosition.x < -10.0 || Intake.wristAngle.asDegrees < -40.0) {
        Intake.coneToward = true
        Drive.maxTranslation = 0.3
        if (Arm.autoArmEnabled) {
            Drive.autoAim = true
        }
        if (isCone) {
            when (FieldManager.nodeList[pieceNumber]?.level) {
                Level.HIGH -> {
                    animateThroughPoses(
                        Pair(0.7, Pose.BACK_HIGH_SCORE_CONE_TOWARD_MID),
                        Pair(0.3, Pose.BACK_HIGH_SCORE_CONE_TOWARD)
                    )
                    autoArmToPose(Pose.BACK_HIGH_SCORE_CONE_TOWARD)
                }
                Level.MID -> {
                    animateThroughPoses(
                        Pair(0.0, Pose.BACK_MIDDLE_SCORE_CONE_TOWARD_MID),
                        Pair(0.0, Pose.BACK_MIDDLE_SCORE_CONE_TOWARD)
                    )
                    autoArmToPose(Pose.BACK_MIDDLE_SCORE_CONE_TOWARD)
                }
                Level.LOW -> {
                    animateToPose(Pose.BACK_LOW_SCORE_CONE_TOWARD)
                    autoArmToPose(Pose.BACK_LOW_SCORE_CONE_TOWARD)
                }
                else -> {
                    println("Error: Node level not given back")
                }
            }
        } else {
            lineUpScoreCube(pieceNumber)
        }
    } else {
        println("Wrong side--flip first!!")
    }
}

suspend fun backScoreAway(isCone: Boolean = NodeDeckHub.isCone, pieceNumber: Int = NodeDeckHub.selectedNode.toInt()) = use(Arm, Intake) {
    if (Arm.wristPosition.x < -10.0 || Intake.wristAngle.asDegrees < -40.0) {
        Intake.coneToward = false
        Drive.maxTranslation = 0.3 //make this a constant
        if (Arm.autoArmEnabled) {
            Drive.autoAim = true
        }
        if (isCone) {
            when (FieldManager.nodeList[pieceNumber]?.level) {
                Level.HIGH -> {
                    animateThroughPoses(
                        Pair(0.9, Pose.BACK_HIGH_SCORE_CONE_AWAY_MID),
                        Pair(0.4, Pose.BACK_HIGH_SCORE_CONE_AWAY)
                    )
                    autoArmToPose(Pose.BACK_HIGH_SCORE_CONE_AWAY)
                }
                Level.MID -> {
                    animateThroughPoses(
                        Pair(0.0, Pose.BACK_MIDDLE_SCORE_CONE_AWAY_MID),
                        Pair(0.0, Pose.BACK_MIDDLE_SCORE_CONE_AWAY)
                    )
                    autoArmToPose(Pose.BACK_MIDDLE_SCORE_CONE_AWAY)
                }
                Level.LOW -> {
                    animateToPose(Pose.BACK_LOW_SCORE_CONE_AWAY)
                    autoArmToPose(Pose.BACK_LOW_SCORE_CONE_AWAY)
                }
                else -> {
                    println("Error: Node level not given back")
                }
            }
        } else {
            lineUpScoreCube(pieceNumber)
        }
    } else {
        println("Wrong side--flip first!!")
    }
}

suspend fun lineUpScoreCube(pieceNumber: Int = NodeDeckHub.selectedNode.toInt()) = use(Arm, Intake) {
    if (Arm.wristPosition.x < -10.0 || Intake.wristAngle.asDegrees < -40.0) {
        Drive.maxTranslation = 0.3
        if (Arm.autoArmEnabled) {
            Drive.autoAim = true
        }
        when (FieldManager.nodeList[pieceNumber]?.level) {
            Level.HIGH -> {
                animateThroughPoses(
                    Pair(0.5, Pose.BACK_HIGH_SCORE_CUBE_MID),
                    Pair(0.0, Pose.BACK_HIGH_SCORE_CUBE)
                )
                autoArmToPose(Pose.BACK_HIGH_SCORE_CUBE)
            }
            Level.MID -> {
                animateThroughPoses(
                    Pair(0.3, Pose.BACK_MIDDLE_SCORE_CUBE_MID),
                    Pair(0.0, Pose.BACK_MIDDLE_SCORE_CUBE)
                )
                autoArmToPose(Pose.BACK_MIDDLE_SCORE_CUBE)
            }
            Level.LOW -> {
                animateToPose(Pose.BACK_LOW_SCORE_CUBE)
                autoArmToPose(Pose.BACK_LOW_SCORE_CUBE)
            }
            else -> {
                println("Error: Node level not given back")
            }
        }
    } else {
        println("Wrong side--flip first!!")
    }
}

private suspend fun autoArmToPose(pose: Pose) {
    val selectedNode = FieldManager.getSelectedNode()
    if (Arm.autoArmEnabled && selectedNode != null) {
        periodic {
            if (!OI.operatorController.leftBumper && !OI.operatorController.rightBumper) {
                Drive.autoAim = false
                this.stop()
            }
            val deltaBumper =
                Arm.distanceToTarget.asInches - 16.0 - (selectedNode.position.y - FieldManager.mirroredGridFromCenterY.asFeet).absoluteValue.feet.asInches
            val directionToNode = selectedNode.position - PoseEstimator.currentPose
            Drive.angleToNode = directionToNode.angle + 180.0.degrees
            Arm.wristPosition.x = pose.wristPosition.x - deltaBumper
            // println("delta2=$delta wristpos=${Arm.wristPosition.x}")
            Arm.deltaValueEntry.setDouble(deltaBumper)
            Arm.nodeAngleEntry.setDouble(Drive.angleToNode.asDegrees)
        }
    }
}

suspend fun flip(overrideFront: Boolean? = null) = use(Arm, Intake) {
    Drive.maxTranslation = 1.0
    println("overrideFront: $overrideFront")
    if (Intake.wristAngle < -75.0.degrees || Arm.wristPosition.x < -10.0 || overrideFront == true) {
        println("Flipping to FRONT")
        animateThroughPoses(Pair(0.5, Pose.BACK_DRIVE_POSE_CENTER), Pair(0.5, Pose.FLIP_FRONT_UP), Pair(0.5, Pose.FLIP_FRONT_WRIST), Pair(0.5, Pose.FRONT_DRIVE_POSE))//, Pose.FLIP_INTAKE_TO_FRONT_WRIST, Pose.FRONT_DRIVE_POSE)
    } else if (Intake.wristAngle > 75.0.degrees || Arm.wristPosition.x > 10.0 || overrideFront == false) {
        println("Flipping to BACK")
        animateThroughPoses(Pair(0.5, Pose.FRONT_DRIVE_POSE_CENTER), Pair(0.5, Pose.FLIP_BACK_UP), Pair(0.5, Pose.FLIP_BACK_WRIST), Pair(0.5, Pose.BACK_DRIVE_POSE))//, Pose.FLIP_INTAKE_TO_BACK_WRIST, Pose.BACK_DRIVE_POSE)
    } else {
        println("Don't know which side to flip")
    }
}

suspend fun scoreObject(pieceNumber: Int = NodeDeckHub.selectedNode.toInt()) = use(Arm, Intake) {
    println("in scoreObject")
    val nodeLevel = FieldManager.nodeList[pieceNumber]?.level
    val isCone = if (DriverStation.isTeleop()) NodeDeckHub.isCone else FieldManager.nodeList[pieceNumber]?.coneOrCube == GamePiece.CONE
    println("isCone? ${isCone}     nodeLevel? ${nodeLevel  == Level.LOW}")
    if (((nodeLevel == Level.MID || nodeLevel == Level.HIGH) && Arm.wristPosition.x < -15.0) || (nodeLevel == Level.LOW && Intake.wristAngle.asDegrees < -40.0 && !DriverStation.isAutonomous())) {
        Drive.maxTranslation = 0.5
        if (isCone) {
            if (Intake.coneToward) {
                when (nodeLevel) {
                    Level.HIGH -> {
                        var midPose = Pose.current + Pose(Vector2(3.5, -4.5), 50.0.degrees, 0.0.degrees)
                        animateToPose(midPose, 0.5,  true)
                        Intake.intakeMotor.setPercentOutput(Intake.CONE_TOWARD_SPIT)
                        midPose += Pose(Vector2(12.0, 14.0), 0.0.degrees, 0.0.degrees)
                        animateToPose(midPose, 0.4)
                        midPose += Pose(Vector2(18.0, 20.0), 0.0.degrees, 0.0.degrees)
                        animateToPose(midPose)
                    }
                    Level.MID -> {
                        val midPose = Pose.current + Pose(Vector2(11.0, -2.5), 50.0.degrees, 0.0.degrees)
                        animateThroughPoses(true, midPose)
                        Intake.intakeMotor.setPercentOutput(Intake.CONE_TOWARD_SPIT)
                        animateToPose(midPose + Pose(Vector2(6.0, -2.0), 10.0.degrees, 0.0.degrees))
                    }
                    Level.LOW -> {
                        Intake.intakeMotor.setPercentOutput(Intake.CONE_TOWARD_SPIT)
                        delay(0.5)
                    }
                    else -> println("Currently can't score there.")
                }
            } else { // cone away
                when (nodeLevel) {
                    Level.HIGH -> {
                        var midPose = Pose.current + Pose(Vector2(3.0, -7.0), 50.0.degrees, 0.0.degrees)
                        animateToPose(midPose, 0.5)
                        Intake.intakeMotor.setPercentOutput(Intake.CONE_AWAY_SPIT)
                        if (!DriverStation.isAutonomous()) {
                            midPose += Pose(Vector2(7.5, 14.0), 0.0.degrees, 0.0.degrees)
                            val midPose2 = midPose + Pose(Vector2(16.0, 17.0), 0.0.degrees, 0.0.degrees)
                            animateThroughPoses(midPose, midPose2)
                        } else {
                            midPose += Pose(Vector2(9.0, 16.0), 0.0.degrees, 0.0.degrees)
                            animateToPose(midPose)
                        }
                    }
                    Level.MID -> {
                        val midPose = Pose.current + Pose(Vector2(7.0, -6.5), 40.0.degrees, 0.0.degrees)
                        animateToPose(midPose, 1.0)
                        Intake.intakeMotor.setPercentOutput(Intake.CONE_AWAY_SPIT)
                        animateToPose(midPose + Pose(Vector2(6.0, -2.0), 10.0.degrees, 0.0.degrees))
                    }
                    Level.LOW -> {
                        Intake.intakeMotor.setPercentOutput(Intake.CONE_AWAY_SPIT)
                        delay(1.0)
                    }
                    else -> println("Currently can't score there.")
                }
            }
        } else {
            Intake.intakeMotor.setPercentOutput(Intake.CUBE_SPIT)
            when (nodeLevel) {
                Level.LOW -> delay(0.8)
                Level.MID -> animateToPose(Pose.current + Pose(Vector2(12.0, 12.0), 0.0.degrees, 0.0.degrees), 0.3)
                Level.HIGH -> {
                    if (DriverStation.isAutonomous()) delay(0.4)
                    animateToPose(Pose.current + Pose(Vector2(14.0, 0.0), 0.0.degrees, 0.0.degrees), 0.3)
                }
                else -> println("Currently can't score there.")
            }
        }

        resetArmVars()
        Intake.intakeMotor.setPercentOutput(0.0)
        if (!DriverStation.isAutonomous()) {
            afterScoreFlip(nodeLevel)
        }
        Drive.maxTranslation = 1.0
    } else {
        println("Wrong side to score, flip first!")
    }
}
suspend fun afterScoreFlip(nodeLevel: Level?) = use(Arm, Intake) {
    println("going to drive pos after score. nodeLevel: $nodeLevel")
    when (nodeLevel) {
        Level.HIGH -> animateThroughPoses(Pose.HIGH_SCORE_TO_PREFLIP, Pose.SCORE_TO_FLIP, Pose.FRONT_DRIVE_POSE)
        Level.MID -> {
            if (NodeDeckHub.isCone) {
                animateThroughPoses(Pose.MIDDLE_SCORE_CONE_TO_PREFLIP, Pose.SCORE_TO_FLIP, Pose.FRONT_DRIVE_POSE)
            } else {
                animateThroughPoses(Pose.MIDDLE_SCORE_CUBE_TO_PREFLIP, Pose.SCORE_TO_FLIP, Pose.FRONT_DRIVE_POSE)
            }
        }
        Level.LOW -> {
            animateThroughPoses(Pose.BACK_DRIVE_POSE)
            flip(true)
        }
        else -> println("No level selected :(")
    }
}

suspend fun toDrivePose() = use(Arm, Intake) {
    resetArmVars()
    if (Arm.wristPosition.x < -10.0 || Intake.wristAngle.asDegrees < -55.0) animateToPose(Pose.BACK_DRIVE_POSE) else if (Arm.wristPosition.x > 10.0 || Intake.wristAngle.asDegrees > 55.0) animateToPose(
        Pose.FRONT_DRIVE_POSE
    )
    Drive.maxTranslation = 1.0
}
suspend fun toFrontDrivePose() = use(Arm, Intake) {
    println("inside toFrontDrivePose")
    resetArmVars()
    println("starting to animate to drive")
    animateToPose(Pose.FRONT_DRIVE_POSE)
    println("finished animating to drive")
    Drive.maxTranslation = 1.0
}
suspend fun toBackDrivePose() = use(Arm, Intake) {
    resetArmVars()
    animateToPose(Pose.BACK_DRIVE_POSE)
    Drive.maxTranslation = 1.0
}

suspend fun  resetArmVars() = use(Arm, Intake) {
    Intake.wristOffset = 0.0.degrees
    Intake.pivotOffset = 0.0.degrees
    Arm.wristCenterOffset = Vector2(0.0, 0.0)
    Arm.isFlipping = false
    if (!Intake.holdingObject) Intake.intakeMotor.setPercentOutput(0.0)
}

suspend fun backScoreAuto(isCone: Boolean, pieceNumber: Int) = use(Arm, Intake) {
    if (Arm.wristPosition.x < -10.0 || Intake.wristAngle.asDegrees < -40.0) {
        Intake.coneToward = true
        Drive.maxTranslation = 0.3
        if (isCone) {
            when (FieldManager.nodeList[pieceNumber]?.level) {
                Level.HIGH -> {
                    animateToPose(Pose.BACK_HIGH_SCORE_CONE_TOWARD_MID, 1.0)
                    animateToPose(
                        Pose.BACK_HIGH_SCORE_CONE_TOWARD + Pose(Vector2(-1.0, 0.0), 0.0.degrees, 0.0.degrees),
                        0.7
                    ) //adding 1 up because in auto no drivers to save it
                }
                Level.MID -> {
                    animateToPose(Pose.BACK_MIDDLE_SCORE_CONE_TOWARD_MID)
                    animateToPose(Pose.BACK_MIDDLE_SCORE_CONE_TOWARD)
                }
                Level.LOW -> println("Low is not an option yet")
                else -> {
                    println("Error: Node level not given back")
                }
            }
        } else {
            lineUpScoreCube(pieceNumber)
        }
    } else {
        println("Wrong side--flip first!!")
    }
    scoreObject(pieceNumber)
}

suspend fun scoreObjectAuto(isCone: Boolean, pieceNumber: Int) = use(Arm, Intake) {
    println("in scoreObject")
    if (isCone) {
        if (Intake.coneToward) {
            when (FieldManager.nodeList[pieceNumber]?.level) {
                Level.HIGH -> {
                    var midPose = Pose.current + Pose(Vector2(6.0, -2.5), 40.0.degrees, 0.0.degrees)//6.0, -2.5
                    animateToPose(midPose, 0.5)
                    Intake.intakeMotor.setPercentOutput(Intake.CONE_TOWARD_SPIT)
                    midPose += Pose(Vector2(7.0, 7.5), 0.0.degrees, 0.0.degrees) // 6.5, 6.0
                    animateToPose(midPose, 0.3)
                    midPose += Pose(Vector2(10.0, 6.0), 0.0.degrees, 0.0.degrees)//10.0, 8.0
                    animateToPose(midPose, 0.3)
                }
                Level.MID -> {
                    val midPose = Pose.current + Pose(Vector2(10.0, -2.0), 40.0.degrees, 0.0.degrees) //10.0, -2.0
                    animateToPose(midPose, 0.5) //1.0
                    Intake.intakeMotor.setPercentOutput(Intake.CONE_TOWARD_SPIT)
                    animateToPose(midPose + Pose(Vector2(6.0, -2.0), 10.0.degrees, 0.0.degrees))//6.0, -2.0
                }

                else -> println("Currently can't score there.")
            }
        } else { // cone away
            when (FieldManager.nodeList[pieceNumber]?.level) {
                Level.HIGH -> {
                    var midPose = Pose.current + Pose(Vector2(3.0, -8.0), 40.0.degrees, 0.0.degrees)//3.0, -8.0
                    animateToPose(midPose, 1.0)
                    Intake.intakeMotor.setPercentOutput(Intake.CONE_AWAY_SPIT)
                    midPose += Pose(Vector2(6.5, 1.0), 0.0.degrees, 0.0.degrees)//6.5, 1.0
                    animateToPose(midPose)
                    midPose += Pose(Vector2(10.0, 6.0), 0.0.degrees, 0.0.degrees)//10.0, 6.0
                    animateToPose(midPose, 1.5)
                }
                Level.MID -> {
                    val midPose = Pose.current + Pose(Vector2(7.0, -4.0), 40.0.degrees, 0.0.degrees)//7.0, -4.0
                    animateToPose(midPose, 1.0)
                    Intake.intakeMotor.setPercentOutput(Intake.CONE_AWAY_SPIT)
                    animateToPose(midPose + Pose(Vector2(6.0, -2.0), 10.0.degrees, 0.0.degrees))//6.0, -2.0
                }
                else -> println("Currently can't score there.")
            }
        }
    } else {
        Intake.intakeMotor.setPercentOutput(Intake.CUBE_SPIT)
        when (FieldManager.nodeList[pieceNumber]?.level) {
            Level.MID -> animateToPose(Pose.current + Pose(Vector2(3.0, 0.0), 0.0.degrees, 0.0.degrees))
            Level.HIGH -> animateToPose(Pose.current + Pose(Vector2(12.0, 0.0), 0.0.degrees, 0.0.degrees))
            else -> println("No pose change necessary to score here.")
        }
    }
    Intake.intakeMotor.setPercentOutput(0.0)
//    toDrivePose()
}

suspend fun quickSpit() =use(Intake) {
    Intake.intakeMotor.setPercentOutput(if (NodeDeckHub.isCone) Intake.CONE_TOWARD_SPIT else Intake.CUBE_SPIT)
    delay(0.5)
    Intake.intakeMotor.setPercentOutput(0.0)
}
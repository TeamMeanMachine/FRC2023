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
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.Timer

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

    suspend fun intakeFromGround(isCone: Boolean = NodeDeckHub.isCone) = use(Arm, Intake) {
        if (Intake.wristAngle.asDegrees > 80.0) {
            try {
                println("in intakeFromGround")
                val path = Path2D("newPath")
                path.addVector2(Pose.current.wristPosition)
                if (isCone) {
                    path.addVector2(Pose.GROUND_INTAKE_FRONT_CONE.wristPosition)
                    path.addVector2(Pose.GROUND_INTAKE_CONE_NEAR.wristPosition)
                    path.addVector2(Pose.GROUND_INTAKE_CONE_FAR.wristPosition)
                } else {
                    path.addVector2(Pose.GROUND_INTAKE_FRONT_CUBE.wristPosition)
                    path.addVector2(Pose.GROUND_INTAKE_CUBE_NEAR.wristPosition)
                    path.addVector2(Pose.GROUND_INTAKE_CUBE_FAR.wristPosition)
                }
                val distance = path.length
                val rate = 55.0  //  inches per second
                val time = distance / rate
                path.addEasePoint(0.0, 0.0)
                path.addEasePoint(time, 1.0)

                val startOfExtend = time * 0.5

                val wristCurve = MotionCurve()
                wristCurve.storeValue(0.0, Pose.current.wristAngle.asDegrees)
                wristCurve.storeValue(time * 0.35, Pose.GROUND_INTAKE_FRONT_CONE.wristAngle.asDegrees)
                if (isCone) {
                    wristCurve.storeValue(startOfExtend, Pose.GROUND_INTAKE_CONE_NEAR.wristAngle.asDegrees)
                    wristCurve.storeValue(time, Pose.GROUND_INTAKE_CONE_FAR.wristAngle.asDegrees)
                } else {
                    wristCurve.storeValue(startOfExtend, Pose.GROUND_INTAKE_CUBE_NEAR.wristAngle.asDegrees)
                    wristCurve.storeValue(time, Pose.GROUND_INTAKE_CUBE_FAR.wristAngle.asDegrees)
                }

                val pivotCurve = MotionCurve()
                pivotCurve.storeValue(0.0, Pose.current.pivotAngle.asDegrees)
                if (isCone) {
                    pivotCurve.storeValue(startOfExtend * 0.7, -90.0)
                    pivotCurve.storeValue(startOfExtend * 0.9, -70.0)
                    pivotCurve.storeValue(startOfExtend, Pose.GROUND_INTAKE_CONE_NEAR.pivotAngle.asDegrees)
                    pivotCurve.storeValue(time, Pose.GROUND_INTAKE_CONE_FAR.pivotAngle.asDegrees)
                } else {
                    pivotCurve.storeValue(time, Pose.GROUND_INTAKE_CUBE_NEAR.pivotAngle.asDegrees)
                    pivotCurve.storeValue(startOfExtend, Pose.GROUND_INTAKE_CUBE_FAR.pivotAngle.asDegrees)
                }

                val slewRateLimiter = SlewRateLimiter(1.0, -1.0, 0.0)

                val timer = Timer()
                timer.start()
                Drive.maxTranslation = 0.5
                periodic {
                    val t = timer.get()
                    Arm.wristPosition = path.getPosition(t)
                    Intake.wristSetpoint = wristCurve.getValue(t).degrees
                    Intake.pivotSetpoint = pivotCurve.getValue(t).degrees
                    if (t > startOfExtend || (OI.operatorLeftTrigger < 0.05 && DriverStation.isTeleop())) {
                        this.stop()
                    }
                }
                Intake.intakeMotor.setPercentOutput(if (isCone) Intake.INTAKE_CONE else Intake.INTAKE_CUBE) //intake bad
                if (OI.operatorLeftTrigger > 0.05 || DriverStation.isAutonomous()) {
                    var tInitialHold = -1.0
                    parallel ({
                        periodic {
                            val tHold = if (tInitialHold != -1.0) timer.get() - tInitialHold else -1.0
                            val alpha2 = if (DriverStation.isTeleop()) slewRateLimiter.calculate((OI.operatorLeftTrigger - 0.1) * 10.0 / 9.0) else slewRateLimiter.calculate(1.0)
                            val tPath = linearMap(0.0, 1.0, startOfExtend, time, alpha2)
                            Arm.wristPosition =
                                path.getPosition(tPath) + if (tHold > 1.0) Vector2(-4.0, 4.0) else Vector2(0.0, 0.0)
                            Intake.wristSetpoint = wristCurve.getValue(tPath).degrees
                            Intake.pivotSetpoint = pivotCurve.getValue(tPath).degrees
                            Intake.intakeMotor.setPercentOutput(if (isCone) (if (tHold > 2.0) Intake.HOLD_CONE else Intake.INTAKE_CONE) else (if (tHold > 2.0) Intake.HOLD_CUBE else Intake.INTAKE_CUBE))
                            if (DriverStation.isTeleop() && OI.operatorLeftTrigger < 0.1) {
                                this.stop()
                            }
                            if (DriverStation.isAutonomous() && tPath >= time) {
                                this.stop()
                            }
                        }
                        if (DriverStation.isAutonomous()) {
                            val timer2 = Timer()
                            timer2.start()
                            var holdingTime = 50.0
                            periodic {
                                if (timer2.get() > 2.0 || timer2.get() - holdingTime > 0.3) {
                                    this.stop()
                                }
                                if (Intake.holdingObject && holdingTime == 50.0) {
                                    holdingTime = timer2.get()
                                }
                            }
                        }
                    }, {
                        periodic {
                            if (Intake.holdingObject && tInitialHold == -1.0) tInitialHold = timer.get()
                            if (!Intake.holdingObject) tInitialHold = -1.0
                            if (OI.operatorLeftTrigger < 0.1 || DriverStation.isAutonomous()) {
                                this.stop()
                            }
                        }
                    })
                }
// play animation backwards
//    periodic {
//        val t = timer.get()
//        Arm.endEffectorPosition = path.getPosition(t)
//        Intake.wristSetpoint = wristCurve.getValue(t).degrees
//        Intake.pivotSetpoint = pivotCurve.getValue(t).degrees
//        if (t>startOfExtend || OI.operatorController.rightTrigger < 0.05) {
//            this.stop()
//        }
//    }
            } finally {
                Drive.maxTranslation = 1.0
                println("Holding = ${Intake.holdingObject}")
                Intake.intakeMotor.setPercentOutput(if (Intake.holdingObject) (if (isCone) Intake.HOLD_CONE else Intake.HOLD_CUBE) else 0.0) //intake bad
                animateToPose(if (isCone) Pose.GROUND_INTAKE_FRONT_CONE else Pose.GROUND_INTAKE_FRONT_CUBE)
                toDrivePose()
            }
        } else {
            println("Wrong side--flip first!!")
        }
    }

    suspend fun backScoreTowardCone() = use(Arm, Intake) {
        if (Arm.wristPosition.x < -10.0 || Intake.wristAngle.asDegrees < -40.0) {
            Intake.coneToward = true
            Drive.maxTranslation = 0.3
            if (NodeDeckHub.isCone) {
                when (FieldManager.getSelectedNode()?.level) {
                    Level.HIGH -> {
                        animateToPose(Pose.BACK_HIGH_SCORE_CONE_TOWARD_MID, 1.0)
                        animateToPose(Pose.BACK_HIGH_SCORE_CONE_TOWARD, 0.5)
                    }
                    Level.MID -> {
                        animateToPose(Pose.BACK_MIDDLE_SCORE_CONE_TOWARD_MID)
                        animateToPose(Pose.BACK_MIDDLE_SCORE_CONE_TOWARD)
                    }
                    Level.LOW -> animateToPose(Pose.BACK_LOW_SCORE_CONE_TOWARD)
                    else -> {
                        println("Error: Node level not given back")
                    }
                }
            } else {
                lineUpScoreCube()
            }
        } else {
            println("Wrong side--flip first!!")
        }
    }

suspend fun backScoreAwayCone() = use(Arm, Intake) {
    if (Arm.wristPosition.x < -10.0 || Intake.wristAngle.asDegrees < -40.0) {
        Intake.coneToward = false
        Drive.maxTranslation = 0.3 //make this a constant
        if (NodeDeckHub.isCone) {
            when (FieldManager.getSelectedNode()?.level) {
                Level.HIGH -> {
                    animateToPose(Pose.BACK_HIGH_SCORE_CONE_AWAY_MID, 1.0)
                    animateToPose(Pose.BACK_HIGH_SCORE_CONE_AWAY, 0.5)
                }
                Level.MID -> {
                    animateToPose(Pose.BACK_MIDDLE_SCORE_CONE_AWAY_MID)
                    animateToPose(Pose.BACK_MIDDLE_SCORE_CONE_AWAY)
                }
                Level.LOW -> animateToPose(Pose.BACK_LOW_SCORE_CONE_AWAY)
                else -> {
                    println("Error: Node level not given back")
                }
            }
        } else {
            lineUpScoreCube()
        }
    } else {
        println("Wrong side--flip first!!")
    }
}

    suspend fun lineUpScoreCube(selectedNode: Int = NodeDeckHub.selectedNode.toInt()) = use(Arm, Intake) {
        if (Arm.wristPosition.x < -10.0 || Intake.wristAngle.asDegrees < -40.0) {
            Drive.maxTranslation = 0.3
            when (FieldManager.nodeList[selectedNode]?.level) {
                Level.HIGH -> {
                    animateToPose(Pose.BACK_HIGH_SCORE_CONE_TOWARD_MID)
                    animateToPose(Pose.BACK_HIGH_SCORE_CONE_TOWARD)
                }
                Level.MID -> {
                    animateToPose(Pose.BACK_MIDDLE_SCORE_CUBE)
                }
                Level.LOW -> animateToPose(Pose.BACK_LOW_SCORE_CUBE)
                else -> {
                    println("Error: Node level not given back")
                }
            }
        } else {
            println("Wrong side--flip first!!")
        }
    }

    suspend fun flip() = use(Arm) {
        Drive.maxTranslation = 1.0
        if (Intake.wristAngle < -75.degrees || Arm.wristPosition.x < -10.0) {
            animateToPose(Pose.FLIP_INTAKE_TO_FRONT_POSE)
            animateToPose(Pose.FLIP_INTAKE_TO_FRONT_WRIST)
            animateToPose(Pose.FRONT_DRIVE_POSE)
        } else if (Intake.wristAngle > 75.0.degrees || Arm.wristPosition.x > 10.0) {
            animateToPose(Pose.FLIP_INTAKE_TO_BACK_POSE)
            animateToPose(Pose.FLIP_INTAKE_TO_BACK_WRIST)
            animateToPose(Pose.BACK_DRIVE_POSE)
        }
    }

    suspend fun scoreObject() = use(Arm, Intake) {
        println("in scoreObject")
        if (NodeDeckHub.isCone) {
            if (Intake.coneToward) {
                when (FieldManager.getSelectedNode()?.level) {
                    Level.HIGH -> {
                        var midPose = Pose.current + Pose(Vector2(6.0, -2.5), 40.0.degrees, 0.0.degrees)
                        animateToPose(midPose, 0.5)
                        Intake.intakeMotor.setPercentOutput(Intake.CONE_TOWARD_SPIT)
                        midPose += Pose(Vector2(6.5, 6.5), 0.0.degrees, 0.0.degrees)
                        animateToPose(midPose)
                        midPose += Pose(Vector2(10.0, 6.0), 0.0.degrees, 0.0.degrees)
                        animateToPose(midPose, 1.0)
                    }
                    Level.MID -> {
                        val midPose = Pose.current + Pose(Vector2(10.0, -2.0), 40.0.degrees, 0.0.degrees)
                        animateToPose(midPose, 1.0)
                        Intake.intakeMotor.setPercentOutput(Intake.CONE_TOWARD_SPIT)
                        animateToPose(midPose + Pose(Vector2(6.0, -2.0), 10.0.degrees, 0.0.degrees))
                    }
                    Level.LOW -> {
                        Intake.intakeMotor.setPercentOutput(Intake.CONE_TOWARD_SPIT)
                        delay(1.0)
                    }
                    else -> println("Currently can't score there.")
                }
            } else { // cone away
                when (FieldManager.getSelectedNode()?.level) {
                    Level.HIGH -> {
                        var midPose = Pose.current + Pose(Vector2(3.0, -6.0), 50.0.degrees, 0.0.degrees)
                        animateToPose(midPose, 1.0)
                        Intake.intakeMotor.setPercentOutput(Intake.CONE_AWAY_SPIT)
                        midPose += Pose(Vector2(6.5, 5.0), 0.0.degrees, 0.0.degrees)
                        animateToPose(midPose)
                        midPose += Pose(Vector2(10.0, 6.0), 0.0.degrees, 0.0.degrees)
                        animateToPose(midPose, 1.5)
                    }
                    Level.MID -> {
                        val midPose = Pose.current + Pose(Vector2(7.0, -4.0), 40.0.degrees, 0.0.degrees)
                        animateToPose(midPose, 1.0)
                        Intake.intakeMotor.setPercentOutput(Intake.CONE_AWAY_SPIT)
                        animateToPose(midPose + Pose(Vector2(6.0, -2.0), 10.0.degrees, 0.0.degrees))
                    }
                    Level.LOW -> {
                        Intake.intakeMotor.setPercentOutput(Intake.CONE_AWAY_SPIT)
                        delay(1.5)
                    }
                    else -> println("Currently can't score there.")
                }
            }
        } else {
            Intake.intakeMotor.setPercentOutput(Intake.CUBE_SPIT)
            when (FieldManager.getSelectedNode()?.level) {
                Level.LOW -> delay(1.0)
                Level.MID -> animateToPose(Pose.current + Pose(Vector2(3.0, 0.0), 0.0.degrees, 0.0.degrees))
                Level.HIGH -> animateToPose(Pose.current + Pose(Vector2(12.0, 0.0), 0.0.degrees, 0.0.degrees))
                else -> println("Currently can't score there.")
            }
        }
        Intake.intakeMotor.setPercentOutput(0.0)
        toDrivePose()
    }

    suspend fun toDrivePose() = use(Arm, Intake) {
        Intake.wristOffset = 0.0.degrees
        Intake.pivotOffset = 0.0.degrees
        Arm.wristCenterOffset = Vector2(0.0, 0.0)
        if (!Intake.holdingObject) Intake.intakeMotor.setPercentOutput(0.0)
        if (Arm.wristPosition.x < -10.0 || Intake.wristAngle.asDegrees < -55.0) animateToPose(Pose.BACK_DRIVE_POSE) else if (Arm.wristPosition.x > 10.0 || Intake.wristAngle.asDegrees > 55.0) animateToPose(Pose.FRONT_DRIVE_POSE)
        Drive.maxTranslation = 1.0
    }
suspend fun backScoreAuto(isCone: Boolean, pieceNumber: Int) = use(Arm, Intake) {
    if (Arm.wristPosition.x < -10.0 || Intake.wristAngle.asDegrees < -40.0) {
        Intake.coneToward = true
        Drive.maxTranslation = 0.3
        if (isCone) {
            when (FieldManager.nodeList[pieceNumber]?.level) {
                Level.HIGH -> {
                    animateToPose(Pose.BACK_HIGH_SCORE_CONE_TOWARD_MID, 1.0)
                    animateToPose(Pose.BACK_HIGH_SCORE_CONE_TOWARD, 0.5)
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
    scoreObjectAuto(isCone, pieceNumber)
}

suspend fun scoreObjectAuto(isCone: Boolean, pieceNumber: Int) = use(Arm, Intake) {
    println("in scoreObject")
    if (isCone) {
        if (Intake.coneToward) {
            when (FieldManager.nodeList[pieceNumber]?.level) {
                Level.HIGH -> {
                    var midPose = Pose.current + Pose(Vector2(6.0, -2.5), 40.0.degrees, 0.0.degrees)//6.0, -2.5
                    animateToPose(midPose, 0.3)
                    Intake.intakeMotor.setPercentOutput(Intake.CONE_TOWARD_SPIT)
                    midPose += Pose(Vector2(6.5, 6.5), 0.0.degrees, 0.0.degrees) // 6.5, 6.0
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


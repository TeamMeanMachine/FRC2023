package org.team2471.frc2023

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.filter.SlewRateLimiter
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

suspend fun intakeCone() = use(Intake, Arm) {
    Intake.pivotSetpoint = 90.0.degrees
    Arm.shoulderSetpoint = -18.0.degrees
    Arm.elbowSetpoint = -27.0.degrees
    delay(2.0)
    Intake.pivotSetpoint = 179.0.degrees
}
var isIntaking: Boolean = false
suspend fun tippedConeIntake() = use(Intake, Arm) {

    if (isIntaking == false) {
        println("Tip Cone Intake: ON")
        isIntaking = true
        parallel(
            { intakeCurrentLogic() },
            { animateToPose(Pose.GROUND_INTAKE_POSE_FAR) }
        )
    } else {
        println("Tip Cone Intake: OFF")
        Intake.intakeMotor.setPercentOutput(0.0) //intake bad
        animateToPose(Pose.FRONT_DRIVE_POSE)
        isIntaking = false
    }

}

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
            if (linearFilter.calculate(Intake.intakeMotor.current) > Intake.INTAKE_DETECT_CONE && intakeDetectTime == 10000.0) { //intake bad
                intakeDetectTime = t.get() + 0.5
                println("detected = ${intakeDetectTime}")
            }
            if (t.get() > intakeDetectTime) {
                Intake.holdingObject = true
                Intake.holdDetectedTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp()
                println("t_get = ${t.get()}")
            }
            if (t.get() > intakeDetectTime + 2.0) Intake.intakeMotor.setPercentOutput(Intake.INTAKE_HOLD_CONE) //intake bad
            if (OI.operatorRightTrigger < 0.05) {
                Intake.intakeMotor.setPercentOutput(if (Intake.holdingObject) Intake.INTAKE_HOLD_CONE else 0.0) //intake bad
                this.stop()
            }
        }
    }
}

    suspend fun intakeFromGround() = use(Arm, Intake) {
        if (Intake.wristAngle.asDegrees > 80.0) {
            try {
                println("in intakeFromGround")
                val path = Path2D("newPath")
                path.addVector2(Pose.current.wristPosition)
                path.addVector2(Pose.GROUND_INTAKE_FRONT.wristPosition)
                path.addVector2(Pose.GROUND_INTAKE_POSE_NEAR.wristPosition)
                path.addVector2(Pose.GROUND_INTAKE_POSE_FAR.wristPosition)
                val distance = path.length
                val rate = 55.0  //  inches per second
                val time = distance / rate
                path.addEasePoint(0.0, 0.0)
                path.addEasePoint(time, 1.0)

                val startOfExtend = time * 0.5

                val wristCurve = MotionCurve()
                wristCurve.storeValue(0.0, Pose.current.wristAngle.asDegrees)
                wristCurve.storeValue(time * 0.35, Pose.GROUND_INTAKE_FRONT.wristAngle.asDegrees)
                wristCurve.storeValue(startOfExtend, Pose.GROUND_INTAKE_POSE_NEAR.wristAngle.asDegrees)
                wristCurve.storeValue(time, Pose.GROUND_INTAKE_POSE_FAR.wristAngle.asDegrees)

                val pivotCurve = MotionCurve()
                pivotCurve.storeValue(0.0, Pose.current.pivotAngle.asDegrees)
                pivotCurve.storeValue(startOfExtend * 0.7, -90.0)
                pivotCurve.storeValue(startOfExtend * 0.9, -70.0)
                pivotCurve.storeValue(startOfExtend, Pose.GROUND_INTAKE_POSE_NEAR.pivotAngle.asDegrees)
                pivotCurve.storeValue(time, Pose.GROUND_INTAKE_POSE_FAR.pivotAngle.asDegrees)

                val slewRateLimiter = SlewRateLimiter(2.0, -2.0, 0.0)

                val timer = Timer()
                timer.start()
                Drive.maxTranslation = 0.5
                periodic {
                    val t = timer.get()
                    Arm.wristPosition = path.getPosition(t)
                    Intake.wristSetpoint = wristCurve.getValue(t).degrees
                    Intake.pivotSetpoint = pivotCurve.getValue(t).degrees
                    if (t > startOfExtend || OI.operatorController.leftTrigger < 0.05) {
                        this.stop()
                    }
                }
                Intake.intakeMotor.setPercentOutput(Intake.INTAKE_CONE) //intake bad
                if (OI.operatorController.leftTrigger > 0.05) {
                    var tInitialHold = -1.0
                    parallel({
                        periodic {
                            val tHold = if (tInitialHold != -1.0) timer.get() - tInitialHold else -1.0
                            val alpha =
                                slewRateLimiter.calculate((OI.operatorController.leftTrigger - 0.1) * 10.0 / 9.0)
                            val tPath = linearMap(0.0, 1.0, startOfExtend, time, alpha)
                            Arm.wristPosition =
                                path.getPosition(tPath) + if (tHold > 1.0) Vector2(-4.0, 4.0) else Vector2(
                                    0.0,
                                    0.0)
                            Intake.wristSetpoint = wristCurve.getValue(tPath).degrees
                            Intake.pivotSetpoint = pivotCurve.getValue(tPath).degrees
                            Intake.intakeMotor.setPercentOutput(if (tHold > 2.0) Intake.INTAKE_HOLD_CONE else Intake.INTAKE_CONE)
                            if (OI.operatorController.leftTrigger < 0.1) {
                                this.stop()
                            }
                        }
                    }, {
                        periodic {
                            if (Intake.holdingObject && tInitialHold == -1.0) tInitialHold = timer.get()
                            if (!Intake.holdingObject) tInitialHold = -1.0
                            if (OI.operatorController.leftTrigger < 0.1) {
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
                Intake.intakeMotor.setPercentOutput(if (Intake.holdingObject) -Intake.INTAKE_HOLD else 0.0) //intake bad
                animateToPose(Pose.GROUND_INTAKE_FRONT)
                animateToPose(Pose.FRONT_DRIVE_POSE)
                if (Intake.holdingObject) {
                    animateToPose(Pose.FLIP_INTAKE_TO_BACK_POSE)
                    animateToPose(Pose.FLIP_INTAKE_TO_BACK_WRIST)
                    animateToPose(Pose.BACK_DRIVE_POSE)
                }
            }
        } else {
            println("Wrong side--flip first!!")
        }
    }

    suspend fun backScoreAwayCone() = use(Arm, Intake) {
        if (Intake.wristAngle.asDegrees < -80.0) {
            Drive.maxTranslation = 0.3 //make this a constant
            when (FieldManager.getSelectedNode()?.level) {
                Level.HIGH -> {
                    animateToPose(Pose.BACK_HIGH_SCORE_CONE_AWAY_MID)
                    animateToPose(Pose.BACK_HIGH_SCORE_CONE_AWAY)
                }
                Level.MID -> {
                    animateToPose(Pose.BACK_MIDDLE_SCORE_CONE_AWAY)
                }
                Level.LOW -> println("Low is not an option yet")
                else -> { println("Error: Node level not given back") }
            }

        } else {
            println("Wrong side--flip first!!")
        }
    }

    suspend fun backScoreTowardCone() = use(Arm, Intake) {
        if (Intake.wristAngle.asDegrees < -80.0) {
            Drive.maxTranslation = 0.3
            when (FieldManager.getSelectedNode()?.level) {
                Level.HIGH -> {
                    animateToPose(Pose.BACK_HIGH_SCORE_CONE_TOWARD_MID)
                    animateToPose(Pose.BACK_HIGH_SCORE_CONE_TOWARD)
                }
                Level.MID -> {
                    animateToPose(Pose.BACK_MIDDLE_SCORE_CONE_TOWARD)
                }
                Level.LOW -> println("Low is not an option yet")
                else -> { println("Error: Node level not given back") }
            }
        } else {
            println("Wrong side--flip first!!")
        }
    }

    suspend fun flip() = use(Arm) {
        Drive.maxTranslation = 1.0
        if (Intake.wristAngle < -75.degrees) {
            animateToPose(Pose.FLIP_INTAKE_TO_FRONT_POSE)
            animateToPose(Pose.FLIP_INTAKE_TO_FRONT_WRIST)
            animateToPose(Pose.FRONT_DRIVE_POSE)
        } else if (Intake.wristAngle > 75.0.degrees) {
            animateToPose(Pose.FLIP_INTAKE_TO_BACK_POSE)
            animateToPose(Pose.FLIP_INTAKE_TO_BACK_WRIST)
            animateToPose(Pose.BACK_DRIVE_POSE)
        }
    }


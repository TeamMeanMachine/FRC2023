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
        Intake.intakeMotor.setPercentOutput(0.0)
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
        linearFilter.calculate(Intake.intakeMotor.current)
        if (!isTimerStarted) {
            t.start()
            isTimerStarted = true
            intakeDetectTime = 10000.0
            Intake.intakeMotor.setPercentOutput(-Intake.INTAKE_POWER)
            println("timer is started")
        } else if (t.get() > 2.0) {
            if (linearFilter.calculate(Intake.intakeMotor.current) > Intake.INTAKE_DETECT_CONE && intakeDetectTime == 10000.0) {
                intakeDetectTime = t.get() + 0.5
                println("detected = ${intakeDetectTime}")
            }
            if (t.get() > intakeDetectTime) {
                Intake.holdingObject = true
                println("t_get = ${t.get()}")
            }
            if (t.get() > intakeDetectTime + 2.0) Intake.intakeMotor.setPercentOutput(-Intake.INTAKE_HOLD)
        } else {
            //    println("Min Timer not Reached → ${t.get()}")
        }
        if (OI.operatorRightTrigger < 0.05) {
            Intake.intakeMotor.setPercentOutput(if (Intake.holdingObject) -Intake.INTAKE_HOLD else 0.0)
            this.stop()
        }
    }
}
suspend fun intakeFromGround() = use(Arm, Intake) {
    try {
        println("in intakeFromGround")
        val path = Path2D("newPath")
        path.addVector2(Pose.current.wristPosition)
        path.addVector2(Pose.GROUND_INTAKE_FRONT.wristPosition)
        path.addVector2(Pose.GROUND_INTAKE_POSE_NEAR.wristPosition)
        path.addVector2(Pose.GROUND_INTAKE_POSE_FAR.wristPosition)
        val distance = path.length
        val rate = 30.0  //  inches per second
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
        pivotCurve.storeValue(time * 0.25, -150.0)
        pivotCurve.storeValue(time * 0.35, -135.0)
        pivotCurve.storeValue(startOfExtend, Pose.GROUND_INTAKE_POSE_NEAR.pivotAngle.asDegrees)
        pivotCurve.storeValue(time, Pose.GROUND_INTAKE_POSE_FAR.pivotAngle.asDegrees)

        val slewRateLimiter = SlewRateLimiter(2.0, -2.0, 0.0)

        val timer = Timer()
        timer.start()
        periodic {
            val t = timer.get()
            Arm.wristPosition = path.getPosition(t)
            Intake.wristSetpoint = wristCurve.getValue(t).degrees
            Intake.pivotSetpoint = pivotCurve.getValue(t).degrees
            if (t > startOfExtend || OI.operatorController.rightTrigger < 0.05) {
                this.stop()
            }
        }
        Intake.intakeMotor.setPercentOutput(-Intake.INTAKE_POWER)
        if (OI.operatorController.rightTrigger > 0.05) {
            parallel({
                periodic {
                    val alpha = slewRateLimiter.calculate((OI.operatorController.rightTrigger - 0.1) * 10.0 / 9.0)
                    val t = linearMap(0.0, 1.0, startOfExtend, time, alpha)
                    Arm.wristPosition = path.getPosition(t) + if (Intake.holdingObject) Vector2(-4.0, 4.0) else Vector2(0.0, 0.0)
                    Intake.wristSetpoint = wristCurve.getValue(t).degrees
                    Intake.pivotSetpoint = pivotCurve.getValue(t).degrees
                    if (OI.operatorController.rightTrigger < 0.1) {
                        this.stop()
                    }
                }
            }, {
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
        Intake.intakeMotor.setPercentOutput(if (Intake.holdingObject) -Intake.INTAKE_HOLD else 0.0)
        animateToPose(Pose.GROUND_INTAKE_FRONT)
        animateToPose(Pose.FRONT_DRIVE_POSE)
    }
}

suspend fun backScoreAwayCone() = use(Arm, Intake) {
    animateToPose(Pose.BACK_MIDDLE_SCORE_CONE_AWAY)
}

suspend fun backScoreTowardCone() = use(Arm, Intake) {
    animateToPose(Pose.BACK_MIDDLE_SCORE_CONE_TOWARD)
}


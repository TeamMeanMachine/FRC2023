package org.team2471.frc2023

import edu.wpi.first.math.filter.LinearFilter
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.parallel
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.Timer

suspend fun scoreIfReady() {
    var startGoScore = false
    periodic {

//        println("Checking if driving... ${OI.driveRotation}  ${OI.driveTranslation.length}")

        if (!Drive.isHumanDriving) {
            startGoScore = true
            stop()
        }
    }
    if (startGoScore) {
        goScore()
    }
}


suspend fun goScore() = use(Drive, Arm, Intake, name = "goScore") {
    periodic {
        if (!OI.operatorController.x || Drive.isHumanDriving) {
            stop()
        }
//        val scoringPos = FieldConstants.Grids.high3dTranslations[NodeDeckHub.selectedNode.toInt()]
        println()
    }
}

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
            { animateToPose(Pose.GROUND_INTAKE_POSE) }
        )
    } else {
        println("Tip Cone Intake: OFF")
        Intake.intakeMotor.setPercentOutput(0.0)
        animateToPose(Pose.DRIVE_POSE)
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
                Intake.intakeMotor.setPercentOutput(-Intake.INTAKE_HOLD)  //1.0
                println("t_get = ${t.get()}")
            }
        } else {
            //    println("Min Timer not Reached → ${t.get()}")
        }
        isTimerStarted = false
    }
}

package org.team2471.frc2023

import kotlinx.coroutines.Delay
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.units.degrees

suspend fun scoreIfReady(){
    var startGoScore = false
    periodic {

        println("Checking if driving... ${OI.driveRotation}  ${OI.driveTranslation.length}")

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

suspend fun intakeCone() = use(Intake,Arm) {
    Intake.pivotSetpoint = 90.0.degrees
    Arm.shoulderSetpoint = -18.0.degrees
    Arm.elbowSetpoint = -27.0.degrees
    delay(2.0)
    Intake.pivotSetpoint = 179.0.degrees

}

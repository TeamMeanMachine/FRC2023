package org.team2471.frc2023

import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use

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

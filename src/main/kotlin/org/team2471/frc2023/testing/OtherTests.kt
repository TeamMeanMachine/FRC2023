package org.team2471.frc2023.testing

import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc2023.Arm
import org.team2471.frc2023.OI

suspend fun Arm.motorTest() = use(Arm){
    periodic {
        shoulderMotor.setPercentOutput(OI.operatorLeftX * 0.2)
        elbowMotor.setPercentOutput(OI.operatorRightX * 0.2)
    }
}
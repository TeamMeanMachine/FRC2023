package org.team2471.frc2023.testing

import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2023.Arm
import org.team2471.frc2023.Intake
import org.team2471.frc2023.OI

suspend fun Arm.feedForwardTest() = use(Arm) {
    var power = 0.0
    periodic {
        elbowMotor.setPercentOutput(power)
        println("power: $power")
        power += 0.003
    }
}

suspend fun Arm.pidTest() = use(Arm) {
    periodic {
        elbowSetpoint = (OI.operatorLeftY * 40).degrees
    }
}

suspend fun Intake.pidTest() = use(Arm) {
    periodic {
        pivotSetpoint = (OI.operatorLeftY * 180).degrees
    }
}

suspend fun Arm.springTest() = use(Arm) {
    elbowMotor.setPercentOutput(0.4)
}
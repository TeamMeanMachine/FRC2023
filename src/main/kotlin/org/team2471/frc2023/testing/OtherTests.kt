package org.team2471.frc2023.testing

import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2023.Arm
import org.team2471.frc2023.Intake
import org.team2471.frc2023.OI

suspend fun Intake.feedForwardTest() = use(Intake) {
    var power = 0.0
    periodic {
        pivotMotor.setPercentOutput(power)
        println("power: $power")
        power += 0.003
    }
}

suspend fun Arm.pidTest() = use(Arm) {
    periodic {
        elbowSetpoint = (OI.operatorLeftY * 60).degrees
    }
}

suspend fun Intake.pidTestOne() = use(this) {
    periodic {
        pivotSetpoint = (180 + OI.operatorLeftY * 80.0).degrees
    }
}

suspend fun Intake.pidTestTwo() = use(Intake) {
    periodic {
        pivotSetpoint = OI.operatorLeftX.degrees * 0.05
    }
}

suspend fun Arm.springTest() = use(Arm) {
    elbowMotor.setPercentOutput(0.4)
}
package org.team2471.frc2023.testing

import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2023.Arm
import org.team2471.frc2023.OI

suspend fun Arm.feedForwardTest() = use(Arm) {
    var power = 0.0
    periodic {
        shoulderMotor.setPercentOutput(power)
        println("power: $power")
        power += 0.003
    }
}

suspend fun Arm.pidTest() = use(Arm) {
    periodic {
        shoulderSetpoint = (OI.operatorLeftY * 50).degrees
    }
}
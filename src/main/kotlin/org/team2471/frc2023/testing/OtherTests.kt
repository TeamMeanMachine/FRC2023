package org.team2471.frc2023.testing

import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2023.Arm
import org.team2471.frc2023.Drive
import org.team2471.frc2023.Intake
import org.team2471.frc2023.OI

suspend fun Arm.feedForwardTest() = use(Arm) {
    var power = 0.0
    periodic {
        shoulderSetpoint = 0.0.degrees
        elbowMotor.setPercentOutput(power)
        println("power: $power elbow: ${elbowMotor.position} current ${elbowMotor.current}")
        power -= 0.001
    }
}

suspend fun Arm.pidTest() = use(Arm, Intake) {
    periodic {
        elbowSetpoint += (OI.operatorLeftY * 0.5).degrees
        wristPosOffset = Vector2(0.0, 0.0)
    }
}

suspend fun Intake.pidTestOne() = use(this) {
    periodic {
        pivotSetpoint = (180 + OI.operatorLeftY * 80.0).degrees
    }
}

suspend fun Intake.pidTestTwo() = use(Intake) {
    periodic {
        pivotSetpoint = (OI.operatorLeftX * 30.0).degrees
    }
}

suspend fun Arm.springTest() = use(Arm) {
    elbowMotor.setPercentOutput(0.4)
}

suspend fun Intake.feedFowardTest() = use(Intake) {
    var power = 0.0
    periodic {
        intakeMotor.setPercentOutput(power)
        power += 0.03
    }
}

suspend fun Intake.intakeTest() = use(Intake) {
    periodic {
        intakeMotor.setPercentOutput(OI.operatorLeftY)
        println("intake power: ${OI.operatorRightY}")
    }
}

suspend fun driveToPointsTest() = use(Drive) {

}
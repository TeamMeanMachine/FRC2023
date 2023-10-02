package org.team2471.frc2023.testing

import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2023.Arm
import org.team2471.frc2023.Drive
import org.team2471.frc2023.Intake
import org.team2471.frc2023.OI
import kotlin.math.absoluteValue

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
//        pivotSetpoint = (OI.operatorLeftY * 90.0).degrees
//        println("setpoint: $pivotSetpoint power ${pivotMotor.output}     current ${pivotMotor.current}")
    }
}
suspend fun Arm.controlerTest() = use(this) {
    periodic {
        val power = OI.operatorLeftY * 40.0
        elbowMotor.setPercentOutput(power)
        shoulderSetpoint = 0.0.degrees
        println("power ${round(power, 2)}  current ${round(elbowMotor.current, 2)}  angle ${round(elbowAngle.asDegrees, 2)}")
    }
}

suspend fun Intake.pidTestTwo() = use(Intake) {
    periodic {
//        pivotSetpoint = (OI.operatorLeftX * 30.0).degrees
    }
}

suspend fun Arm.springTest() = use(Arm) {
    elbowMotor.setPercentOutput(0.4)
}

suspend fun Intake.feedFowardTest() = use(Intake) {
    var power = 0.0
    periodic {
//        pivotMotor.setPercentOutput(power)
//        println("power $power  curent ${pivotMotor.current}  angle ${pivotAngle.asDegrees}")
//        power += 0.01
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

suspend fun autoFeedForwardTest(motor: MotorController, motorAngle: () -> Angle, angleList: ArrayList<Double>, powerInterval: Double = 0.01) {
    val powerList: ArrayList<Double> = ArrayList()

    for (angle in angleList) {
        var power = 0.0
        var lastAngleError: Double? = null
        periodic {
            val angleError = angle - motorAngle.invoke().asDegrees
            val rate = (lastAngleError ?: angleError) - angleError
            if (angleError.absoluteValue < 0.5 && rate.absoluteValue < 0.5) {
                powerList.add(power.round(2))
                this.stop()
            } else if (angleError > angle) {
                power += powerInterval
                motor.setPercentOutput(power)
            } else if (angleError < angle) {
                power -= powerInterval
                motor.setPercentOutput(power)
            }
            lastAngleError = angleError
        }
    }
    println("Angles: ${angleList}")
    println("Powers: ${powerList}")
}
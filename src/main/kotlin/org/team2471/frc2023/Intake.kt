package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogInput

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.*
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import kotlin.math.absoluteValue


object Intake : Subsystem("Intake") {
    val wristMotor = MotorController(TalonID(Talons.WRIST))
    val pivotMotor = MotorController(TalonID(Talons.INTAKE_PIVOT))
    val intakeMotor = MotorController(SparkMaxID(Sparks.INTAKE))
//    val wristSensor = hall effect
    val pivotSensor = AnalogInput(AnalogSensors.INTAKE_PIVOT)


    private val table = NetworkTableInstance.getDefault().getTable(Intake.name)
    val wristEntry = table.getEntry("Wrist Angle")
    val wristSetpointEntry = table.getEntry("Wrist Setpoint")
    val pivotEntry = table.getEntry("Pivot Angle")
    val pivotSetpointEntry = table.getEntry("Pivot Setpoint")

    val wristAngle: Angle
        get() = wristMotor.position.degrees + wristOffset
    var wristOffset = 0.0.degrees
    var wristSetpoint: Angle = wristAngle
        get() = wristSetpointEntry.getDouble(0.0).degrees
        set(value) {
            field = value.asDegrees.coerceIn(WRIST_BOTTOM, WRIST_TOP).degrees
            wristSetpointEntry.setDouble(field.asDegrees)
        }
    val pivotAngle: Angle
        get() = (analogPivotAngle + (if ((motorPivotAngle - analogPivotAngle).asDegrees.absoluteValue > 90.0) 180.0.degrees else 0.0.degrees) + pivotOffset).wrap()
    val analogPivotAngle: Angle
        get() = (pivotSensor.value / 2.0).degrees
    val motorPivotAngle: Angle
        get() = pivotMotor.position.degrees
    var pivotOffset = 0.0.degrees
    var pivotSetpoint: Angle = pivotAngle
        get() = pivotSetpointEntry.getDouble(0.0).degrees
        set(value) {
            field = value
            pivotSetpointEntry.setDouble(value.asDegrees)
        }

    var wristIsReset = false

    const val WRIST_BOTTOM = -40.0
    const val WRIST_TOP = 40.0
    const val INTAKE_POWER = 0.7

    init {
        wristMotor.config(20) {
//            feedbackCoefficient
            brakeMode()
//            pid
            currentLimit(20, 30, 1)
        }
        pivotMotor.config(20) {
//            feedbackCoefficient   // degrees in a rotation, ticks per rotation
            brakeMode()
//            pid
            currentLimit(20, 30, 1)
        }
        intakeMotor.config {
            coastMode()
            currentLimit(20, 40, 1)
        }

        pivotMotor.position = pivotSensor.value.toDouble()

        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                wristEntry.setDouble(wristAngle.asDegrees)
                pivotEntry.setDouble(pivotAngle.asDegrees)

                wristMotor.setPositionSetpoint(wristSetpoint.asDegrees)
                pivotMotor.setPositionSetpoint(pivotSetpoint.asDegrees)

                //zeroing wrist
//                if (!wristIsReset) println("Wrist angle is not zeroed")
//                if (hall effect
            //                true) {
//                    wristAngle = 0.0.degrees
//                    wristIsReset = true
//                }
            }
        }
    }

    override suspend fun default() {
        periodic {
            println("intakePeriodicOn")
            if (OI.driverController.a){
                intakeMotor.setPercentOutput(-1.0)
            }
            else if (OI.driverController.b){
                intakeMotor.setPercentOutput(1.0)
            }
            else{
                intakeMotor.setPercentOutput(0.5 * OI.driverController.leftTrigger - 0.5 * OI.driverController.rightTrigger)
            }
        }
    }
}



suspend fun Intake.pivotTest() = use(this) {
    var angle = pivotAngle
    var upPressed = false
    var downPressed = false
    periodic {
        if (OI.driverController.dPad == Controller.Direction.UP) {
            upPressed = true
        } else if (OI.driverController.dPad == Controller.Direction.DOWN) {
            downPressed = true
        }
        if (OI.driverController.dPad != Controller.Direction.UP && upPressed) {
            upPressed = false
            angle += 10.degrees
            println("up = ${angle}")
        }
        if (OI.driverController.dPad != Controller.Direction.DOWN && downPressed) {
            downPressed = false
            angle -= 10.degrees
            println("down = ${angle}")
        }
        pivotSetpoint = angle
        println("angle = ${angle}")
    }
}


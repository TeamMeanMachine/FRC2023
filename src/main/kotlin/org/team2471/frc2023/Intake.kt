package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogInput

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.*
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import kotlin.math.absoluteValue


object Intake : Subsystem("Intake") {
    val wristMotor = MotorController(SparkMaxID(Sparks.WRIST))
    val pivotMotor = MotorController(TalonID(Talons.INTAKE_PIVOT))
    val intakeMotor = MotorController(SparkMaxID(Sparks.INTAKE))
//    val wristSensor = hall effect
    val pivotSensor = AnalogInput(AnalogSensors.INTAKE_PIVOT)


    private val table = NetworkTableInstance.getDefault().getTable(Intake.name)
    val wristEntry = table.getEntry("Wrist Angle")
    val wristSetpointEntry = table.getEntry("Wrist Setpoint")
    val pivotEntry = table.getEntry("Pivot Angle")
    val pivotAnalogEntry = table.getEntry("Pivot Analog Angle")
    val pivotSetpointEntry = table.getEntry("Pivot Setpoint")

    val wristAngle: Angle
        get() = wristMotor.position.degrees + wristOffset
    var wristOffset = 90.0.degrees
    var wristSetpoint: Angle = wristAngle
        get() = wristSetpointEntry.getDouble(0.0).degrees
        set(value) {
            field = value.asDegrees.coerceIn(WRIST_BOTTOM, WRIST_TOP).degrees
            wristSetpointEntry.setDouble(field.asDegrees)
        }
    val pivotAngle: Angle
        get() = (pivotAnalogAngle + pivotOffset + pivotConversionOffset).wrap()
    val pivotIncrementAngle: Angle
        get() = pivotAngle
    val pivotAnalogAngle: Angle
        get() = ((pivotSensor.value - 15) / 4080.0 * 180.0).degrees  //(((pivotSensor.value - 1665.0) / 4095.0 * 180.0)).degrees.wrap() // -72.65.degrees
    var prevPivotAngle: Angle = pivotAngle
    var prevPivotAnalog = pivotAnalogAngle
    var pivotOffset: Angle = -68.2.degrees
    var pivotConversionOffset: Angle = 0.0.degrees
    var rotations: Double = 0.0
    var pivotSetpoint: Angle = pivotAngle
        get() = pivotSetpointEntry.getDouble(0.0).degrees
        set(value) {
            field = value
            pivotSetpointEntry.setDouble(value.asDegrees)
        }
    val pivotPDController = PDController(0.03, 0.04)   //0.35, 0.03

    var wristIsReset = false

    const val WRIST_BOTTOM = -40.0
    const val WRIST_TOP = 40.0
    const val INTAKE_POWER = 0.7

    init {
        wristMotor.config(20) {
            feedbackCoefficient = 261.0 / 1273.0 * 208.1 / 359.0 //redo!
            brakeMode()
//            pid     //percentOutput(0.1) started moving
            currentLimit(20, 30, 1)
        }
        pivotMotor.config(20) {
            inverted(true)
            brakeMode()
            currentLimit(30, 40, 1)
        }
        intakeMotor.config {
            brakeMode()
            currentLimit(30, 40, 1)
        }

        pivotMotor.position = pivotSensor.value.toDouble()


        GlobalScope.launch(MeanlibDispatcher) {
            var tempPivot = pivotAngle
            periodic {
                wristEntry.setDouble(wristAngle.asDegrees)
                pivotEntry.setDouble(pivotAngle.asDegrees)
                pivotAnalogEntry.setDouble(pivotAnalogAngle.asDegrees)

                if (pivotAnalogAngle >= 120.0.degrees && prevPivotAnalog < 20.0.degrees) pivotConversionOffset += 180.0.degrees
                if (pivotAnalogAngle < 20.0.degrees && prevPivotAnalog >= 120.0.degrees) pivotConversionOffset -= 180.0.degrees

                tempPivot = pivotAngle
                if (tempPivot < -100.0.degrees && prevPivotAngle >= -0.1.degrees) rotations += 1
                else if (tempPivot >= 100.0.degrees && prevPivotAngle < 0.1.degrees) rotations -= 1

//                wristMotor.setPositionSetpoint(wristSetpoint.asDegrees)

//                val power = pivotPDController.update(pivotSetpoint.asDegrees - pivotAngle.asDegrees)
//                pivotMotor.setPercentOutput(power)
//                if (power > maxPower) maxPower = power
//                println("pivotSetpoint: ${pivotSetpoint.asDegrees}       pdController: ${round(power, 3)}        maxPower: $maxPower")

                //zeroing wrist
//                if (!wristIsReset) println("Wrist angle is not zeroed")
//                if (hall effect
            //                true) {
//                    wristAngle = 0.0.degrees
//                    wristIsReset = true
//                }
//                if ((pivotAnalogAngle.asDegrees > 80.0 && prevPivotAnalog.asDegrees < -80.0) ||) inOtherZone = !inOtherZone
                prevPivotAnalog = pivotAnalogAngle
                prevPivotAngle = tempPivot
            }
        }
    }

    override suspend fun default() {
        periodic {
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


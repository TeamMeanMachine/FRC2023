package org.team2471.frc2023

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DigitalInput

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
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.Timer


object Intake : Subsystem("Intake") {
    val wristMotor = MotorController(SparkMaxID(Sparks.WRIST))
    val pivotMotor = MotorController(TalonID(Talons.INTAKE_PIVOT))
    val intakeMotor = MotorController(SparkMaxID(Sparks.INTAKE))
    val wristSensor = DigitalInput(DigitalSensors.WRIST_SWITCH)
    val pivotSensor = AnalogInput(AnalogSensors.INTAKE_PIVOT)

    private val table = NetworkTableInstance.getDefault().getTable(Intake.name)
    val wristEntry = table.getEntry("Wrist Angle")
    val wristSetpointEntry = table.getEntry("Wrist Setpoint")
    val pivotEntry = table.getEntry("Pivot Angle")
    val pivotAnalogEntry = table.getEntry("Pivot Analog Angle")
    val pivotSetpointEntry = table.getEntry("Pivot Setpoint")
    val intakeCurrentEntry = table.getEntry("Intake Currrent")
    val intakePowerEntry = table.getEntry("Intake Power")

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
        get() = (pivotAnalogAngle + pivotOffset + pivotConversionOffset).wrap()
    val pivotIncrementAngle: Angle
        get() = pivotAngle + (rotations * 360.0).degrees
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
    var pivotIncrementSetpoint: Angle = pivotAngle
        get() = pivotSetpoint + (rotations * 360.0).degrees
    val pivotPDController = PDController(0.017, 0.001) //0.03, 0.04)   //0.35, 0.03
    val pFeedForward: Double  //later: include wristAngle to include facing backwards
        get() = pivotCurve.getValue(pivotAngle.asDegrees /*    + 180.0.degrees * sin(wristAngle)    then wrap whole thing   */)
    val pivotCurve = MotionCurve()

    var wristIsReset = false

    const val WRIST_BOTTOM = -90.0 //Arm.elbowAngle.asDegrees - 40.0 to be based on elbow's angle
    const val WRIST_TOP = 20.0  //Arm.elbowAngle.asDegrees + 40.0
    const val INTAKE_POWER = 1.0

    const val INTAKE_HOLD = 0.05
    const val INTAKE_DETECT_CONE = 55
    const val INTAKE_CURR = 55.0

    init {
        wristMotor.restoreFactoryDefaults()
        intakeMotor.restoreFactoryDefaults()
        wristMotor.config(20) {
            feedbackCoefficient = 261.0 / 1273.0 * 208.1 / 359.0 //redo!
            coastMode()
            pid {
                p(0.00001)
            }
            currentLimit(0, 60, 0)
            burnSettings()
        }
        pivotMotor.config(20) {
            inverted(true)
            brakeMode()
            currentLimit(30, 40, 1)
        }
        intakeMotor.config {
            brakeMode()
            currentLimit(0, 60, 0)
            burnSettings()
        }

        pivotMotor.position = pivotSensor.value.toDouble()
        wristMotor.setRawOffset(-90.0)
        GlobalScope.launch(MeanlibDispatcher) {
            var tempPivot: Angle

            pivotCurve.storeValue(-179.0, 0.0)
            pivotCurve.storeValue(-170.0, 0.18) //.05
            pivotCurve.storeValue(-90.0, 0.28)
            pivotCurve.storeValue(-45.0, 0.09)
            pivotCurve.storeValue(0.0, 0.0)
            pivotCurve.storeValue(45.0, -0.09)
            pivotCurve.storeValue(90.0, -0.28)
            pivotCurve.storeValue(170.0, -0.18)
            pivotCurve.storeValue(179.0, 0.0)

            wristSetpointEntry.setDouble(wristAngle.asDegrees)
            pivotSetpointEntry.setDouble(pivotAngle.asDegrees)

//            wristSetpoint = -90.0.degrees
//            pivotSetpoint = -180.0.degrees

            periodic {
                wristEntry.setDouble(wristAngle.asDegrees)
                pivotEntry.setDouble(pivotAngle.asDegrees)
                pivotAnalogEntry.setDouble(pivotAnalogAngle.asDegrees)
                intakeCurrentEntry.setDouble(intakeMotor.current)

                if (pivotAnalogAngle >= 120.0.degrees && prevPivotAnalog < 20.0.degrees) pivotConversionOffset += 180.0.degrees
                if (pivotAnalogAngle < 20.0.degrees && prevPivotAnalog >= 120.0.degrees) pivotConversionOffset -= 180.0.degrees

                tempPivot = pivotAngle
                if (tempPivot < -100.0.degrees && prevPivotAngle >= -0.1.degrees) rotations += 1
                else if (tempPivot >= 100.0.degrees && prevPivotAngle < 0.1.degrees) rotations -= 1

                wristMotor.setPositionSetpoint(wristSetpoint.asDegrees)

                val power = pivotPDController.update(pivotIncrementSetpoint.asDegrees - pivotIncrementAngle.asDegrees) + pFeedForward
//                pivotMotor.setPercentOutput(power)


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
        val t = Timer()
        var isTimerStarted = false
        var intakeDetected = 0.0
        var linearFilter = LinearFilter.movingAverage(5)
        periodic {
            if (OI.operatorController.a) {
                  //-1.0
                linearFilter.calculate(intakeMotor.current)
                if (!isTimerStarted) {
                    t.start()
                    isTimerStarted = true
                    intakeDetected = 10000.0
                    intakeMotor.setPercentOutput(-INTAKE_POWER)
                    println("timer is started")
                } else if (t.get() > 2.0) {
                    if ( linearFilter.calculate(intakeMotor.current) > INTAKE_DETECT_CONE && intakeDetected ==  10000.0) {
                        intakeDetected = t.get() + 0.5
                        println("detected = ${intakeDetected}")
                    }
                    if (t.get() > intakeDetected) {
                        intakeMotor.setPercentOutput(-INTAKE_HOLD)  //1.0
                        println("t_get = ${t.get()}")
                    }
                } else {
                //    println("Min Timer not Reached → ${t.get()}")
                }
            } else if (OI.operatorController.b){
                intakeMotor.setPercentOutput(INTAKE_POWER)  //1.0
            } else {
                intakeMotor.setPercentOutput(0.5 * OI.driverController.leftTrigger - 0.5 * OI.driverController.rightTrigger)
                isTimerStarted = false
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


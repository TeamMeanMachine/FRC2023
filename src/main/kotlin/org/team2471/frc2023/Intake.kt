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
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.deadband
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.Timer
import kotlin.math.absoluteValue
import kotlin.math.sin


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
    val pFeedEntry = table.getEntry("P Feed Forward")
    val pErrorEntry = table.getEntry("Pivot Error")
    val intakePowerEntry = table.getEntry("Intake Power")
    val wristPose = table.getEntry("Wrist Pose")
    val pivotPose = table.getEntry("Pivot Pose")

    val wristAngle: Angle
        get() = wristMotor.position.degrees
    var wristOffset = 0.0.degrees
    var wristSetpoint: Angle = wristAngle
        get() = wristSetpointEntry.getDouble(0.0).degrees
        set(value) {
            field = value.asDegrees.coerceIn(wristMin.asDegrees, wristMax.asDegrees).degrees
            wristSetpointEntry.setDouble(field.asDegrees)
//            println("field: ${round(field.asDegrees,1)}     offset: ${round(wristOffset.asDegrees, 1)}")
            wristMotor.setPositionSetpoint((field + wristOffset).asDegrees)
        }

    val pivotAnalogAngle: Angle
        get() = ((pivotSensor.value - 3595.0).degrees / 4100.0 * 360.0).wrap()
    var pivotOffset: Angle = 0.0.degrees

    val pivotAngle: Angle
        get() = pivotAnalogAngle
    var pivotSetpoint: Angle = pivotAngle
        set(value) {
            field = value.asDegrees.coerceIn(-180.0, 180.0).degrees
            setPivotPower()
            pivotSetpointEntry.setDouble(field.asDegrees)
        }
    val pivotPDController = PDController(0.100, 0.001) //0.1, 0.001  //0.03, 0.04)   //0.35, 0.03
    val pFeedForward: Double
        get() = pivotCurve.getValue((pivotAngle + if (wristAngle > 0.0.degrees) 180.0.degrees else 0.0.degrees).wrap().asDegrees) * sin(wristAngle.asRadians.absoluteValue)
    val pivotCurve = MotionCurve()

    var wristIsReset = false

    var wristMin = -90.0.degrees
        get() = -115.0.degrees + Arm.elbowAngle
    var wristMax = 90.0.degrees
        get() = 115.0.degrees + Arm.elbowAngle
    var linearFilter = LinearFilter.movingAverage(5)
    var holdingObject: Boolean = false
        get() = linearFilter.calculate(intakeMotor.current) > INTAKE_DETECT_CONE

    const val INTAKE_POWER = 1.0
    const val INTAKE_HOLD = 0.4
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

        wristMotor.setRawOffset(90.0)
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

            println("pFeed: ${pFeedEntry.getDouble(0.0)}")

            periodic {
                wristEntry.setDouble(wristAngle.asDegrees)
                pivotEntry.setDouble(pivotAngle.asDegrees)
                pivotSetpointEntry.setDouble(pivotSetpoint.asDegrees)
                pErrorEntry.setDouble(pivotSetpoint.asDegrees - pivotAngle.asDegrees)
                pFeedEntry.setDouble(pFeedForward)

                var pivot = OI.operatorController.rightThumbstickX.deadband(0.2)
                var wrist = OI.operatorController.rightThumbstickY.deadband(0.2)
                pivot *= 45.0 * 0.02  // degrees per second, time 1/50 second
                wrist *= 45.0 * 0.02
                pivotOffset -= pivot.degrees
                wristOffset += wrist.degrees
                wristSetpoint += 0.0.degrees

//                println("rot: $rotations    pInc:  ${round(pivotIncrementAngle.asDegrees, 2)}  pivotSet: ${pivotSetpoint.asDegrees}")

//
//                var pose3d = Pose3d(0.0,0.0,0.0, Rotation3d(0.0, 0.0, pivotAngle.asDegrees))
//                pivotPose.setValue(pose3d)
//
//                pose3d = Pose3d(0.0,0.0,0.0, Rotation3d(0.0, 0.0, wristAngle.asDegrees))
//                wristPose.setValue(pose3d)

                intakeCurrentEntry.setDouble(intakeMotor.current)
                setPivotPower()
//                println("min: ${round(wristMin.asDegrees, 1)}  max ${round(wristMax.asDegrees, 1)}")
//                println("power: ${round(power, 1)}      openLoop: ${round(openLoopPower, 1)}    error: ${round(pError, 1)}   setpoint: ${round(pivotSetpoint.asDegrees, 1)}    angle: ${round(pivotAngle.asDegrees, 1)}")

//                if (OI.operatorController.b) {
//                    holdingObject = false
//                    intakeMotor.setPercentOutput(INTAKE_POWER)
//                } else {
//                    intakeMotor.setPercentOutput(0.5 * OI.driverController.leftTrigger -
//                            0.5 * OI.driverController.rightTrigger +
//                            if (holdingObject) INTAKE_HOLD else 0.0
//                    )
//                    if (OI.driverController.rightTrigger > 0.1) {
//                        holdingObject = false
//                    }
//                }
            }
        }
    }

    override suspend fun default() {
        periodic {
            if (Arm.shoulderAngle.asDegrees.absoluteValue < 3.0 && Arm.elbowAngle.asDegrees.absoluteValue < 3.0) {
                if (wristAngle < -70.0.degrees) wristSetpoint = -90.0.degrees
                if (wristAngle > 70.0.degrees) wristSetpoint = 90.0.degrees
            }
        }
    }

    override fun preEnable() {
        pivotSetpoint = pivotAngle
        wristSetpoint = wristAngle
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

fun setPivotPower() {
    val pError = (Intake.pivotSetpoint + Intake.pivotOffset - Intake.pivotAngle).wrap().asDegrees
    val openLoopPower = Intake.pivotPDController.update(pError).coerceIn(-1.0, 1.0)
//    println("pivotError: ${round(pError, 1)}    openLoopPower: ${round(openLoopPower, 1)}")
    val power = openLoopPower + Intake.pFeedForward
    Intake.pivotMotor.setPercentOutput(power)
}
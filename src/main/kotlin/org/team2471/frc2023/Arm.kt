package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees

object Arm : Subsystem("Arm") {
    val shoulderMotor = MotorController(SparkMaxID(Sparks.SHOULDER_A), SparkMaxID(Sparks.SHOULDER_B))
    val elbowMotor = MotorController(SparkMaxID(Sparks.ELBOW))
//    val shoulderSensor = hall effect
//    val elbowSensor = hall effect

    private val table = NetworkTableInstance.getDefault().getTable(Intake.name)
    val shoulderEntry = table.getEntry("Shoulder Angle")
    val shoulderSetpointEntry = table.getEntry("Shoulder Setpoint")
    val elbowEntry = table.getEntry("Elbow Angle")
    val elbowSetpointEntry = table.getEntry("Elbow Setpoint")

    var shoulderAngle: Angle
        get() = shoulderMotor.position.degrees + shoulderOffset
        set(value) { shoulderMotor.position = value.asDegrees }
    var shoulderOffset = 0.0.degrees
    var shoulderSetpoint: Angle = shoulderAngle
        get() = shoulderSetpointEntry.getDouble(0.0).degrees
        set(value) {
            field = value.asDegrees.coerceIn(SHOULDER_BOTTOM, SHOULDER_TOP).degrees
            shoulderSetpointEntry.setDouble(field.asDegrees)
        }
    var elbowAngle: Angle
        get() = elbowMotor.position.degrees + elbowOffset
        set(value) { elbowMotor.position = value.asDegrees }
    var elbowOffset = 0.0.degrees
    var elbowSetpoint: Angle = elbowAngle
        get() = elbowSetpointEntry.getDouble(0.0).degrees
        set(value) {
            field = value.asDegrees.coerceIn(ELBOW_BOTTOM, ELBOW_TOP).degrees
            elbowSetpointEntry.setDouble(field.asDegrees)
        }

    var shoulderIsZeroed = false
    var elbowIsZeroed = false

    val SHOULDER_BOTTOM = -40.0
    val SHOULDER_TOP = 40.0
    val ELBOW_BOTTOM = -120.0
    val ELBOW_TOP = 120.0

    init {
        shoulderMotor.config(20) {
//            feedbackCoefficient
            brakeMode()
            //inverted? followersInverted?
//            pid{}
            currentLimit(50, 60, 1)
        }
        elbowMotor.config(20) {
            //feedbackCoefficient
            brakeMode()
            //inverted?
            //pid{}
            currentLimit(30, 40, 1)
        }
        shoulderIsZeroed = false
        elbowIsZeroed = false

        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                shoulderEntry.setDouble(shoulderAngle.asDegrees)
                elbowEntry.setDouble(elbowAngle.asDegrees)

                shoulderMotor.setPositionSetpoint(shoulderSetpoint.asDegrees)
                elbowMotor.setPositionSetpoint(elbowSetpoint.asDegrees)

                //zeroing
//                if (!shoulderIsZeroed) println("Shoulder angle is not zeroed")
//                if (!elbowIsZeroed) println("Elbow angle is not zeroed")
//                if (hall effect sensor true) {
//                    shoulderAngle = 0.0.degrees
//                    shoulderIsReset = true
//                }
//                if (hall effect sensor true) {
//                    elbowAngle = 0.0.degrees
//                    elbowIsReset = true
//                }
            }
        }
    }
}
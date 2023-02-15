package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.radians
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.pow

object Arm : Subsystem("Arm") {
    val shoulderMotor = MotorController(SparkMaxID(Sparks.SHOULDER_A), SparkMaxID(Sparks.SHOULDER_B))
    val elbowMotor = MotorController(SparkMaxID(Sparks.ELBOW))
//    val shoulderSensor = hall effect
//    val elbowSensor = hall effect

    private val table = NetworkTableInstance.getDefault().getTable(Arm.name)
    val shoulderEntry = table.getEntry("Shoulder Angle")
    val shoulderSetpointEntry = table.getEntry("Shoulder Setpoint")
    val elbowEntry = table.getEntry("Elbow Angle")
    val elbowSetpointEntry = table.getEntry("Elbow Setpoint")
    val endEffectorPositionXEntry = table.getEntry("endEffectorPosition X")
    val endEffectorPositionYEntry = table.getEntry("endEffectorPosition Y")
    val shoulderIKEntry = table.getEntry("Shoulder IK Angle")
    val elbowIKEntry = table.getEntry("Elbow IK Angle")

    val shoulderAngle: Angle
        get() = -shoulderMotor.position.degrees + shoulderOffset
    var shoulderOffset = 0.0.degrees
    var shoulderSetpoint: Angle = shoulderAngle
        get() = shoulderSetpointEntry.getDouble(0.0).degrees
        set(value) {
            field = value.asDegrees.coerceIn(SHOULDER_BOTTOM, SHOULDER_TOP).degrees
            shoulderSetpointEntry.setDouble(field.asDegrees)
        }
    val sFeedForward: Double
        get() = shoulderCurve.getValue(shoulderAngle.asDegrees)
    val shoulderCurve = MotionCurve()
    val elbowAngle: Angle
        get() = elbowMotor.position.degrees + elbowOffset
    var elbowOffset = 0.0.degrees
    var elbowSetpoint: Angle = elbowAngle
        get() = elbowSetpointEntry.getDouble(0.0).degrees
        set(value) {
            field = value.asDegrees.coerceIn(ELBOW_BOTTOM, ELBOW_TOP).degrees
            elbowSetpointEntry.setDouble(field.asDegrees)
        }
    val eFeedForward: Double
        get() = elbowCurve.getValue(elbowAngle.asDegrees)
    val elbowCurve = MotionCurve()

    var shoulderIsZeroed = false
    var elbowIsZeroed = false

    val SHOULDER_BOTTOM = -30.0
    val SHOULDER_TOP = 30.0
    val ELBOW_BOTTOM = -120.0
    val ELBOW_TOP = 120.0

    const val shoulderLength = 37.0
    const val elbowLength = 28.0

    /** Converts joint angles to the end effector position.  */
    fun forwardKinematics(inShoulder: Angle, inElbow: Angle) : Vector2 {
        val shoulder = inShoulder + 90.degrees
        val elbow = inElbow - 90.degrees
        return Vector2(
            shoulderLength * shoulder.cos() + elbowLength * elbow.cos(),
            shoulderLength * shoulder.sin() + elbowLength * elbow.sin())
    }

    /** Converts the end effector position to joint angles. */
    fun inverseKinematics(endPosition: Vector2) : Pair<Angle, Angle>{
        var relativePosition = endPosition
        // Flip when X is negative
        val isFlipped = relativePosition.x < 0.0
        if (isFlipped) {
            relativePosition.x = -relativePosition.x
        }

        // Calculate angles
        var elbow = -Math.acos((relativePosition.x.pow(2.0) + relativePosition.y.pow(2.0)
                - shoulderLength.pow(2.0) - elbowLength.pow(2.0)) / (2.0 * shoulderLength * elbowLength))

        if (elbow.isNaN()) {
            return Pair(0.radians, 0.radians)
        }

        var shoulder = Math.atan(relativePosition.y / relativePosition.x) -
                Math.atan((elbowLength * elbowAngle.sin()) / (shoulderLength + elbowLength * elbowAngle.cos()))

        // Invert shoulder angle if invalid
        val testPosition = forwardKinematics(shoulder.radians, elbow.radians)
        if ((testPosition-relativePosition).length > 1e-3) {
            shoulder += Math.PI
        }

        // Flip angles
        if (isFlipped) {
            shoulder = Math.PI - shoulder
            elbow = -elbow
        }

        // Wrap angles to correct ranges
        return Pair((shoulder.radians + 90.0.degrees).wrap(), (elbow.radians + shoulder.radians - 90.degrees).wrap())
    }

    const val REACH_LIMIT = 30.0
    const val HEIGHT_LIMIT = 25.0
    const val FLOOR_HEIGHT = -5.0
    const val ROBOT_COVER_HEIGHT = 2.0
    const val ROBOT_HALF_WIDTH = 36.0 / 2.0

    var endEffectorPosition : Vector2
    get() {
        return forwardKinematics(shoulderAngle, elbowAngle)
    }
    set(position) {
        var clampedPosition = position
        clampedPosition.x = clampedPosition.x.coerceIn(-REACH_LIMIT, REACH_LIMIT)
        clampedPosition.y = clampedPosition.y.coerceIn(FLOOR_HEIGHT, HEIGHT_LIMIT)
        if (clampedPosition.x.absoluteValue < ROBOT_HALF_WIDTH) {
            clampedPosition.y = max(clampedPosition.y, ROBOT_COVER_HEIGHT)
        }
        val (shoulder, elbow) = inverseKinematics(clampedPosition)
        shoulderSetpoint = shoulder
        elbowSetpoint = elbow
    }

    init {
        shoulderMotor.restoreFactoryDefaults()
        shoulderMotor.config(20) {
            feedbackCoefficient = 360.0 / 42.0 / 184.0  // ticks / degrees / gear ratio
            brakeMode()
            inverted(true)    //a is inverted
            followersInverted(false) //--spark max WAS inverted in rev hardware client -> advanced, check if weird
            pid {
                p(0.0000017)
                d(0.000001)
            }
            currentLimit(0, 60, 0)
            burnSettings()
        }
        elbowMotor.config(20) {
            feedbackCoefficient = 360.0 / 42.0 / 75.0
            brakeMode()
            pid{
                p(0.0000055)
                d(0.000004)
            }
            currentLimit(0, 60, 0)
            burnSettings()
        }
        shoulderIsZeroed = false
        elbowIsZeroed = false

        GlobalScope.launch(MeanlibDispatcher) {

            shoulderCurve.storeValue(-65.0, 0.13)
            shoulderCurve.storeValue(-30.0, 0.09)
            shoulderCurve.storeValue(-5.0, 0.05)
            shoulderCurve.storeValue(5.0, -0.05)
            shoulderCurve.storeValue(30.0, -0.09)
            shoulderCurve.storeValue(65.0, -0.13)

            elbowCurve.storeValue(-90.0, -0.14)
            elbowCurve.storeValue(-40.0, -0.09)
            elbowCurve.storeValue(0.0, 0.0)
            elbowCurve.storeValue(40.0, 0.15)
            elbowCurve.storeValue(90.0, 0.3)

            shoulderSetpointEntry.setDouble(shoulderAngle.asDegrees)
            elbowSetpointEntry.setDouble(elbowAngle.asDegrees)

            periodic {
                shoulderEntry.setDouble(shoulderAngle.asDegrees)
                elbowEntry.setDouble(elbowAngle.asDegrees)
                endEffectorPositionXEntry.setDouble(endEffectorPosition.x)
                endEffectorPositionYEntry.setDouble(endEffectorPosition.y)
                val (ikShoulder, ikElbow) = inverseKinematics(endEffectorPosition)
                shoulderIKEntry.setDouble(ikShoulder.asDegrees)
                elbowIKEntry.setDouble(ikElbow.asDegrees)

                if (Intake.pivotAngle > 80.0.degrees && Intake.pivotAngle < 100.0.degrees) { // && (Intake.wristAngle < -80.0.degrees || Intake.wristAngle > 80.0.degrees)) { //pivotAngle will need to be negated when pivotCurve inverted properly
                    elbowMotor.setPositionSetpoint(elbowSetpoint.asDegrees, eFeedForward)
                    shoulderMotor.setPositionSetpoint(shoulderSetpoint.asDegrees, sFeedForward)
                    println("Allowed to move shoulder and elbow")
                } else {
                    println("Arm and elbow not moving")
                }
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
    fun shoulderCoastMode() {
        shoulderMotor.coastMode()
    }
    fun shoulderBrakeMode() {
        shoulderMotor.brakeMode()
    }
}
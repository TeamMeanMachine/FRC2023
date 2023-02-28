package org.team2471.frc2023

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DigitalInput
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.deadband
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.radians
import kotlin.math.*

object Arm : Subsystem("Arm") {
    val shoulderMotor = MotorController(SparkMaxID(Sparks.SHOULDER_A))
    val shoulderFollowerMotor = MotorController(SparkMaxID(Sparks.SHOULDER_B))
    val elbowMotor = MotorController(SparkMaxID(Sparks.ELBOW))
    val shoulderEncoder = AnalogInput(AnalogSensors.SHOULDER)
    val elbowEncoder = AnalogInput(AnalogSensors.ELBOW)
    val shoulderSensor = DigitalInput(DigitalSensors.SHOULDER_SWTICH)
    val elbowSensor = DigitalInput(DigitalSensors.ELBOW_SWITCH)

    private val table = NetworkTableInstance.getDefault().getTable(Arm.name)
    val shoulderEntry = table.getEntry("Shoulder Angle")
    val shoulderSetpointEntry = table.getEntry("Shoulder Setpoint")
    val elbowEntry = table.getEntry("Elbow Angle")
    val elbowSetpointEntry = table.getEntry("Elbow Setpoint")
    val wristPositionXEntry = table.getEntry("Wrist Position X")
    val wristPositionYEntry = table.getEntry("Wrist Position Y")
    val shoulderIKEntry = table.getEntry("Shoulder IK Angle")
    val elbowIKEntry = table.getEntry("Elbow IK Angle")
    val shoulderFollowerEntry = table.getEntry("Shoulder Follower Angle")
    val shoulderZeroedForwardEntry = table.getEntry("Shoulder Zero Forward")
    val shoulderZeroedBackwardEntry = table.getEntry("Shoulder Zero Backward")
    val shoulderIsZeroedEntry = table.getEntry("Shoulder Is Zeroed")
    var shoulderGetZeroCount = 0
    var shoulderZeroForward = false
    var shoulderZeroBackward = false
//    val shoulderPose = table.getEntry("Shoulder Pose")
//    val elbowPose = table.getEntry("Elbow Pose")

    val shoulderAngle: Angle
        get() = -(shoulderEncoder.value - 3020.0).degrees / (if (shoulderEncoder.value - 3020.0 < 0.0) 12.4 else 10.5)
    var shoulderOffset = 0.0.degrees
    var shoulderSetpoint: Angle = shoulderAngle
        set(value) {
            var temp = value
//            if (value > 0.0.degrees) {
//                temp -= shoulderZeroedForwardEntry.getDouble(0.0).degrees
//            } else {
//                temp -= shoulderZeroedBackwardEntry.getDouble(0.0).degrees
//            }

            field = temp.asDegrees.coerceIn(SHOULDER_BOTTOM, SHOULDER_TOP).degrees
            shoulderSetpointEntry.setDouble(field.asDegrees)
            shoulderMotor.setPositionSetpoint(field.asDegrees, sFeedForward)
            shoulderFollowerMotor.setPositionSetpoint(field.asDegrees, sFeedForward)
        }
    val sFeedForward: Double
        get() = shoulderCurve.getValue(shoulderAngle.asDegrees)
    val shoulderCurve = MotionCurve()
    var tempShoulder = shoulderAngle
    var prevShoulder = shoulderAngle
    val shoulderFollowerAngle: Angle
        get() = shoulderFollowerMotor.position.degrees + shoulderOffset
    var shoulderNegSlop = 0.0.degrees
    var shoulderPosSlop = 0.0.degrees
    var waitForNegative = false
    var waitForPositive = false
    val elbowAngle: Angle
        get() = elbowMotor.position.degrees + elbowOffset
    var elbowOffset = 0.0.degrees
    var elbowSetpoint: Angle = elbowAngle
        set(value) {
            field = value.asDegrees.coerceIn(ELBOW_BOTTOM, ELBOW_TOP).degrees
            elbowSetpointEntry.setDouble(field.asDegrees)
            elbowMotor.setPositionSetpoint(field.asDegrees, eFeedForward)
        }
    val shoulderIsZeroed
        get() = shoulderZeroForward && shoulderZeroBackward
    val eFeedForward: Double
        get() = elbowCurve.getValue(elbowAngle.asDegrees)
    val elbowCurve = MotionCurve()
    var tempElbow = elbowAngle
    var prevElbow = elbowAngle

    var elbowIsZeroed = false

    val SHOULDER_BOTTOM = -40.0
    val SHOULDER_TOP = 40.0
    val SHOULDER_MAGNET = 1.0
    val ELBOW_BOTTOM = -120.0
    val ELBOW_TOP = 120.0

    const val shoulderLength = 37.0
    const val elbowLength = 28.0

    const val shoulderBindAngle = 90.0
    const val elbowBindAngle = -90.0

    /** Converts joint angles to the end effector position.  */
    fun forwardKinematics(inShoulder: Angle, inElbow: Angle): Vector2 {
        val shoulder = inShoulder + shoulderBindAngle.degrees
        val elbow = inElbow + elbowBindAngle.degrees
        return Vector2(
            shoulderLength * shoulder.cos() + elbowLength * elbow.cos(),
            shoulderLength * shoulder.sin() + elbowLength * elbow.sin()
        )
    }

    /** Converts the end effector position to joint angles. */
    fun inverseKinematics(endPosition: Vector2): Pair<Angle, Angle> {
        val length0 = shoulderLength
        val length1 = elbowLength
        val length2 = endPosition.length

        // Inner angle alpha
        val cosInnerAlpha = (length2 * length2 + length0 * length0 - length1 * length1) / (2 * length2 * length0)
        val innerAlpha = acos(cosInnerAlpha)

        // Inner angle beta
        val cosInnerBeta = (length1 * length1 + length0 * length0 - length2 * length2) / (2 * length1 * length0)
        val innerBeta = acos(cosInnerBeta)

        // overall arm direction
        val effectorAngle = atan2(endPosition.y, endPosition.x)

        var jointAngleA1 = effectorAngle - innerAlpha
        val jointAngleA2 = effectorAngle + innerAlpha
        var jointAngleB = Math.PI - innerBeta

        if (sin(jointAngleA2) > sin(jointAngleA1)) {  // prefer the higher elbow position
            jointAngleA1 = jointAngleA2
            jointAngleB = -jointAngleB
        }

        // Adjust angles to starting pose, change elbow to absolute angle, and wrap angles to a tight range.
        return Pair(
            (jointAngleA1.radians - shoulderBindAngle.degrees).wrap(),
            (jointAngleA1.radians + jointAngleB.radians - elbowBindAngle.degrees).wrap()
        )
    }

    const val REACH_LIMIT = 40.0
    const val HEIGHT_LIMIT = 50.0
    const val FLOOR_HEIGHT = -5.0
    const val ROBOT_COVER_HEIGHT = 9.0
    const val ROBOT_HALF_WIDTH = 36.0 / 2.0

    var wristPosition = forwardKinematics(shoulderAngle, elbowAngle)
        set(position) {
            var clampedPosition = position
            clampedPosition.x = clampedPosition.x.coerceIn(-REACH_LIMIT, REACH_LIMIT)
            clampedPosition.y = clampedPosition.y.coerceIn(FLOOR_HEIGHT, HEIGHT_LIMIT)
            if (clampedPosition.x.absoluteValue < ROBOT_HALF_WIDTH) {  // over top of robot
                clampedPosition.y = max(clampedPosition.y, ROBOT_COVER_HEIGHT)
                clampedPosition.y = min(
                    clampedPosition.y,
                    (HEIGHT_LIMIT / ROBOT_HALF_WIDTH) * clampedPosition.x.absoluteValue + ROBOT_COVER_HEIGHT
                )
            }
            field = clampedPosition
            val (shoulder, elbow) = inverseKinematics(clampedPosition)
           // println("clampPosition=${clampedPosition} shoulder=${shoulder}  elbow=${elbow}")
            if (!shoulder.asDegrees.isNaN()) {
                shoulderSetpoint = shoulder
            }
            if (!elbow.asDegrees.isNaN()) {
                elbowSetpoint = elbow
            }
        }

    init {
        println("Arm init")
        shoulderMotor.restoreFactoryDefaults()
        shoulderFollowerMotor.restoreFactoryDefaults()
        elbowMotor.restoreFactoryDefaults()
        shoulderMotor.config(20) {
            feedbackCoefficient = 360.0 / 42.0 / 184.0  // ticks / degrees / gear ratio
            coastMode()
            inverted(false)
            pid {
                p(0.0000017)
                d(0.000001)
            }
            currentLimit(0, 60, 0)
            burnSettings()
        }
        shoulderFollowerMotor.config(20) {
            feedbackCoefficient = 360.0 / 42.0 / 184.0  // ticks / degrees / gear ratio
            coastMode()
            inverted(false)
            pid {
                p(0.0000017)
                d(0.000001)
            }
            currentLimit(0, 60, 0)
            burnSettings()
        }
        elbowMotor.config(20) {
            feedbackCoefficient = 360.0 / 42.0 / 75.0
            coastMode()
            pid {
                p(0.0000055)
                d(0.000004)
            }
            currentLimit(0, 60, 0)
            burnSettings()
        }
        elbowIsZeroed = false
        shoulderSetpoint = shoulderMotor.position.degrees
        elbowSetpoint = elbowMotor.position.degrees

        GlobalScope.launch(MeanlibDispatcher) {

            shoulderCurve.storeValue(-65.0, 0.13)
            shoulderCurve.storeValue(-30.0, 0.09)
            shoulderCurve.storeValue(-5.0, 0.05)
            shoulderCurve.storeValue(5.0, -0.05)
            shoulderCurve.storeValue(30.0, -0.09)
            shoulderCurve.storeValue(65.0, -0.13)

            elbowCurve.storeValue(-90.0, -0.18)
            elbowCurve.storeValue(-40.0, -0.13)
            elbowCurve.storeValue(0.0, 0.0)
            elbowCurve.storeValue(40.0, 0.18)
            elbowCurve.storeValue(90.0, 0.19)

            var shoulderDirection = LinearFilter.movingAverage(3)
            var previousShoulderSensor = false

            shoulderSetpointEntry.setDouble(shoulderAngle.asDegrees)
            elbowSetpointEntry.setDouble(elbowAngle.asDegrees)

            println("shoulderFollower: ${shoulderFollowerEntry.getDouble(0.0)}")
            periodic {
                shoulderEntry.setDouble(shoulderAngle.asDegrees)
                elbowEntry.setDouble(elbowAngle.asDegrees)

//                var pose3d = Pose3d(0.0,0.0,0.0, Rotation3d(0.0, 0.0, shoulderAngle.asDegrees))
//                shoulderPose.setValue(pose3d)
//
//                pose3d = Pose3d(0.0,0.0,0.0, Rotation3d(0.0, 0.0, elbowAngle.asDegrees))
//                elbowPose.setValue(pose3d)

                wristPositionXEntry.setDouble(wristPosition.x)
                wristPositionYEntry.setDouble(wristPosition.y)
                val (ikShoulder, ikElbow) = inverseKinematics(wristPosition)
                shoulderIKEntry.setDouble(ikShoulder.asDegrees)
                elbowIKEntry.setDouble(ikElbow.asDegrees)
                shoulderFollowerEntry.setDouble(shoulderFollowerAngle.asDegrees)
                shoulderIsZeroedEntry.setBoolean(shoulderIsZeroed)

                //zeroing
                if ((shoulderMotor.position - shoulderAngle.asDegrees).absoluteValue > 2.0) {
                    println("Resetting shoulder to $shoulderAngle")
                    shoulderMotor.setRawOffset(shoulderAngle.asDegrees)
                    shoulderFollowerMotor.setRawOffset(shoulderAngle.asDegrees)
                }
//                if (!shoulderIsZeroed) {
//                    tempShoulder = shoulderAngle
//                    val shoulderForward = shoulderDirection.calculate(tempShoulder.asDegrees) > 0.0
//                    if (!shoulderSensor.get()) {
//                        if (shoulderForward && !previousShoulderSensor) {
//                            shoulderMotor.setRawOffset(SHOULDER_MAGNET)
//                            if (waitForPositive) {
//                                shoulderNegSlop = (shoulderAngle.asDegrees - SHOULDER_MAGNET).absoluteValue.degrees
////                                shoulderPosIsZeroed = true
//                            } else {
//                                waitForPositive = true
//                            }
//                        }
//                        previousShoulderSensor = true
//                    } else {
//                        if (!shoulderForward && previousShoulderSensor) {
//                            shoulderMotor.setRawOffset(SHOULDER_MAGNET)
//                            if (waitForNegative) {
//                                shoulderPosSlop = (shoulderAngle.asDegrees - SHOULDER_MAGNET).absoluteValue.degrees
////                                shoulderNegIsZeroed = true
//                            } else {
//                                waitForNegative = true
//                            }
//                        }
//                        previousShoulderSensor = false
//                    }
//                    if (waitForPositive && shoulderAngle > 20.0.degrees) waitForPositive = false
//                    if (waitForNegative && shoulderAngle < -20.0.degrees) waitForNegative = false
//                }

                tempElbow = elbowAngle
                prevShoulder = tempShoulder
                prevElbow = tempElbow
            }
        }
    }

    fun resetShoulderZero() {
        shoulderZeroForward = false
        shoulderZeroBackward = false
        shoulderGetZeroCount = 0
    }

    override fun preEnable() {
        shoulderMotor.setPercentOutput(0.0)
        shoulderFollowerMotor.setPercentOutput(0.0)
        elbowMotor.setPercentOutput(0.0)
        Intake.wristMotor.setPercentOutput(0.0)
        wristPosition = forwardKinematics(shoulderMotor.position.degrees, elbowAngle)
        println("wristPos: $wristPosition")
    }

    override suspend fun default() {
        periodic {
            var move = Vector2(
                OI.operatorController.leftThumbstickX.deadband(0.2),
                -OI.operatorController.leftThumbstickY.deadband(0.2)
            )
            move *= 12.0 * 0.02   // d = r * t  where rate is inches per second and time is 1/50 second
            wristPosition += move
        }
    }

    fun shoulderCoastMode() {
        shoulderMotor.coastMode()
    }

    fun shoulderBrakeMode() {
        shoulderMotor.brakeMode()
    }

    fun testIK() {
        var matches = 0
        var mismatches = 0
        var mirrors = 0
        for (shoulderDegrees in -80..80 step (5)) {
            val shoulderAngle = shoulderDegrees.toDouble().degrees
            for (elbowDegrees in -90..90 step (5)) {
                if (elbowDegrees == -shoulderDegrees) {
                    continue
                }
                val elbowAngle = elbowDegrees.toDouble().degrees
                val position = forwardKinematics(shoulderAngle, elbowAngle)
                val (shoulderIKAngle, elbowIKAngle) = inverseKinematics(position)
                if ((shoulderDegrees.toDouble() - shoulderIKAngle.asDegrees).absoluteValue > 0.5 ||
                    (elbowDegrees.toDouble() - elbowIKAngle.asDegrees).absoluteValue > 0.5
                ) {
                    val positionIK = forwardKinematics(shoulderIKAngle, elbowIKAngle)
                    if ((positionIK.x - position.x).absoluteValue > 0.5 ||
                        (positionIK.y - position.y).absoluteValue > 0.5
                    ) {
                        println("sh:$shoulderDegrees=${shoulderIKAngle.asDegrees}  el:$elbowDegrees=${elbowIKAngle.asDegrees}")
                        mismatches++
                    } else {
                        mirrors++
                    }
                } else {
                    matches++
                }
            }
        }
        assert(mismatches == 0)
        println("matches=$matches mismatches=$mismatches mirrors=$mirrors\n")

//    val shoulderDegrees = 30.0
//    val elbowDegrees = 40.0
//    val shoulderRadians = Math.toRadians(shoulderDegrees)
//    val elbowRadians = Math.toRadians(elbowDegrees)
//    val position = forwardKinematics(shoulderRadians, elbowRadians)
//    println("x=${position.first}  y=${position.second}")
//    val (shoulderIKRadians, elbowIKRadians) = inverseKinematics(position)
//    val shoulderIKDegrees = Math.toDegrees(shoulderIKRadians)
//    val elbowIKDegrees = Math.toDegrees(elbowIKRadians)
//    println("sh:$shoulderDegrees=$shoulderIKDegrees  el:$elbowDegrees=$elbowIKDegrees")
    }
}
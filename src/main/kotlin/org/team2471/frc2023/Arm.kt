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
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.*
import org.team2471.frc2023.Robot.isCompBot
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
    val xFKEntry = table.getEntry("X FK Wrist Position")
    val yFKENtry = table.getEntry("Y FK Wrist Position")
    val shoulderZeroedForwardEntry = table.getEntry("Shoulder Zero Forward")
    val shoulderZeroedBackwardEntry = table.getEntry("Shoulder Zero Backward")
    val shoulderIsZeroedEntry = table.getEntry("Shoulder Is Zeroed")
    val shoulderTickEntry = table.getEntry("Shoulder Ticks")
    val elbowTickEntry = table.getEntry("Elbow Ticks")
    val autoArmEntry = table.getEntry("Auto Arm")
    var shoulderGetZeroCount = 0
    var shoulderZeroForward = false
    var shoulderZeroBackward = false
    val elbowAngleCheck = table.getEntry("Elbow Angle Check")
    val shoulderAngleCheck = table.getEntry("Shoulder Angle Check")
    val deltaValueEntry = table.getEntry("Delta Value")
    val elbowMotorCurrentEntry = table.getEntry("Elbow Motor Current")
    val nodeAngleEntry = table.getEntry("Robot Angle")
    val wristFrontOffsetEntry = table.getEntry("Front Wrist Offset")
    val wristBackOffsetEntry = table.getEntry("Back Wrist Offset")


    val driverInControlEntry = table.getEntry("Driver in Control")
    val operatorInControlEntry = table.getEntry("Operator in Control")

    val shoulderTicks: Int
        get() = shoulderEncoder.value
    val shoulderAngle: Angle
        get() =
            if (isCompBot) {
                (-shoulderEncoder.value.degrees + 3120.degrees) / 11.2 //if (-shoulderEncoder.value + 1183 < 0.0) 12.4 else 9.8)
            } else {
                (-shoulderEncoder.value.degrees + 1133.degrees) / 11.2
            }

    val shoulderAnalogAngle: Angle
        get() = shoulderEncoder.value.degrees
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
    val elbowTicks: Int
        get() = elbowEncoder.value
    val elbowAngle: Angle
        get() = if (isCompBot) {
            (-elbowEncoder.value.degrees + 1469.degrees) * 90.0 / 1054.0
        } else {
            (-elbowEncoder.value.degrees + 1912.degrees) * 90.0 / 1054.0
        }

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
    val elbowFilter = LinearFilter.movingAverage(5)
    val elbowDirection
        get() = elbowFilter.calculate(tempElbow.asDegrees - prevElbow.asDegrees)
    var isFlipping: Boolean = false

    val distanceToTarget: Length
        get() = ((FieldManager.getSelectedNode()?.position ?: PoseEstimator.currentPose) - PoseEstimator.currentPose).length.feet
    val targetOffset: Length
        get() = distanceToTarget - wristPosition.x.inches

    val SHOULDER_BOTTOM = -50.0
    val SHOULDER_TOP = 50.0
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

    const val REACH_LIMIT = 47.0
    const val HEIGHT_LIMIT = 50.0
    const val FLOOR_HEIGHT = -5.0
    const val ROBOT_COVER_HEIGHT = 9.0
    const val ROBOT_HALF_WIDTH = 36.0 / 2.0

    var wristPosition = forwardKinematics(shoulderAngle, elbowAngle)
        set (position) {
            field = position
            var clampedPosition = position + if (position.x.absoluteValue < 10.0) Vector2(0.0, 0.0) else wristPosOffset

            //clamp
            clampedPosition.x = clampedPosition.x.coerceIn(-REACH_LIMIT, REACH_LIMIT)
            clampedPosition.y = clampedPosition.y.coerceIn(FLOOR_HEIGHT, HEIGHT_LIMIT)
            if (clampedPosition.x.absoluteValue < ROBOT_HALF_WIDTH) {  // over top of robot
                clampedPosition.y = max(clampedPosition.y, ROBOT_COVER_HEIGHT)
                clampedPosition.y = min(
                    clampedPosition.y,
                    (HEIGHT_LIMIT / ROBOT_HALF_WIDTH) * clampedPosition.x.absoluteValue + ROBOT_COVER_HEIGHT
                )
            }

            //set shoulder/angle setpoints
            val (shoulder, elbow) = inverseKinematics(clampedPosition)
//            println("shoulder=${round(shoulder.asDegrees, 1)}  elbow=${round(elbow.asDegrees, 1)} field=${field} ")
            if (!shoulder.asDegrees.isNaN()) {
                shoulderSetpoint = shoulder
            }
            if (!elbow.asDegrees.isNaN()) {
                elbowSetpoint = elbow
            }
        }

//    val actualWristPosition
//        get() = forwardKinematics(shoulderAngle, elbowAngle)
    var wristPosOffset
        get() = if (wristPosition.x > 7.0) wristFrontOffset else if (wristPosition.x < -7.0) wristBackOffset else wristCenterOffset
        set(value) {
            if (wristPosition.x > 7.0) wristFrontOffset = value else if (wristPosition.x < -7.0) wristBackOffset = value else wristCenterOffset = value
        }
    var wristFrontOffset = Vector2(0.0, 0.0)
    var wristBackOffset = Vector2(0.0, 0.0)
    var wristCenterOffset = Vector2(0.0, 0.0)

    val autoArmEnabled: Boolean
        get() = autoArmEntry.getBoolean(true)

    init {
        println("Arm init")
        autoArmEntry.setBoolean(false)
        shoulderMotor.restoreFactoryDefaults()
        shoulderFollowerMotor.restoreFactoryDefaults()
        elbowMotor.restoreFactoryDefaults()
        shoulderMotor.config(20) {
            feedbackCoefficient = 360.0 / 42.0 / 165.0 //184.0  // ticks / degrees / gear ratio
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
            feedbackCoefficient = 360.0 / 42.0 / 165.0  // ticks / degrees / gear ratio
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

            shoulderCurve.storeValue(-70.0, 0.16)
            shoulderCurve.storeValue(-65.0, 0.13)
            shoulderCurve.storeValue(-30.0, 0.09)
            shoulderCurve.storeValue(-5.0, 0.05)
            shoulderCurve.storeValue(5.0, -0.05)
            shoulderCurve.storeValue(30.0, -0.09)
            shoulderCurve.storeValue(65.0, -0.13)
            shoulderCurve.storeValue(70.0, -0.16)

            elbowCurve.storeValue(-110.0, -0.21)
            elbowCurve.storeValue(-90.0, -0.18)
            elbowCurve.storeValue(-40.0, -0.13)
            elbowCurve.storeValue(0.0, 0.0)
            elbowCurve.storeValue(40.0, 0.18)
            elbowCurve.storeValue(90.0, 0.19)
            elbowCurve.storeValue(110.0, 0.23)

            var shoulderDirection = LinearFilter.movingAverage(3)
            var previousShoulderSensor = false

            shoulderSetpointEntry.setDouble(shoulderAngle.asDegrees)
            elbowSetpointEntry.setDouble(elbowAngle.asDegrees)

            println("shoulderFollower: ${shoulderFollowerEntry.getDouble(0.0)}")
            periodic {

                elbowAngleCheck.setBoolean(elbowAngle.asDegrees.absoluteValue < 10)
                shoulderAngleCheck.setBoolean(shoulderAngle.asDegrees.absoluteValue < 10)
                wristFrontOffsetEntry.setDoubleArray(arrayOf(wristFrontOffset.x, wristFrontOffset.y))
                wristBackOffsetEntry.setDoubleArray(arrayOf(wristBackOffset.x, wristBackOffset.y))

                driverInControlEntry.setBoolean(OI.controlledBy == OI.PERSONINCONTROL.DRIVER)
                operatorInControlEntry.setBoolean(OI.controlledBy == OI.PERSONINCONTROL.OPERATOR)

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
                shoulderFollowerEntry.setDouble(shoulderMotor.position)
                shoulderIsZeroedEntry.setBoolean(shoulderIsZeroed)
                val (fkX, fkY) = forwardKinematics(shoulderAngle, elbowAngle)
                xFKEntry.setDouble(fkX)
                yFKENtry.setDouble(fkY)
                shoulderTickEntry.setDouble(shoulderTicks.toDouble())
                elbowTickEntry.setDouble(elbowTicks.toDouble())

                var move = Vector2(
                    OI.operatorController.leftThumbstickX.deadband(0.2),
                    -OI.operatorController.leftThumbstickY.deadband(0.2)
                )
                move *= 10.0 * 0.02   // d = r * t  where rate is inches per second and time is 1/50 second
                wristPosOffset += move
//                println("wristPosOffset: $wristPosOffset")
                wristPosition += Vector2(0.0, 0.0)

                //zeroing
                if ((shoulderMotor.position - shoulderAngle.asDegrees).absoluteValue > 2.0) {
                    if ((shoulderMotor.position - shoulderAngle.asDegrees).absoluteValue > 15.0) println("Resetting shoulder from ${round(shoulderMotor.position, 1)} to ${round(shoulderAngle.asDegrees, 1)}")
                    shoulderMotor.setRawOffset(shoulderAngle.asDegrees)
                    shoulderFollowerMotor.setRawOffset(shoulderAngle.asDegrees)
                }
                if ((elbowMotor.position - elbowAngle.asDegrees).absoluteValue > 4.0) { //testing time
                    if ((elbowMotor.position - elbowAngle.asDegrees).absoluteValue > 15.0) println("Resetting elbow from ${round(elbowMotor.position, 1)} to ${round(elbowAngle.asDegrees, 1)}")
                    elbowMotor.setRawOffset(elbowAngle.asDegrees)
                }

                tempElbow = elbowAngle
//                if (!elbowSensor.get()) {
//                    if (!seesElbowSwitch){
//                        println("zeroing from elbow ${round(elbowAngle.asDegrees, 1)}")
//                        elbowOffset -= if (elbowDirection <= 0.0) tempElbow - 4.0.degrees else tempElbow + 1.0.degrees
//                        println("to elbow ${round(elbowAngle.asDegrees, 1)}")
//                    }
//                    seesElbowSwitch = true
//                } else {
//                    if (seesElbowSwitch) {
//                        println("zeroing from elbow ${round(elbowAngle.asDegrees, 1)}")
//                        elbowOffset -= if (elbowDirection >= 0.0) tempElbow - 4.0.degrees else tempElbow + 1.0.degrees
//                        println("to elbow ${round(elbowAngle.asDegrees, 1)}")
//                    }
//                    seesElbowSwitch = false
//                }
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
        OI.controlledBy = OI.PERSONINCONTROL.NONE
        shoulderFollowerMotor.setPercentOutput(0.0)
        elbowMotor.setPercentOutput(0.0)
        wristPosition = forwardKinematics(shoulderMotor.position.degrees, elbowAngle)
        println("wristPos: $wristPosition")
    }

    override suspend fun default() {
        periodic {
            wristPositionXEntry.setDouble(wristPosition.x)
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
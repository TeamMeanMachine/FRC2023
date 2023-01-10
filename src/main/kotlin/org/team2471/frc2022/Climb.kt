package org.team2471.frc2022

import com.ctre.phoenix.motorcontrol.StatusFrame
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.ejml.interfaces.linsol.LinearSolver
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.parallel
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.Angle.Companion.cos
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.radians
import javax.swing.Action
import kotlin.math.absoluteValue

object Climb : Subsystem("Climb") {
    val heightMotor = MotorController(FalconID(Falcons.CLIMB), FalconID(Falcons.CLIMB_TWO))
    val angleMotor = MotorController(FalconID(Falcons.CLIMB_ANGLE))
    val angleEncoder = DutyCycleEncoder(if (isCompBot) {DigitalSensors.CLIMB_ANGLE} else {7})
    private val table = NetworkTableInstance.getDefault().getTable(name)

    val heightEntry = table.getEntry("Height")
    val heightSetpointEntry = table.getEntry("Height Setpoint")
    val angleEntry = table.getEntry("Angle")
    val angleMotorEntry = table.getEntry("Angle Motor")
    val throughBoreEntry = table.getEntry("Climb Through Bore")
    val angleSetpointEntry = table.getEntry("Angle Setpoint")
    val robotRollEntry = table.getEntry("Roll")
    val heightMotorOutput = table.getEntry("Height Output")
    val angleMotorOutput = table.getEntry("Angle Output")

    var climbIsPrepped = false
    var climbStage = 0
    var climbMode = false
    var bungeeTakeOver = false
    val height: Double
        get() = heightMotor.position
    var heightSetpoint = height
        get() = heightSetpointEntry.getDouble(height)
        set(value) {
            field = value.coerceIn(HEIGHT_BOTTOM, HEIGHT_TOP)
            heightSetpointEntry.setDouble(field)
        }

    val tuningMode = false

    const val HOLDING_ANGLE = 1.0

    const val HEIGHT_TOP = 30.5
    const val HEIGHT_VERTICAL_TOP = 25.5
    const val HEIGHT_PARTIAL_PULL = 15.0
    const val HEIGHT_BOTTOM_DETACH = 8.0
    const val HEIGHT_BOTTOM = 0.0

    val ANGLE_TOP = if (isCompBot) 32.0 else 36.0 //comp: 33.5
    const val ANGLE_BOTTOM = -1.0 //-4.0

    val roll : Double
        get() = Drive.gyro.getRoll()
    val angleOffset = if (isCompBot) 0.0 else 28.0 //-45 to -12
    val angleEncoderModifier = if (isCompBot) 1.0 else -1.0
//    val angleAbsoluteRaw : Double
//        get() = angleEncoder.absolutePosition
//    val angleRelativeRaw : Double
//        get() = angleEncoder.get()
//    val angleRelative: Double
//        get() = ((((angleEncoder.get() * angleEncoderModifier) - 0.05) * 37.0 / 0.13) + angleOffset).degrees.wrap().asDegrees
    val angle: Double
//        get() = angleMotor.position
         get() = (((((angleEncoder.absolutePosition * angleEncoderModifier) - 0.0429) * 360.0) -2.0) * 29.1 / 42.2).degrees.wrap().asDegrees

    var angleSetpoint = 0.0
        get() = angleSetpointEntry.getDouble(0.0)
        set(value) {
            field = value.coerceIn(ANGLE_BOTTOM, ANGLE_TOP)
            angleSetpointEntry.setDouble(field)
        }
    val anglePDController = if (isCompBot) PDController(0.04, 0.002) else PDController(0.01, 0.002)//0.03, 0.0)    //0.006
    val angleFeedForward: Double
        /*if (climbIsPrepped || tuningMode) */
        get() {
            if (isCompBot) {
                val returnThis = angleFeedForwardCurve.getValue(angle) // linearMap(ANGLE_BOTTOM, ANGLE_TOP, 0.19, 0.06, angle) //outLo 0.15  outHi 0.06  //outLo 0.09   outHi 0.04   //(0.16 - 0.027) * ((27.0 - angle) / 32.5) + 0.027 //(0.17 - 0.04) * ((27.0 - angle) / 32.5) + 0.04    ((ff at min angle) - (ff at max)) * ((max angle + min angle) - angle) / (max angle)) + (ff at max angle)
////                println("feedForward: $returnThis      angle: $angle ")
                return returnThis
//                return 0.07
            } else {
                return 0.2
            }
        }/* else 0.0*/ //compbot 0.09                         //feedforward estimates: at -4.5 min angle -> 0.17         at 28.0 max angle -> 0.04
    val angleFeedForwardCurve = MotionCurve()

    var isAngleMotorControlled = false

    init {
        heightMotor.config {
            brakeMode()
            inverted(true)
            followersInverted(true)
            feedbackCoefficient = 3.14 / 2048.0 / 9.38 * 28.5 / 25.5 // * 30.0 / 25.5 //3.14 / 2048.0 / 9.38 * 30.0 / 26.0
            pid {
                p(0.00000002)
            }
        }
        angleMotor.config {
            coastMode()  //0.09
            inverted(true)
            feedbackCoefficient = (360.0 / 2048.0 / 87.1875 * 90.0 / 83.0 / 3.0) // * (if (isCompBot) 34.0 / 40.0 else 39.0 / 26.0))
            pid {
                p(0.00000001) //0.04)
                d(0.00000001)
//                p(0.000000012) //0.000000008) //2e-8) //1e-5)
//                d(1e-7)
            }
            setRawOffsetConfig(angle.degrees) //(-4.5).degrees)
            currentLimit(45, 50, 1)      //not tested yet but these values after looking at current graph 3/30
        }
        heightMotor.position = 1.0
        heightSetpointEntry.setDouble(height)
        angleSetpointEntry.setDouble(angle)
        setStatusFrames(true)
        GlobalScope.launch {
//            parallel ({
//                periodic {
//                    if (angleEncoder.isConnected && angle > ANGLE_BOTTOM && angle < ANGLE_TOP) {
//                        angleMotor.setRawOffset(-4.0.degrees) //angle.degrees) goodbye encoder
//                        angleSetpoint = angle
//                        println("setpoints angle $angle")
//                        this.stop()
//                    }
//                }
//            }, {
                periodic {
                    //println("absolute: ${round(angleAbsoluteRaw, 4)} relative:$angleRelativeRaw abs: $angleAbsolute rel: $angle diff = ${angleAbsolute - angle}")
                    heightEntry.setDouble(heightMotor.position)
                    angleEntry.setDouble(angle)
                    angleMotorEntry.setDouble(angleMotor.position)
                    val throughBoreAngle = (((((angleEncoder.absolutePosition * angleEncoderModifier) - 0.0429) * 360.0) -2.0) * 29.1 / 42.2).degrees.wrap().asDegrees
                    throughBoreEntry.setDouble(throughBoreAngle)
                    robotRollEntry.setDouble(roll)
                    heightMotorOutput.setDouble(heightMotor.output)
                    angleMotorOutput.setDouble(angleMotor.output)

                    angleFeedForwardCurve.setMarkBeginOrEndKeysToZeroSlope(false)

                    angleFeedForwardCurve.storeValue(-5.0, 0.2)  // 0.22 //0.2)
                    angleFeedForwardCurve.storeValue(5.0, 0.18) //0.18
                    angleFeedForwardCurve.storeValue(15.0, 0.07) // 0.075 //0.11
                    angleFeedForwardCurve.storeValue(32.0, 0.04)  // 0.06  //0.09

//                    println("angle: $angle      f: $angleFeedForward")

                    angleMotor.setRawOffset(angle.degrees)
                    if ((climbMode && !bungeeTakeOver) || OI.operatorRightX.absoluteValue > 0.1) {
                            if (OI.operatorRightX.absoluteValue > 0.1) {
                                angleSetpoint += OI.operatorRightX * 0.1
                                bungeeTakeOver = false
                                println("operatorRightX")
                            }
                            val power = anglePDController.update(angleSetpoint - angle)
                            angleSetPower(power + angleFeedForward)
//                        angleMotor.setPositionSetpoint(angleSetpoint, angleFeedForward)
//                        angleMotor.setPositionSetpoint(angleSetpoint)F
                            //                        println("pdController setting angle power to ${power + angleFeedForward}")
                    } else if (!tuningMode) {
                        angleSetPower(0.0)
                    }
                    if (OI.operatorLeftY.absoluteValue > 0.1 && climbMode) heightSetpoint -= OI.operatorLeftY * 0.45
                    if ((OI.operatorLeftTrigger > 0.1 || OI.operatorRightTrigger > 0.1)/* && !climbMode*/) {  //!climbMode commented out to allow for zeroing in climbMode
                        setPower((OI.operatorLeftTrigger - OI.operatorRightTrigger) * 0.5)
                    } else {
                        if (heightSetpoint > 1.5 && !climbMode) {
                            heightSetpoint -= 0.05
                        } else if (heightSetpoint < 1.0 && !climbMode) {
                            heightSetpoint = 1.0
                        }
                        heightMotor.setPositionSetpoint(heightSetpoint)
                    }
//                    println("angleOutput: ${angleMotor.output}")
                }
        }
    }
    fun setStatusFrames(forClimb : Boolean = false) {
        val framePeriod_1 = if (forClimb) 10 else 100
        val framePeriod_2 = 2*framePeriod_1
        println("height statusframe1 from ${heightMotor.getStatusFramePeriod(StatusFrame.Status_1_General)} to $framePeriod_1")
        println("height statusframe2 from ${heightMotor.getStatusFramePeriod(StatusFrame.Status_2_Feedback0)} to $framePeriod_2")
        heightMotor.setStatusFramePeriod(StatusFrame.Status_1_General, framePeriod_1)
        heightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, framePeriod_2)
        angleMotor.setStatusFramePeriod(StatusFrame.Status_1_General, framePeriod_1)
        angleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, framePeriod_2)
    }

    override fun postEnable() {
        heightSetpoint = height
        climbMode = false
    }

    fun setPower(power: Double) {
        heightMotor.setPercentOutput(power)
    }

    fun angleSetPower(power: Double) {
        if (!bungeeTakeOver) {
            angleMotor.setPercentOutput(power)
        } else {
            angleMotor.setPercentOutput(0.0)
        }
    }

    fun zeroClimb() {
        heightMotor.setRawOffset(0.0.radians)
        heightSetpoint = 0.0
    }
    fun angleChangeTime(target: Double) : Double {
        val distance = (angle - target).absoluteValue
        val rate = 45.0 / 1.0 //20.0 / 1.0  // degrees per sec
        return distance / rate
    }

    suspend fun changePosition(current: Double, target: Double, time : Double, function: (value : Double) -> (Unit)) {
        val curve = MotionCurve()
        curve.storeValue(0.0, current)
        curve.storeValue(time, target)
        val timer = Timer()
        timer.start()
        periodic {
            val t = timer.get()
            function(curve.getValue(t))
            if (t >= curve.length) {
                stop()
            }
        }
    }

    suspend fun changeAngle(target: Double, minTime: Double = 0.0) {
        var time = angleChangeTime(target)
        if (minTime > time) {
            println("Time extended for changeAngle using minTime: $minTime")
            time = minTime
        }
        changePosition(angle, target, time) { value: Double ->
            angleSetpoint = value
            updatePositions()
        }
    }

    fun heightChangeTime(target: Double) : Double {
        val distance = (height - target)
        val rate = if (distance < 0.0) 40.0 else 20.0  // inches per sec
        return distance.absoluteValue / rate
    }

    suspend fun changeHeight(target: Double, minTime: Double = 0.0) {
        var time = heightChangeTime(target)
        if (minTime > time) {
            println("Time extended for changeHeight using minTime: $minTime")
            time = minTime
        }
        changePosition(height, target, time) { value: Double ->
            heightSetpoint = value
            updatePositions()
        }
    }

    fun updatePositions() {
        heightMotor.setPositionSetpoint(heightSetpoint)
        if (!bungeeTakeOver) {
            if (isAngleMotorControlled) {
                angleMotor.setPositionSetpoint(angleSetpoint, angleFeedForward)
//            println("motor setting angle power to ${angleMotor.output}")
            } else {
                val power = anglePDController.update(angleSetpoint - angle)
                angleSetPower(power + angleFeedForward)
                //feedforward for this not tested!!
//            println("pdController setting angle power to ${power + angleFeedForward}")
            }
        } else {
            angleSetPower(0.0)
        }
    }

    override suspend fun default() {
        periodic {
            if (tuningMode) {
                println("is tuning mode")
///                updatePositions()
            } else if (OI.operatorLeftY.absoluteValue > 0.1 || OI.operatorRightY.absoluteValue > 0.1) {
//                heightSetpoint -= OI.operatorLeftY * 0.45
//                angleSetpoint += OI.operatorRightY * 0.2
//                heightMotor.setPositionSetpoint(heightSetpoint)
            }

        }
    }


}
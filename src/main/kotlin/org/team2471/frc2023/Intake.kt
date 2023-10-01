package org.team2471.frc2023

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DriverStation
//import io.github.pseudoresonance.pixy2api.Pixy2
//import io.github.pseudoresonance.pixy2api.Pixy2CCC
//import io.github.pseudoresonance.pixy2api.links.SPILink
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.*
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller
import org.team2471.frc.lib.math.deadband
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2023.Robot.isCompBot
import kotlin.math.absoluteValue
import kotlin.math.sin


object Intake : Subsystem("Intake") {
    val wristMotor = MotorController(SparkMaxID(Sparks.WRIST))
    val intakeMotor = MotorController(SparkMaxID(Sparks.INTAKE)) //intake bad
    val wristSensor = AnalogInput(AnalogSensors.WRIST)

    private val table = NetworkTableInstance.getDefault().getTable(Intake.name)
    val wristEntry = table.getEntry("Wrist Angle")
    val wristSetpointEntry = table.getEntry("Wrist Setpoint")
    val intakeCurrentEntry = table.getEntry("Intake Currrent")
    val cubeDetectEntry = table.getEntry("Cube Detect Power")
    val coneDetectEntry = table.getEntry("Cone Detect Power")
    val cubeHoldPowerEntry = table.getEntry("Cube Hold Power")
    val coneHoldPowerEntry = table.getEntry("Cone Hold Power")
    val holdingObjectEntry = table.getEntry("Holding Object")
    val wristTicksOffsetEntry = table.getEntry("Wrist Ticks Offset")
    val wristTicksEntry = table.getEntry("Wrist Ticks")
    val wristMotorAngleEntry = table.getEntry("Wrist Motor Angle")
    val wristEncoderAngleEntry = table.getEntry("Wrist Encoder Angle")
    val wristEncoderRawAngleEntry = table.getEntry("Wrist Encoder Raw Angle")

    val wristAngle: Angle
        get() {
                val wristAng = wristMotor.position.degrees
                if ((wristAng - prevWristAngle).asDegrees.absoluteValue > 15.0 && (wristAng.asDegrees + 89.0).absoluteValue < 1.0) {
                    println("Difference from wristAngle and prevAngle > 15. wristAngle: $wristAng prevWristAngle: $prevWristAngle")
                    return prevWristAngle
                } else {
                    return wristAng
                }
        }
    val wristEncoderAngle: Angle
        get() {
            if (!isCompBot) {
                return wristEncoderRawAngle + Arm.elbowAngle
            } else {
                return -90.0.degrees
            }
        }
    val wristEncoderRawAngle: Angle
        get() {
            if (!isCompBot) {
                return (wristSensor.value.degrees - wristTicksOffsetEntry.getDouble(1695.0).degrees) * 90.0 / 1054.0
            } else {
                return -90.0.degrees
            }
        }



    var prevWristAngle: Angle = -90.0.degrees

    var wristOffset = 0.0.degrees
    var wristSetpoint: Angle = wristAngle
        set(value) {
            field = value.asDegrees.coerceIn(wristMin.asDegrees, wristMax.asDegrees).degrees
//            println("field: ${round(field.asDegrees,1)}     offset: ${round(wristOffset.asDegrees, 1)}")
            wristMotor.setPositionSetpoint((field + wristOffset).asDegrees)
            if (FieldManager.homeField) {
                wristSetpointEntry.setDouble((field + wristOffset).asDegrees)
            }
        }
    val wristTicks: Int
        get() = if (isCompBot) 0 else wristSensor.value

    fun wristCoastMode() {
        wristMotor.coastMode()
    }

    fun wristBrakeMode() {
        wristMotor.brakeMode()
    }
    val wristError
        get() = wristAngle - wristSetpoint

    var wristIsReset = false

    var wristMin = -90.0.degrees
        get() = round(-140.0+ Arm.elbowAngle.asDegrees, 4).degrees
    var wristMax = 90.0.degrees
        get() = round(140.0 + Arm.elbowAngle.asDegrees, 4).degrees
    var linearFilter = LinearFilter.movingAverage(10)
    var holdingObject: Boolean = false

    var holdDetectedTime = -5.0

//    lateinit var pixy : Pixy2
//    var coneFacingUp = false
    var coneToward = true //not supposed to be a get. We want to set it when pixy facing ground

    const val INTAKE_POWER = 1.0
    const val INTAKE_CONE = -1.0
    const val INTAKE_CUBE = 0.70
    var HOLD_CONE = -0.3 //change default instead
        get() = coneHoldPowerEntry.getDouble(-0.17).coerceIn(-0.5, 0.0) //coerce to prevent too large values in shuffleboard
    var HOLD_CUBE = 0.075 //change default instead
        get() =  cubeHoldPowerEntry.getDouble(0.075).coerceIn(0.0, 0.5)//coerce to prevent too large values in shuffleboard
    var DETECT_CONE = 20
//        get() = coneDetectEntry.getInteger(20.toLong()).toInt()
    var DETECT_CUBE = 13
//        get() = cubeDetectEntry.getInteger(13.toLong()).toInt()
    const val CONE_TOWARD_SPIT = 1.0
    const val CONE_AWAY_SPIT = 1.0
    const val CUBE_SPIT = -0.2

    init {
//        initializePixy()
        wristMotor.restoreFactoryDefaults()
        intakeMotor.restoreFactoryDefaults() //intake bad
        wristMotor.config(20) {
            feedbackCoefficient = 261.0 / 1273.0 * 198.0 / 360.0  //last one is fudge factor
            coastMode()
            pid {
                p(0.00014) //00002
            }
            currentLimit(0, 60, 0)
            burnSettings()
        }
        intakeMotor.config { //intake bad
            brakeMode()
            currentLimit(0, 50, 0)
            burnSettings()
        }
        if (!wristTicksOffsetEntry.exists()) {
            wristTicksOffsetEntry.setDouble(wristSensor.value.toDouble())
            wristTicksOffsetEntry.setPersistent()
            println("Wrist didn't exist")
        }

        wristMotor.setRawOffset(if (isCompBot) -90.0 else wristEncoderAngle.asDegrees)
        //wristSetpoint = wristAngle
//        wristMotor.setRawOffset(-90.0)
        wristSetpoint = wristMotor.position.degrees
        GlobalScope.launch(MeanlibDispatcher) {
            if (FieldManager.homeField) {
                wristSetpointEntry.setDouble(wristSetpoint.asDegrees)
                coneHoldPowerEntry.setDouble(HOLD_CONE)
                coneDetectEntry.setInteger(DETECT_CONE.toLong())
                cubeHoldPowerEntry.setDouble(HOLD_CUBE)
                cubeDetectEntry.setInteger(DETECT_CUBE.toLong())
           }
//            coneMaxBlockCount.setInteger(20)
//            coneMinArea.setInteger(20)
//            coneMinBlockY.setInteger(100)
            periodic {
                if (!isCompBot) {
                    /*if ((wristAngle.asDegrees - wristMotor.position).absoluteValue > 3) {
                        println("resetting wrist motor offset to: ${wristAngle.asDegrees}  prev motor position: ${wristMotor.position}  ${wristMotor.analogAngle}")
                        wristMotor.setRawOffset(wristAngle.asDegrees)
                    }*/
                }

                holdingObject = linearFilter.calculate(intakeMotor.current) > if (NodeDeckHub.isCone) DETECT_CONE else DETECT_CUBE
                wristEntry.setDouble(wristAngle.asDegrees)
                wristTicksEntry.setDouble(wristTicks.toDouble())
                wristMotorAngleEntry.setDouble(wristMotor.position)
                wristEncoderAngleEntry.setDouble(wristEncoderAngle.asDegrees)
                wristEncoderRawAngleEntry.setDouble(wristEncoderRawAngle.asDegrees)

                var wrist = OI.operatorController.rightThumbstickY.deadband(0.2)
                wrist *= 45.0 * 0.02 // degrees per second, time 1/50 second

                wristMotor.setPercentOutput(wrist * 0.5)
                wristOffset += wrist.degrees
                wristSetpoint += 0.0.degrees
                if (FieldManager.homeField) {
                    intakeCurrentEntry.setDouble(intakeMotor.current) //intake bad
                }

                prevWristAngle = wristAngle
            }
        }
    }

    override suspend fun default() {
        periodic {
//            if (Arm.shoulderAngle.asDegrees.absoluteValue < 3.0 && Arm.elbowAngle.asDegrees.absoluteValue < 5.0) {
//                if (wristAngle < -65.0.degrees) wristSetpoint = -90.0.degrees
//                if (wristAngle > 65.0.degrees) wristSetpoint = 90.0.degrees
//            }
        }
    }

    override fun preEnable() {
        wristMotor.setPercentOutput(0.0)
        intakeMotor.setPercentOutput(0.0)
        //wristSetpoint = wristAngle
        wristSetpoint = wristMotor.position.degrees
        wristOffset = 0.0.degrees
    }

//    fun initializePixy() {
//        pixy = Pixy2.createInstance(SPILink()) // Creates a new Pixy2 camera using SPILink
//        pixy.init() // Initializes the camera and prepares to send/receive data
//        pixy.setLamp(0.toByte(), 0.toByte()) // Turns the LEDs on
//        pixy.setLED(255, 255, 255) // Sets the RGB LED to full white
//    }

//    fun coneUp(): Boolean? {
//        try {
//            val blockCount: Int = pixy.ccc.getBlocks(false, (Pixy2CCC.CCC_SIG1 + Pixy2CCC.CCC_SIG2), coneMaxBlockCount.getInteger(10).toInt())
//            //println("Found $blockCount blocks $blockCount!") // Reports number of blocks found
//            if (blockCount <= 0) {
//                return false // If blocks were not found, stop processing
//            }
//            val blocks: ArrayList<Pixy2CCC.Block> = pixy.ccc.blockCache // Gets a list of all blocks found by the Pixy2
//            for (block in blocks) { // Loops through all blocks and finds the widest one
//                if (block.y < coneMinBlockY.getInteger(100) && block.area > coneMinArea.getInteger(100)) {
//                    return true
//                }
//            }
//        } catch (ex:java.lang.Exception) {
//            println("Pixy 2 exception")
//            return null
//        }
//        return false
//    }

//    fun getBiggestBlock(): Pixy2CCC.Block? {
//        // Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
//        // does not wait for new data if none is available,
//        // and limits the number of returned blocks to 25, for a slight increase in efficiency
//        var largestBlock: Pixy2CCC.Block? = null
//        try {
//            val blockCount: Int = pixy.ccc.getBlocks(false, Pixy2CCC.CCC_SIG1.toInt(), 25)
//            println("Found $blockCount blocks!") // Reports number of blocks found
//            if (blockCount <= 0) {
//                return null // If blocks were not found, stop processing
//            }
//            val blocks: ArrayList<Pixy2CCC.Block> = pixy.ccc.blockCache // Gets a list of all blocks found by the Pixy2
//            for (block in blocks) { // Loops through all blocks and finds the widest one
//                if (largestBlock == null) {
//                    largestBlock = block
//                } else if (block.width > largestBlock.width) {
//                    largestBlock = block
//                }
//            }
//            if (largestBlock != null) {
//                println("Area: ${largestBlock.height * largestBlock.width} , distance from ground ${largestBlock.y + largestBlock.height}")
//            }
//        } catch (ex:java.lang.Exception) {
//            println("Pixy 2 exception")
//        }
//        return largestBlock
//    }

    override fun onDisable() {
        intakeMotor.setPercentOutput(0.0)
    }
}

//val Pixy2CCC.Block.area : Int
//    get() = this.width * this.height


//fun setPivotPower() {
//    val pError = (Intake.pivotSetpoint + Intake.pivotOffset - Intake.pivotAngle).wrap().asDegrees
//    val openLoopPower = Intake.pivotPDController.update(pError).coerceIn(-1.0, 1.0)
//    if ((Intake.pivotSetpoint.asDegrees - Intake.pivotAngle.asDegrees).absoluteValue > 40.0 && DriverStation.isEnabled()) println("pivotError: ${round(pError, 1)}    openLoopPower: ${round(openLoopPower, 1)}")
//    val power = openLoopPower + Intake.pFeedForward
//    Intake.pivotMotor.setPercentOutput(power)
//    println("power: $power  setpoint ${Intake.pivotSetpoint} current: ${Intake.pivotMotor.current}")
//}
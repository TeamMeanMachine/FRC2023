package org.team2471.frc2023

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DigitalInput
import io.github.pseudoresonance.pixy2api.Pixy2
import io.github.pseudoresonance.pixy2api.Pixy2CCC
import io.github.pseudoresonance.pixy2api.links.SPILink
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
    val pivotMotor = MotorController(TalonID(Talons.INTAKE_PIVOT))
    val intakeMotor = MotorController(SparkMaxID(Sparks.INTAKE)) //intake bad
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
    val coneOrientationEntry = table.getEntry("Cone Orientation")
    val coneMinBlockY = table.getEntry("Pixy Min Block Y")
    val coneMinArea = table.getEntry("Pixy Min Area")
    val coneMaxBlockCount = table.getEntry("Pixy Max Blocks")
    val coneFacingUpEntry = table.getEntry("Cone Is Up")
    val coneDebouncer = Debouncer(0.5,Debouncer.DebounceType.kFalling)
    val cubeDetectEntry = table.getEntry("Cube Detect Power")
    val coneDetectEntry = table.getEntry("Cone Detect Power")
    val cubeHoldPowerEntry = table.getEntry("Cube Hold Power")
    val coneHoldPowerEntry = table.getEntry("Cone Hold Power")


    val wristAngle: Angle
        get() = wristMotor.position.degrees
    var wristOffset = 0.0.degrees
    var wristSetpoint: Angle = wristAngle
        set(value) {
            field = value.asDegrees.coerceIn(wristMin.asDegrees, wristMax.asDegrees).degrees
//            println("field: ${round(field.asDegrees,1)}     offset: ${round(wristOffset.asDegrees, 1)}")
            wristMotor.setPositionSetpoint((field + wristOffset).asDegrees)
            wristSetpointEntry.setDouble((field + wristOffset).asDegrees)
        }

    val pivotAnalogAngle: Angle
        get() = ((pivotSensor.value - if (isCompBot) 913.0 else 2116.0).degrees / 4096.0 * 360.0).wrap()
    var pivotOffset: Angle = 0.0.degrees

    val pivotAngle: Angle
        get() {
            val returnValue = pivotAnalogAngle.unWrap(prevPivotAngle)
            prevPivotAngle = returnValue
            return returnValue
        }

    var prevPivotAngle = pivotAngle
    var pivotSetpoint: Angle = pivotAngle
        set(value) {
            if (value > 270.0.degrees) {  // too far, have to twist back to equivalent angle to protect wires
                field = value - 360.0f.degrees;
            }
            else if (value < -270.0.degrees) {  // too far, have to twist ahead to equivalent angle to protect wires
                field = value + 360.0f.degrees;
            }
            else {
                field = value
            }
            setPivotPower()
            pivotSetpointEntry.setDouble(field.asDegrees)
        }
    val pivotPDController = PDController(0.100, 0.001) //0.1, 0.001  //0.03, 0.04)   //0.35, 0.03
    var prevPFeedForward = 0.0
    val pFeedForward: Double
        get() {
            try {
                val temp: Double = pivotCurve.getValue((pivotAngle + if (wristAngle > 0.0.degrees) 180.0.degrees else 0.0.degrees).wrap().asDegrees) * sin(wristAngle.asRadians.absoluteValue)
                prevPFeedForward = temp
            } catch (ex: Exception){
                println("pFeedForward returned null.")
            }
            return prevPFeedForward
        }
    val pivotCurve = MotionCurve()

    var wristIsReset = false

    var wristMin = -90.0.degrees
        get() = round(-140.0+ Arm.elbowAngle.asDegrees, 4).degrees
    var wristMax = 90.0.degrees
        get() = round(140.0 + Arm.elbowAngle.asDegrees, 4).degrees
    var linearFilter = LinearFilter.movingAverage(10)
    var holdingObject: Boolean = false
        get() = linearFilter.calculate(intakeMotor.current) > if (NodeDeckHub.isCone) DETECT_CONE else DETECT_CUBE

    var holdDetectedTime = -5.0

    lateinit var pixy : Pixy2
    var coneFacingUp = false
    var coneToward = true //not supposed to be a get. We want to set it when pixy facing ground

    const val INTAKE_POWER = 1.0
    const val INTAKE_CONE = -1.0
    const val INTAKE_CUBE = 0.55
    var HOLD_CONE = -0.20
        get() = coneHoldPowerEntry.getDouble(-0.20)
    var HOLD_CUBE = 0.05
        get() = cubeHoldPowerEntry.getDouble(0.05)
    var DETECT_CONE = 20
        get() = coneDetectEntry.getInteger(20.toLong()).toInt()
    var DETECT_CUBE = 13
        get() = cubeDetectEntry.getInteger(13.toLong()).toInt()
    const val CONE_TOWARD_SPIT = 0.6
    const val CONE_AWAY_SPIT = 1.0
    const val CUBE_SPIT = -0.20 // was at -0.25

    init {
        initializePixy()
        wristMotor.restoreFactoryDefaults()
        intakeMotor.restoreFactoryDefaults() //intake bad
        wristMotor.config(20) {
            feedbackCoefficient = 261.0 / 1273.0 * 198.4 / 360.0  //last one is fudge factor
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
        intakeMotor.config { //intake bad
            brakeMode()
            currentLimit(0, 60, 0)
            burnSettings()
        }

        pivotCurve.storeValue(-185.0, 0.0)
        pivotCurve.storeValue(-179.0, 0.0)
        pivotCurve.storeValue(-170.0, 0.13) //.05
        pivotCurve.storeValue(-90.0, 0.26)
        pivotCurve.storeValue(-45.0, 0.11)
        pivotCurve.storeValue(0.0, 0.0)
        pivotCurve.storeValue(45.0, -0.11)
        pivotCurve.storeValue(90.0, -0.23)
        pivotCurve.storeValue(170.0, -0.10)
        pivotCurve.storeValue(179.0, 0.0)
        pivotCurve.storeValue(185.0, 0.0)

        wristMotor.setRawOffset(-90.0)
        GlobalScope.launch(MeanlibDispatcher) {
            var tempPivot: Angle

            wristSetpointEntry.setDouble(wristAngle.asDegrees)
            pivotSetpointEntry.setDouble(pivotAngle.asDegrees)
            coneHoldPowerEntry.setDouble(-20.0)
            coneDetectEntry.setInteger(20)
            cubeHoldPowerEntry.setDouble(0.05)
            cubeDetectEntry.setInteger(13)

            coneMaxBlockCount.setInteger(20)
            coneMinArea.setInteger(20)
            coneMinBlockY.setInteger(100)

            periodic {

                wristEntry.setDouble(wristAngle.asDegrees)
                pivotEntry.setDouble(pivotAngle.asDegrees)
                pivotSetpointEntry.setDouble(pivotSetpoint.asDegrees)
                pErrorEntry.setDouble(pivotSetpoint.asDegrees - pivotAngle.asDegrees)
                pFeedEntry.setDouble(pFeedForward)
                val coneOrientation = coneUp()
                coneFacingUp = coneDebouncer.calculate(coneOrientation == true)

                coneOrientationEntry.setInteger(if (coneOrientation == null) -1 else if (coneOrientation) 1 else 0)
                coneFacingUpEntry.setBoolean(coneFacingUp)

                var pivot = OI.operatorController.rightThumbstickX.deadband(0.2)
                var wrist = OI.operatorController.rightThumbstickY.deadband(0.2)
                pivot *= 45.0 * 0.02  // degrees per second, time 1/50 second
                wrist *= 45.0 * 0.02

//                wristMotor.setPercentOutput(wrist * 0.5)
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

                intakeCurrentEntry.setDouble(intakeMotor.current) //intake bad
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
            if (Arm.shoulderAngle.asDegrees.absoluteValue < 3.0 && Arm.elbowAngle.asDegrees.absoluteValue < 5.0) {
                if (wristAngle < -65.0.degrees) wristSetpoint = -90.0.degrees
                if (wristAngle > 65.0.degrees) wristSetpoint = 90.0.degrees
            }
        }
    }

    override fun preEnable() {
        wristMotor.setPercentOutput(0.0)
        intakeMotor.setPercentOutput(0.0)
        pivotSetpoint = pivotAngle
        wristSetpoint = wristAngle
    }

    fun initializePixy() {
        pixy = Pixy2.createInstance(SPILink()) // Creates a new Pixy2 camera using SPILink
        pixy.init() // Initializes the camera and prepares to send/receive data
        pixy.setLamp(0.toByte(), 0.toByte()) // Turns the LEDs on
        pixy.setLED(255, 255, 255) // Sets the RGB LED to full white
    }

    fun coneUp(): Boolean? {
        try {
            val blockCount: Int = pixy.ccc.getBlocks(false, (Pixy2CCC.CCC_SIG1 + Pixy2CCC.CCC_SIG2), coneMaxBlockCount.getInteger(10).toInt())
            //println("Found $blockCount blocks $blockCount!") // Reports number of blocks found
            if (blockCount <= 0) {
                return false // If blocks were not found, stop processing
            }
            val blocks: ArrayList<Pixy2CCC.Block> = pixy.ccc.blockCache // Gets a list of all blocks found by the Pixy2
            for (block in blocks) { // Loops through all blocks and finds the widest one
                if (block.y < coneMinBlockY.getInteger(100) && block.area > coneMinArea.getInteger(100)) {
                    return true
                }
            }
        } catch (ex:java.lang.Exception) {
            println("Pixy 2 exception")
            return null
        }
        return false
    }

    fun getBiggestBlock(): Pixy2CCC.Block? {
        // Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
        // does not wait for new data if none is available,
        // and limits the number of returned blocks to 25, for a slight increase in efficiency
        var largestBlock: Pixy2CCC.Block? = null
        try {
            val blockCount: Int = pixy.ccc.getBlocks(false, Pixy2CCC.CCC_SIG1.toInt(), 25)
            println("Found $blockCount blocks!") // Reports number of blocks found
            if (blockCount <= 0) {
                return null // If blocks were not found, stop processing
            }
            val blocks: ArrayList<Pixy2CCC.Block> = pixy.ccc.blockCache // Gets a list of all blocks found by the Pixy2
            for (block in blocks) { // Loops through all blocks and finds the widest one
                if (largestBlock == null) {
                    largestBlock = block
                } else if (block.width > largestBlock.width) {
                    largestBlock = block
                }
            }
            if (largestBlock != null) {
                println("Area: ${largestBlock.height * largestBlock.width} , distance from ground ${largestBlock.y + largestBlock.height}")
            }
        } catch (ex:java.lang.Exception) {
            println("Pixy 2 exception")
        }
        return largestBlock
    }

    override fun onDisable() {
        intakeMotor.setPercentOutput(0.0)
    }
}

val Pixy2CCC.Block.area : Int
    get() = this.width * this.height

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
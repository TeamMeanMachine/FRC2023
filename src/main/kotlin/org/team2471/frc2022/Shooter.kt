package org.team2471.frc2022

import com.revrobotics.ColorSensorV3
import edu.wpi.first.math.filter.LinearFilter
//import edu.wpi.first.networktables.EntryListenerFlags
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.TalonID
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.math.square
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.asFeet
import kotlin.math.absoluteValue
import kotlin.math.sqrt


object Shooter : Subsystem("Shooter") {
    val tuningMode = false
    val shootingMotorOne = MotorController(FalconID(Falcons.SHOOTER))
    val shootingMotorTwo = MotorController(FalconID(Falcons.SHOOTER_TWO))
    val pitchMotor = MotorController(TalonID(Talons.PITCH))
    private val table = NetworkTableInstance.getDefault().getTable(name)
    val pitchEncoder = DutyCycleEncoder(DigitalSensors.SHOOTER_PITCH)
    val rpmErrorFilter = LinearFilter.movingAverage(10)

    private val i2cPort: I2C.Port = I2C.Port.kMXP
    private val colorSensor = ColorSensorV3(i2cPort)
    val colorEntry = table.getEntry("Color")
    val isKnownShot: knownShotType
        get() {
            return knownShotType.valueOf(SmartDashboard.getString("KnownShot/selected", "notset").uppercase())
        }

    const val aimMaxError = 3.0
    const val rpmMaxError = 200.0
    const val pitchMaxError = 2.5

    var autoOdomPitch = 0.0
        get() = backPitchCurve.getValue(Drive.position.length + 2.25)
    var autoOdomRPM = 0.0
        get() = backRPMCurve.getValue(Drive.position.length + 2.25)

    enum class knownShotType {
        NOTSET, FENDER, WALL, SAFE_FRONT, SAFE_BACK
    }

    val rpmEntry = table.getEntry("RPM")
    val rpmSetpointEntry = table.getEntry("RPM Setpoint")
    val aimMaxErrorEntry = table.getEntry("aimMaxError")
    val pitchErrorEntry = table.getEntry("pitchError")
    val rpmMaxErrorEntry = table.getEntry("rpm maxError")
    val pitchMaxErrorEntry = table.getEntry("pitchMaxError")
    val rpmErrorEntry = table.getEntry("RPM Error")
    val frontRPMOffsetEntry = table.getEntry("Front RPM Offset")
    val backRPMOffsetEntry = table.getEntry("Back RPM Offset")
    val pitchEntry = table.getEntry("Pitch")
    val pitchSetpointEntry = table.getEntry("Pitch Setpoint")
    val shootModeEntry = table.getEntry("Shoot Mode")
    val frontPitchOffsetEntry = table.getEntry("Front Pitch Offset")
    val backPitchOffsetEntry = table.getEntry("Back Pitch Offset")
    val pitchGoodEntry = table.getEntry("pitchGood")
    val aimGoodEntry = table.getEntry("aimGood")
    val rpmGoodEntry = table.getEntry("rpmGood")
    val shootMotorsGoodEntry = table.getEntry("shootMotorsGood")
    val allGoodEntry = table.getEntry("allGood")
    val frontPitch5Entry = table.getEntry("frontPitch5Entry")
    val frontPitch10Entry = table.getEntry("frontPitch10Entry")
    val frontPitch15Entry = table.getEntry("frontPitch15Entry")
    val frontPitch20Entry = table.getEntry("frontPitch20Entry")
    val frontRPM5Entry = table.getEntry("frontRPM5Entry")
    val frontRPM10Entry = table.getEntry("frontRPM10Entry")
    val frontRPM15Entry = table.getEntry("frontRPM15Entry")
    val frontRPM20Entry = table.getEntry("frontRPM20Entry")
    val backPitch5Entry = table.getEntry("backPitch5Entry")
    val backPitch10Entry = table.getEntry("backPitch10Entry")
    val backPitch15Entry = table.getEntry("backPitch15Entry")
    val backPitch20Entry = table.getEntry("backPitch20Entry")
    val backRPM5Entry = table.getEntry("backRPM5Entry")
    val backRPM10Entry = table.getEntry("backRPM10Entry")
    val backRPM15Entry = table.getEntry("backRPM15Entry")
    val backRPM20Entry = table.getEntry("backRPM20Entry")
    val distanceEntry = table.getEntry("fixedDistances")
    val colorAlignedEntry = table.getEntry("colorAligned")
    val useAutoOdomEntry = table.getEntry("useAutoPreset")
    val distFlyOffsetEntry = table.getEntry("Pitch Fly Offset")
//    val rpmFlyOffsetEntry = table.getEntry("RPM Fly Offset")
    private val knownShotChooser = SendableChooser<String?>().apply {
        setDefaultOption("NOTSET", "notset")
        addOption("FENDER", "fender")
        addOption("WALL", "wall")
        addOption("SAFE_FRONT", "safe_front")
        addOption("SAFE_BACK", "safe_back")
    }


    val pitchFilter = LinearFilter.movingAverage(15) //if (tuningMode) {10} else {2})

    const val PITCH_LOW = -26.2
    const val PITCH_HIGH = 35.0

    const val PROXIMITY_STAGED_MIN = 200.0
    const val PROXMITY_STAGED_MAX_SAFE = 350.0

    var allGood = false
    var aimGood = false
    var rpmGood = false
    var pitchGood = false
    var shootMotorsGood = false
    var isCargoAlignedWithAlliance = true
    var pastMinWait = false

    var distFlyOffset: Double = 0.0
        get() = 0.0 //distFlyOffsetEntry.getDouble(0.0)

    //        get() = distFlyOffsetCurve.getValue(Drive.radialVelocity)
    var pitchOffset = if (isCompBot) 1.3 else -76.0
    var pitch: Double = 0.0
        get() = (pitchEncoder.absolutePosition - 0.218) * 33.0 / 0.182 + pitchOffset
        set(value) {
            pitchSetpoint = value
            field = value
        }
    var pitchSetpoint = pitch //demo
        //        get() = pitchSetpointEntry.getDouble(20.0) //normal demo
        get() //= if (Feeder.isAuto) pitchSetpointEntry.getDouble(20.0) else backPitchCurve.getValue(Limelight.distance.asFeet + distFlyOffset) //rotary demo
        {
            if (tuningMode) {
                field = pitchSetpointEntry.getDouble(10.0)
            } else if (isKnownShot != knownShotType.NOTSET) {
                field = when (isKnownShot) {
                    knownShotType.FENDER -> 17.5
                    knownShotType.SAFE_FRONT -> 35.0
                    knownShotType.SAFE_BACK -> -19.0
                    knownShotType.WALL -> 35.0
                    else -> 15.0
                }
            } else if (Feeder.isAuto && useAutoOdomEntry.getBoolean(false)) {
                field = autoOdomPitch
            } else if (!Limelight.useFrontLimelight && Limelight.hasValidBackTarget) {
                val tempPitch = backPitchCurve.getValue(Limelight.distance.asFeet + distFlyOffset)
                field = tempPitch
            } else if (Limelight.useFrontLimelight && Limelight.hasValidFrontTarget) {
                val tempPitch = frontPitchCurve.getValue(Limelight.distance.asFeet + distFlyOffset)
                field = tempPitch
            } else {
                field = 32.0
            }
            // don't allow values outside of range even with offset
            field = field.coerceIn(PITCH_LOW - distFlyOffset, PITCH_HIGH - distFlyOffset)
//            println("tuningMode $tuningMode     useFrontLL ${Limelight.useFrontLimelight}     frontTarget ${Limelight.hasValidFrontTarget}        backTarget ${Limelight.hasValidBackTarget}")
            pitchSetpointEntry.setDouble(field)
            return field
        }
        set(value) {
            field = value.coerceIn(PITCH_LOW, PITCH_HIGH)
            pitchSetpointEntry.setDouble(field)
        }

    var pitchPDEnable = true
    val pitchPDController = PDController(0.05, 0.095) //.11) //0.06, 0.095) //0.055, 0.03) //0.06, 0.0) // d 0.1
    const val K_PITCH_FEED_FORWARD = -0.22
    val pitchIsReady: Boolean
        get() {
//            println("${pitchPDEnable}     ${pitchSetpoint > PITCH_LOW}     ${pitchSetpoint < PITCH_HIGH}     ${pitchEncoder.isConnected}")
            return pitchPDEnable && pitch > (PITCH_LOW - 2.0) && pitchSetpoint < (PITCH_HIGH + 2.0) && pitchEncoder.isConnected
        }
    val backPitchCurve: MotionCurve = MotionCurve()
    val frontPitchCurve: MotionCurve = MotionCurve()
    val frontRPMCurve: MotionCurve = MotionCurve()
    val backRPMCurve: MotionCurve = MotionCurve()
//    val rpmFlyOffsetCurve: MotionCurve = MotionCurve()
    val distFlyOffsetCurve: MotionCurve = MotionCurve()

//    var rpmFlyOffset: Double = 0.0
//        get() = rpmFlyOffsetEntry.getDouble(0.0)
//        get() = rpmFlyOffsetCurve.getValue(Drive.filteredRadialVelocity)
    var rpmSecondOffset = 0.0
    var rpmSetpoint: Double = 0.0 //demo
        get() {
            if (tuningMode) {
                rpmSetpointEntry.getDouble(1300.0)
            } else if (Feeder.isAuto && useAutoOdomEntry.getBoolean(false)) {
                field = autoOdomRPM
            } else if (isKnownShot != knownShotType.NOTSET) {
                field = frontLLRPMOffset * when (isKnownShot) {
                    knownShotType.FENDER -> 3200.0
                    knownShotType.SAFE_FRONT -> 4000.0 //4200
                    knownShotType.SAFE_BACK -> 3500.0
                    knownShotType.WALL -> 3450.0
                    else -> 3200.0
                }
            } else if (!Limelight.useFrontLimelight && Limelight.hasValidBackTarget) {
                field = backRPMCurve.getValue(Limelight.distance.asFeet + distFlyOffset) * backLLRPMOffset
            } else if (Limelight.useFrontLimelight && Limelight.hasValidFrontTarget) {
                field = frontRPMCurve.getValue(Limelight.distance.asFeet + distFlyOffset) * frontLLRPMOffset
            } else {
                field = 3200.0
            }
//            field += rpmFlyOffset
            field += rpmSecondOffset
            if (!tuningMode) {
                rpmSetpointEntry.setDouble(field)
            }
            return field
        }

    var rpm: Double
        get() = shootingMotorOne.velocity
        set(value) {
            shootingMotorOne.setVelocitySetpoint(value)
            shootingMotorTwo.setVelocitySetpoint(value)
        }
    var rpmError = rpm
        get() = rpmErrorEntry.getDouble(rpmSetpoint - rpm)

    var shootMode = false



    const val RED = 'r'
    const val BLUE = 'b'
    const val NOTSET = 'n'
    val dsAllianceColor : Char
        get() = when (DriverStation.getAlliance().name.first().lowercaseChar()) {
            'r' -> RED
            'b' -> BLUE
            else -> NOTSET
        }
    var allianceColor = dsAllianceColor
    var stagedColorString = "notset"

    init {
        SmartDashboard.putData("KnownShot", knownShotChooser)

        if (!backPitch5Entry.exists()) {
            backPitch5Entry.setDouble(-8.0)
            backPitch10Entry.setDouble(-19.0)
            backPitch15Entry.setDouble(-27.0)
            backPitch20Entry.setDouble(-30.0)
            frontPitch5Entry.setDouble(24.8)
            frontPitch10Entry.setDouble(31.8)
            frontPitch15Entry.setDouble(36.0)
            frontPitch20Entry.setDouble(36.0)
            backRPM5Entry.setDouble(2800.0)
            backRPM10Entry.setDouble(3300.0)
            backRPM15Entry.setDouble(3950.0)
            backRPM20Entry.setDouble(5200.0)
            frontRPM5Entry.setDouble(3200.0)
            frontRPM10Entry.setDouble(3600.0)
            frontRPM15Entry.setDouble(4550.0)
            frontRPM20Entry.setDouble(5800.0)

            backPitch5Entry.setPersistent()
            backPitch10Entry.setPersistent()
            backPitch15Entry.setPersistent()
            backPitch20Entry.setPersistent()
            frontPitch5Entry.setPersistent()
            frontPitch10Entry.setPersistent()
            frontPitch15Entry.setPersistent()
            frontPitch20Entry.setPersistent()
            backRPM5Entry.setPersistent()
            backRPM10Entry.setPersistent()
            backRPM15Entry.setPersistent()
            backRPM20Entry.setPersistent()
            frontRPM5Entry.setPersistent()
            frontRPM10Entry.setPersistent()
            frontRPM15Entry.setPersistent()
            frontRPM20Entry.setPersistent()
        }

//        frontPitch5Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        frontPitch10Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        frontPitch15Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        frontPitch20Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        backPitch5Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        backPitch10Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        backPitch15Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        backPitch20Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        frontRPM5Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        frontRPM10Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        frontRPM15Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        frontRPM20Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        backRPM5Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        backRPM10Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        backRPM15Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
//        backRPM20Entry.addListener({ event -> rebuildCurves() }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)


        rebuildCurves()

//        rpmFlyOffsetCurve.setMarkBeginOrEndKeysToZeroSlope(false)
        distFlyOffsetCurve.setMarkBeginOrEndKeysToZeroSlope(false)

//        rpmFlyOffsetCurve.storeValue(0.0, 0.0)
//        rpmFlyOffsetCurve.storeValue(0.0, 0.0)
//        rpmFlyOffsetCurve.storeValue(0.0, 0.0)
//        rpmFlyOffsetCurve.storeValue(0.0, 0.0)
        distFlyOffsetCurve.storeValue(0.0, 0.0)
        distFlyOffsetCurve.storeValue(2.0, 1.0)
        distFlyOffsetCurve.storeValue(10.0, 6.0)
//        pitchFlyOffsetCurve.storeValue(0.0, 0.0)
//        pitchFlyOffsetCurve.storeValue(0.0, 0.0)

        shootingMotorOne.config {
//            followersInverted(true)
            coastMode()
            feedbackCoefficient = 1.0 / 2048.0 * 60.0
            pid {
                p(7.5e-6)
                i(0.0)
                d(2.0e-4)
                f(0.0140)
            }
        }
        shootingMotorTwo.config {
            inverted(true)
            coastMode()
            feedbackCoefficient = 1.0 / 2048.0 * 60.0
            pid {
                p(7.5e-6)
                i(0.0)
                d(2.0e-4)
                f(0.0140)
            }
        }

        pitchMotor.config {
            currentLimit(20, 25, 1)
            inverted(!isCompBot)
            brakeMode()
        }

        rpmSetpointEntry.setDouble(rpmSetpoint)
        pitchSetpointEntry.setDouble(pitch)

        GlobalScope.launch(MeanlibDispatcher) {
            var upPressed = false
            var downPressed = false
            var leftPressed = false
            var rightPressed = false
            frontRPMOffsetEntry.setDouble(frontLLRPMOffset)
            backRPMOffsetEntry.setDouble(backLLRPMOffset)
            distanceEntry.setDoubleArray(doubleArrayOf(5.0, 10.0, 15.0, 20.0))
            distFlyOffsetEntry.setDouble(0.0)
            pitchSetpointEntry.setDouble(20.0) //demo

//            rpmFlyOffsetEntry.setDouble(rpmFlyOffset)

            frontRPMOffsetEntry.setPersistent()
            backRPMOffsetEntry.setPersistent()
            frontPitchOffsetEntry.setPersistent()
            backPitchOffsetEntry.setPersistent()
            rpmSetpointEntry.setPersistent()
            useAutoOdomEntry.setPersistent()

//            pitchSetpoint = pitch  //demo
            pitchFilter.calculate(pitch)

            periodic {
                if (pitchIsReady && pitchPDEnable && !Climb.climbIsPrepped && !tuningMode) {
                    val power = pitchPDController.update(pitchFilter.calculate(pitchSetpoint) - pitch) + linearMap(-30.0, 30.0, 0.1, 0.05, pitch) // mapping (-30.0, 30.0) to (0.1, 0.05)
                    pitchSetPower(power)
//                    println("pitchPower $power")
                }
                rpmEntry.setDouble(rpm)
                rpmErrorEntry.setDouble(rpmSetpoint - rpm)
                shootModeEntry.setBoolean(shootMode)
                pitchMaxErrorEntry.setDouble(pitchMaxError)
                rpmMaxErrorEntry.setDouble(rpmMaxError)
                pitchErrorEntry.setDouble(pitchSetpoint-pitch)
                aimMaxErrorEntry.setDouble(aimMaxError)
                distFlyOffsetEntry.setDouble(distFlyOffset)

                val filteredError = rpmErrorFilter.calculate(rpmError)
                aimGood = Limelight.filteredAimError < aimMaxError
                rpmGood = filteredError < rpmMaxError
                pitchGood = (pitchSetpoint - pitch).absoluteValue < pitchMaxError
                shootMotorsGood = (rpm - shootingMotorTwo.velocity).absoluteValue < 50.0
                isCargoAlignedWithAlliance = true // (allianceColor == cargoColor || cargoColor == NOTSET) //ignore color
                allGood = shootMode && aimGood && rpmGood && pitchGood && shootMotorsGood

                aimGoodEntry.setBoolean(aimGood)
                rpmGoodEntry.setBoolean(rpmGood)
                pitchGoodEntry.setBoolean(pitchGood)
                shootMotorsGoodEntry.setBoolean(shootMotorsGood)
                allGoodEntry.setBoolean(allGood)
                colorAlignedEntry.setBoolean(isCargoAlignedWithAlliance)

                if (allGood && DriverStation.isTeleop()) {
                    OI.driverController.rumble = 0.5
                }

                 //val detectedColor: Color = m_colorSensor.color

//                println("Color = ${detectedColor.red} ${detectedColor.green} ${detectedColor.blue}")

                if (OI.operatorController.dPad == Controller.Direction.UP) {
                    upPressed = true
                } else if (OI.operatorController.dPad == Controller.Direction.DOWN) {
                    downPressed = true
                }
                if (OI.operatorController.dPad != Controller.Direction.UP && upPressed) {
                    upPressed = false
                    backLLRPMOffset += 0.01
                    println("up. hi.")
                }
                if (OI.operatorController.dPad != Controller.Direction.DOWN && downPressed) {
                    downPressed = false
                    backLLRPMOffset -= 0.01
                    println("down. hi.")
                }
                pitchEntry.setDouble(pitch)

                if (OI.operatorController.dPad == Controller.Direction.LEFT) {
                    leftPressed = true
                } else if (OI.operatorController.dPad == Controller.Direction.RIGHT) {
                    rightPressed = true
                }
                if (OI.operatorController.dPad != Controller.Direction.LEFT && leftPressed) {
                    leftPressed = false
                    frontLLRPMOffset -= 0.01
                    println("left. hi.")
                }
                if (OI.operatorController.dPad != Controller.Direction.RIGHT && rightPressed) {
                    rightPressed = false
                    frontLLRPMOffset += 0.01
                    println("right. hi.")
                }

                // adjust shot for non alliance color cargo
                stagedColorString = when (cargoColor) {
                    RED -> "red"
                    BLUE -> "blue"
                    else -> "notset"
                }
//                val rpmBadShotAdjustment = if (isCargoAlignedWithAlliance) 1.0 else if (pitch > 0) 0.4 else 0.1
                stagedColorString = "$stagedColorString $isCargoAlignedWithAlliance ${colorSensor.proximity}"
                colorEntry.setString(stagedColorString)
                // set rpm for shot
                if (isCargoAlignedWithAlliance) {
                    if  (shootMode || tuningMode) {
                        rpm = rpmSetpoint
                    } else {
                        shootingMotorOne.setPercentOutput(0.0)
                        shootingMotorTwo.setPercentOutput(0.0)
                    }
                } else {
                    if (pitch > 0) {
                        rpm = 1200.0
                    } else {
                        rpm = 1200.0
                    }
                }

            }
        }
    }

    override fun onDisable() {
        OI.operatorController.rumble = 0.0
        super.onDisable()
    }

    fun rebuildCurves() {
        backPitchCurve.setMarkBeginOrEndKeysToZeroSlope(false)
        frontPitchCurve.setMarkBeginOrEndKeysToZeroSlope(false)

        backPitchCurve.storeValue(5.0, backPitch5Entry.getDouble(-8.0))
        backPitchCurve.storeValue(10.0, backPitch10Entry.getDouble(-19.0))
        backPitchCurve.storeValue(15.0, backPitch15Entry.getDouble(-27.0))
        backPitchCurve.storeValue(20.0, backPitch20Entry.getDouble(-30.0))
        backPitchCurve.storeValue(25.0, -32.0) //lower than min //-32

        frontPitchCurve.storeValue(5.0, frontPitch5Entry.getDouble(24.8))
        frontPitchCurve.storeValue(10.0, frontPitch10Entry.getDouble(31.8))
        frontPitchCurve.storeValue(15.0, frontPitch15Entry.getDouble(36.0)) //higher than max
        frontPitchCurve.storeValue(20.0, frontPitch20Entry.getDouble(36.0)) //higher than max //36

        backRPMCurve.setMarkBeginOrEndKeysToZeroSlope(false)
        frontRPMCurve.setMarkBeginOrEndKeysToZeroSlope(false)

        backRPMCurve.storeValue(5.0, backRPM5Entry.getDouble(2800.0))
        backRPMCurve.storeValue(10.0, backRPM10Entry.getDouble(3300.0)) //3200.0)
        backRPMCurve.storeValue(15.0, backRPM15Entry.getDouble(3950.0)) //4000.0)
        backRPMCurve.storeValue(20.0, backRPM20Entry.getDouble(5200.0)) // 5100.0)
        backRPMCurve.storeValue(25.0, 5650.0)

        frontRPMCurve.storeValue(5.0, frontRPM5Entry.getDouble(3200.0))
        frontRPMCurve.storeValue(10.0, frontRPM10Entry.getDouble(3600.0)) //3900.0)
        frontRPMCurve.storeValue(15.0, frontRPM15Entry.getDouble(4550.0))
        frontRPMCurve.storeValue(20.0, frontRPM20Entry.getDouble(5800.0))
    }

    override fun preEnable() {
        shootMode = false
        allianceColor = dsAllianceColor
    }

    val cargoStageProximity : Int
        get() = colorSensor.proximity
    val cargoIsStaged : Boolean
        get() = colorSensor.proximity > PROXIMITY_STAGED_MIN
    val cargoColor : Char
        get() =  if (colorSensor.proximity < PROXIMITY_STAGED_MIN) NOTSET else if (colorSensor.color.red >= colorSensor.color.blue) RED else BLUE

    fun pitchSetPower(power: Double) {
        pitchMotor.setPercentOutput(power)
    }

    suspend fun changeAngle(angle: Double) {
        println("shooter change angle from $pitch to $angle")
        val angleCurve = MotionCurve()
        angleCurve.storeValue(0.0, pitch)
        angleCurve.storeValue((pitch - angle).absoluteValue / 30.0, angle)
        val timer = Timer()
        timer.start()
        periodic {
            val t = timer.get()
            pitch = angleCurve.getValue(t)
            if (t >= angleCurve.length) {
                stop()
            }
        }
    }

    var backLLRPMOffset: Double = 1.0
        get() = backRPMOffsetEntry.getDouble(1.0)
        set(value) {
            field = value
            backRPMOffsetEntry.setDouble(value)
        }

    var frontLLRPMOffset: Double = 1.0
        get() = frontRPMOffsetEntry.getDouble(1.0)
        set(value) {
            field = value
            frontRPMOffsetEntry.setDouble(value)
        }

    var pitchDriverOffset: Double
        get() = if (Limelight.useFrontLimelight) frontLLPitchOffset else backLLPitchOffset
        set(value) {
            if (Limelight.useFrontLimelight) {
                frontLLPitchOffset = value
                println("set frontLL to $value")
            } else {
                backLLPitchOffset = value
                println("set backLL to $value")
            }
        }

    fun changePitchDriverOffset(change: Double) {
        pitchDriverOffset = pitchDriverOffset + change
        println("setting pitchOffset to ${pitchDriverOffset}")
    }

    var backLLPitchOffset: Double = 0.0
        get() = backPitchOffsetEntry.getDouble(0.0)
        set(value) {
            field = value
            backPitchOffsetEntry.setDouble(value)
        }
    var frontLLPitchOffset: Double = 0.0
        get() = frontPitchOffsetEntry.getDouble(0.0)
        set(value) {
            field = value
            frontPitchOffsetEntry.setDouble(value)
        }

    override suspend fun default() {
        periodic {
            pitchEntry.setDouble(pitch)
        }
    }
}

suspend fun Shooter.pitchPowerTest() = use(this) {
    var power = 0.0
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
            power += 0.01
            println("up power= ${power}")
        }
        if (OI.driverController.dPad != Controller.Direction.DOWN && downPressed) {
            downPressed = false
            power -= 0.01
            println("down power= ${power}")
        }
        pitchMotor.setPercentOutput(power)
//        println("power= ${power}")
    }
}


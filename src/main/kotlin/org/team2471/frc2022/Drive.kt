package org.team2471.frc2022

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.control.PDConstantFController
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.*
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion.following.*
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.following.SwerveParameters
import org.team2471.frc.lib.units.*
import org.team2471.frc2022.OI.driverController
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin

@OptIn(DelicateCoroutinesApi::class)
object Drive : Subsystem("Drive"), SwerveDrive {

    val table = NetworkTableInstance.getDefault().getTable(name)
    val redCargoEntry = table.getSubTable("Radar").getEntry("RedCargo")
    val redFieldCargoEntry = table.getSubTable("Field").getEntry("RedCargo")
    val navXGyroEntry = table.getEntry("NavX Gyro")
    val blueCargoEntry = table.getSubTable("Radar").getEntry("BlueCargo")
    val robotsRadarEntry = table.getSubTable("Radar").getEntry("Robots")
    val robotFieldEntry = table.getSubTable("Field").getEntry("Robot")
    val robotsFieldEntry = table.getSubTable("Field").getEntry("Robots")
    val blueFieldCargoEntry = table.getSubTable("Field").getEntry("BlueCargo")
    val odometer0Entry = table.getEntry("Odometer 0")
    val odometer1Entry = table.getEntry("Odometer 1")
    val odometer2Entry = table.getEntry("Odometer 2")
    val odometer3Entry = table.getEntry("Odometer 3")

    val radialVelocityEntry = table.getEntry("Radial Velocity")
    val angularVelocityEntry = table.getEntry("Angular Velocity")
    val angleSpeedEntry = table.getEntry("angleSpeed")
    val radialSpeedEntry = table.getEntry("radialSpeed")

    val fieldObject = Field2d()
    val radarObject = Field2d()

    val limitingFactor : Double
        get() = if (Climb.climbIsPrepped) 0.25 else 1.0
    val fieldDimensions = Vector2(26.9375.feet.asMeters,54.0.feet.asMeters)
    val fieldCenterOffset = fieldDimensions/2.0
    /**
     * Coordinates of modules
     * **/
    override val modules: Array<SwerveDrive.Module> = arrayOf(
        Module(
            MotorController(FalconID(Falcons.DRIVE_FRONTLEFT)),
            MotorController(FalconID(Falcons.STEER_FRONTLEFT)),
            Vector2(-11.5, 14.0),
            (45.0).degrees,
            AnalogSensors.SWERVE_FRONT_LEFT,
            odometer0Entry
        ),
        Module(
            MotorController(FalconID(Falcons.DRIVE_FRONTRIGHT)),
            MotorController(FalconID(Falcons.STEER_FRONTRIGHT)),
            Vector2(11.5, 14.0),
            (135.0).degrees,
            AnalogSensors.SWERVE_FRONT_RIGHT,
            odometer1Entry
        ),
        Module(
            MotorController(FalconID(Falcons.DRIVE_BACKRIGHT)),
            MotorController(FalconID(Falcons.STEER_BACKRIGHT)),
            Vector2(11.5, -14.0),
            (-135.0).degrees,
            AnalogSensors.SWERVE_BACK_RIGHT,
            odometer2Entry
        ),
        Module(
            MotorController(FalconID(Falcons.DRIVE_BACKLEFT)),
            MotorController(FalconID(Falcons.STEER_BACKLEFT)),
            Vector2(-11.5, -14.0),
            (-45.0).degrees,
            AnalogSensors.SWERVE_BACK_LEFT,
            odometer3Entry
        )
    )

    //    val gyro: Gyro? = null
    private var navX: NavxWrapper = NavxWrapper()
//    private var analogDevices: ADXRS450_Gyro = ADXRS450_Gyro()

    val gyro = navX
//    val gyro = navX /*if (navXGyroEntry.getBoolean(true) && navX != null) navX else if (analogDevices != null) analogDevices else meanGyro*/

    private var gyroOffset = 0.0.degrees

    override var heading: Angle
        get() = gyroOffset - (gyro.angle.degrees.wrap())
        set(value) {
            gyroOffset = value // -gyro.angle.degrees + value
            gyro.reset()
        }

    override val headingRate: AngularVelocity
        get() = -gyro.rate.degrees.perSecond

    var autoAim: Boolean = false

    override var velocity = Vector2(0.0, 0.0)

    override var position = Vector2(0.0, -12.0)

    override var robotPivot = Vector2(0.0, 0.0)

    override var headingSetpoint = 0.0.degrees

    override val parameters: SwerveParameters = SwerveParameters(
        gyroRateCorrection = 0.0,// 0.001,
        kpPosition = 0.32,
        kdPosition = 0.6,
        kPositionFeedForward = 0.0, //075,
        kpHeading = 0.008,
        kdHeading = 0.01,
        kHeadingFeedForward = 0.001
    )

    override val carpetFlow = if (DriverStation.getAlliance() == DriverStation.Alliance.Red) Vector2(0.0, 1.0) else Vector2(0.0, -1.0)
    override val kCarpet = 0.025 // how much downstream and upstream carpet directions affect the distance, for no effect, use  0.0 (2.5% more distance downstream)
//    override val kTread: Double
//        get() = 1.0
    override val kTread = 0.04 // how much of an effect treadWear has on distance (fully worn tread goes 4% less than full tread)  0.0 for no effect

    val autoPDController = PDConstantFController(0.015, 0.04, 0.05) //0.015, 0.012, 0.008)
    val teleopPDController =  PDConstantFController(0.012, 0.09, 0.05) //0.01, 0.05, 0.05)

    var aimPDController = teleopPDController // 0.006, 0.032, 0.011  // 0.012, 0.03, 0.0
    var lastError = 0.0

    var prevRadius = 0.0
    var prevAngle = 0.0.degrees
    var lastPosition : Pose2d = Pose2d()

    val angleSpeed : Double
    get() = angleSpeedEntry.getDouble(0.0)
    val radialSpeed : Double
    get() = radialSpeedEntry.getDouble(0.0)

    val aimFlyOffsetEntry = table.getEntry("Aim Fly Offset")

       var radialVelocity = 0.0
           get() = radialVelocityEntry.getDouble(0.0)
           set(value) {
                   radialVelocityEntry.setDouble(value)
                   field = value
           }
       var angularVelocity = 0.0
          get() = angularVelocityEntry.getDouble(0.0)
           set(value) {
               angularVelocityEntry.setDouble(value)
                   field = value
           }
       val radialVelocityFilter = LinearFilter.movingAverage(2)
       val angularVelocityFilter = LinearFilter.movingAverage(2)
       var filteredRadialVelocity = 0.0
       var filteredAngularVelocity = 0.0
       var aimFlyOffset: Double = 0.0
                get() = 0.0 //aimFlyOffsetEntry.getDouble(0.0)
//            get() = filteredAngularVelocity.sign * angularVelocityCurve.getValue(filteredAngularVelocity.absoluteValue)
       val angularVelocityCurve: MotionCurve = MotionCurve()


    init {
        println("drive init")
        initializeSteeringMotors()

        if(!angleSpeedEntry.exists()) {
            angleSpeedEntry.setDouble(0.0)
            radialSpeedEntry.setDouble(0.0)
        }

        angularVelocityCurve.setMarkBeginOrEndKeysToZeroSlope(false)
        angularVelocityCurve.storeValue(0.0, 0.0)
        //        angularVelocityCurve.storeValue()
        //        angularVelocityCurve.storeValue()
        //        angularVelocityCurve.storeValue()
        //        angularVelocityCurve.storeValue()

        GlobalScope.launch(MeanlibDispatcher) {
//            odometer0Entry.setPersistent()
//            odometer1Entry.setPersistent()
//            odometer2Entry.setPersistent()
//            odometer3Entry.setPersistent()
            println("in drive global scope")

//
            SmartDashboard.setDefaultNumber("DemoSpeed", 0.4)
//            SmartDashboard.clearPersistent("Demo Speed")



            val headingEntry = table.getEntry("Heading")
            val xEntry = table.getEntry("X")
            val yEntry = table.getEntry("Y")

//            val aimPEntry = table.getEntry("p")
//            val aimDEntry = table.getEntry("d")
            val aimErrorEntry = table.getEntry("Aim Error")
            val useGyroEntry = table.getEntry("Use Gyro")

            //val zeroGyroEntry = table.getEntry("Zero Gyro")

            SmartDashboard.setPersistent("Use Gyro")
            SmartDashboard.setPersistent("Gyro Type")
            SmartDashboard.putData("Field", fieldObject)
            SmartDashboard.setPersistent("Field")
            SmartDashboard.putData("Radar", radarObject)
            SmartDashboard.setPersistent("Radar")


            useGyroEntry.setBoolean(true)
            navXGyroEntry.setBoolean(false)

            val angleZeroEntry = table.getEntry("Swerve Angle 0")
            val angleOneEntry = table.getEntry("Swerve Angle 1")
            val angleTwoEntry = table.getEntry("Swerve Angle 2")
            val angleThreeEntry = table.getEntry("Swerve Angle 3")
            aimFlyOffsetEntry.setDouble(aimFlyOffset)

            val autoAimEntry = table.getEntry("Auto Aim")
            val defaultXYPos = doubleArrayOf(0.0,0.0)

            val robotHalfWidth = (35.0/12.0)/2.0

            val reducedField = Vector2(fieldCenterOffset.x.meters.asFeet - robotHalfWidth, fieldCenterOffset.y.meters.asFeet - robotHalfWidth)
            lastPosition = Pose2d(position.x.feet.asMeters+fieldCenterOffset.x, position.y.feet.asMeters+fieldCenterOffset.y, -Rotation2d((heading-90.0.degrees).asRadians))

//            aimPEntry.setDouble(0.015)
//            aimDEntry.setDouble(0.005)
            periodic {
                var (x, y) = position
                if (x.absoluteValue > reducedField.x || y.absoluteValue > reducedField.y ){
                    println("Coercing x inside field dimensions")
                    x = x.coerceIn(-reducedField.x, reducedField.x)
                    y = y.coerceIn(-reducedField.y, reducedField.y)
                    position = Vector2(x, y)
                }
                xEntry.setDouble(x)
                yEntry.setDouble(y)
                headingEntry.setDouble(heading.asDegrees)
                aimErrorEntry.setDouble(Limelight.aimError)
                angleZeroEntry.setDouble((modules[0] as Module).analogAngle.asDegrees)
                angleOneEntry.setDouble((modules[1] as Module).analogAngle.asDegrees)
                angleTwoEntry.setDouble((modules[2] as Module).analogAngle.asDegrees)
                angleThreeEntry.setDouble((modules[3] as Module).analogAngle.asDegrees)
//               println("XPos: ${position.x.feet} yPos: ${position.y.feet}")
//                val currRobotPose = fieldObject.robotPose


                val currRadius = position.length
                val currAngle = position.angle.radians
                radialVelocityEntry.setDouble(radialVelocity)
                angularVelocityEntry.setDouble(angularVelocity)
                radialVelocity = (currRadius - prevRadius) * 50.0
                angularVelocity = (currAngle - prevAngle).wrap().asDegrees * 50.0
                filteredRadialVelocity = radialVelocityFilter.calculate(radialVelocity)
                filteredAngularVelocity = angularVelocityFilter.calculate(angularVelocity)
                prevRadius = currRadius
                prevAngle = currAngle

                val lastRobotFieldXY = robotFieldEntry.getDoubleArray(defaultXYPos)
                val lastX = lastRobotFieldXY[0]
                val lastY = lastRobotFieldXY[1]
                if (!Shooter.shootMode &&  lastX != 0.0 && lastY != 0.0 && robotHalfWidth < lastX && lastX < fieldDimensions.x - robotHalfWidth && robotHalfWidth < lastY && lastY < fieldDimensions.y - robotHalfWidth && (lastPosition.x != lastX || lastPosition.y != lastY)) {
                    position = Vector2((lastX - fieldCenterOffset.x).meters.asFeet, (lastY - fieldCenterOffset.y).meters.asFeet)
                    lastPosition = fieldObject.robotPose
                    println("from fieldobject")
                } else {
                    val robotPose = Pose2d(
                        position.x.feet.asMeters + fieldCenterOffset.x,
                        position.y.feet.asMeters + fieldCenterOffset.y,
                        -Rotation2d((heading - 90.0.degrees).asRadians)
                    )
                    fieldObject.robotPose = robotPose
                    lastPosition = robotPose
                }

                val redCargoOnField = getFieldOffsets(redCargoEntry.getDoubleArray(emptyArray()).toDoubleArray())
                val blueCargoOnField = getFieldOffsets(blueCargoEntry.getDoubleArray(emptyArray()).toDoubleArray())
                val robotOnField = getFieldOffsets(robotsRadarEntry.getDoubleArray(emptyArray()).toDoubleArray())


                redFieldCargoEntry.setDoubleArray(redCargoOnField.toDoubleArray())
                blueFieldCargoEntry.setDoubleArray(blueCargoOnField.toDoubleArray())
                robotsFieldEntry.setDoubleArray(robotOnField.toDoubleArray())

                autoAim = Shooter.shootMode && Shooter.isKnownShot == Shooter.knownShotType.NOTSET
                // println(gyro.getNavX().pitch.degrees)

//                for (moduleCount in 0..3) {
//                    val module = (modules[moduleCount] as Module)
//                    print("Module: $moduleCount analogAngle: ${round(module.analogAngle.asDegrees, 2)}  ")
//                }
//                println()
//                aimPDController.p = aimPEntry.getDouble(0.015)
//                aimPDController.d = aimDEntry.getDouble(0.005)
            }
        }
    }

    fun getFieldOffsets(arrObjects : DoubleArray ): ArrayList<Double>{
        val sinRobot = sin(lastPosition.rotation.radians - 90.0.degrees.asRadians)
        val cosRobot = cos(lastPosition.rotation.radians - 90.0.degrees.asRadians)
        val returnArray = ArrayList<Double>()


        for (i in 0..arrObjects.size) {
            if ((i + 1) % 3 != 0)
                continue
            val cargoX = (arrObjects[i-2] - 3.0)
            val cargoY = arrObjects[i-1]
            //val cargoWithBot = Vector2((cargoX * sinRobot) + lastPosition.x, (cargoY * cosRobot) + lastPosition.y)
            val cargoWithBot = Vector2((cargoX * cosRobot - cargoY * sinRobot) + lastPosition.x, (cargoY * cosRobot + cargoX * sinRobot) + lastPosition.y)
            //val cargoWithBot = fieldObject.robotPose + Transform2d(Translation2d(redCargo[i-2], redCargo[i-1]), Rotation2d(gyro.angle.degrees.asRadians+90.0.degrees.asRadians))
            returnArray.add(cargoWithBot.x)
            returnArray.add(cargoWithBot.y)
            returnArray.add(0.0)
        }
        return returnArray
    }

    override fun preEnable() {
        super.preEnable()
        odometer0Entry.setDouble(Preferences.getDouble("odometer 0",0.0))
        odometer1Entry.setDouble(Preferences.getDouble("odometer 1",0.0))
        odometer2Entry.setDouble(Preferences.getDouble("odometer 2",0.0))
        odometer3Entry.setDouble(Preferences.getDouble("odometer 3",0.0))
        println("prefs at enable=${Preferences.getDouble("odometer 0",0.0)}")
    }

    override fun onDisable() {
        if (odometer0Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 0", odometer0Entry.getDouble(0.0))
        if (odometer1Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 1", odometer1Entry.getDouble(0.0))
        if (odometer2Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 2", odometer2Entry.getDouble(0.0))
        if (odometer3Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 3", odometer3Entry.getDouble(0.0))
        super.onDisable()
    }

    fun zeroGyro() {
        heading = 0.0.degrees
        //gyro.reset()
    }

    override suspend fun default() {
        periodic {
            var turn = 0.0
            if (OI.driveRotation.absoluteValue > 0.001) {
                turn = OI.driveRotation
            }
//            else if (Limelight.hasValidTarget && autoAim) {
//                turn = aimPDController.update(Limelight.aimError)
////                println("LimeLightAimError=${Limelight.aimError}")
//            } else if (autoAim) {
//                var error = (position.angle.radians - heading).wrap()
//                if (error.asDegrees.absoluteValue > 90.0) error = (error - 180.0.degrees).wrap()
//                turn = aimPDController.update(error.asDegrees)
//            } //demo
//            printEncoderValues()

            headingSetpoint = OI.driverController.povDirection
            if(!driverController.a) {
                drive(
                OI.driveTranslation * limitingFactor,
                turn * limitingFactor,
                SmartDashboard.getBoolean("Use Gyro", true) && !DriverStation.isAutonomous(),
                false
                )
            }
        }
    }

    fun autoSteer() {
        var turn = 0.0
        if (Limelight.hasValidTarget && (!Feeder.isAuto || !Shooter.useAutoOdomEntry.getBoolean(false))) {
            turn = aimPDController.update(Limelight.aimError)
        }
        Drive.drive(
            Vector2(0.0,0.0),
            turn,
            false
        )
    }

    fun printEncoderValues() {
        for (moduleCount in 0..3) {
            print("$moduleCount=${round((modules[moduleCount] as Module).analogAngle.asDegrees, 2)}   ")
        }
        println(" ")
    }


    fun initializeSteeringMotors() {
        for (moduleCount in 0..3) {
            val module = (modules[moduleCount] as Module)
            module.turnMotor.setRawOffset(module.analogAngle.asDegrees)
            println("Module: $moduleCount analogAngle: ${module.analogAngle}")
        }
    }

    fun resetDriveMotors() {
        for (moduleCount in 0..3) {
            val module = (modules[moduleCount] as Module)
            module.driveMotor.restoreFactoryDefaults()
            println("For module $moduleCount, drive motor's factory defaults were restored.")
        }
    }

    fun resetSteeringMotors() {
        for (moduleCount in 0..3) {
            val module = (modules[moduleCount] as Module)
            module.turnMotor.restoreFactoryDefaults()
            println("For module $moduleCount, turn motor's factory defaults were restored.")
        }
    }

    fun brakeMode() {
//        for (moduleCount in 0..3) {
//            val module = (modules[moduleCount] as Module)
//            module.driveMotor.brakeMode()
//        }
    }

    fun coastMode() {
        for (moduleCount in 0..3) {
            val module = (modules[moduleCount] as Module)
            module.driveMotor.coastMode()
        }
    }

    class Module(
        val driveMotor: MotorController,
        val turnMotor: MotorController,
        override val modulePosition: Vector2,
        override val angleOffset: Angle,
        private val analogAnglePort: Int,
        private val odometerEntry: NetworkTableEntry
    ) : SwerveDrive.Module {
        companion object {
            private const val ANGLE_MAX = 983
            private const val ANGLE_MIN = 47

            private val P = 0.0075 //0.010
            private val D = 0.00075
        }

        override val angle: Angle
            get() = turnMotor.position.degrees

        private val analogAngleInput : AnalogInput = AnalogInput(analogAnglePort)

        val analogAngle: Angle
            get() {
                return ((((analogAngleInput.voltage - 0.0) / 5.0) * 360.0).degrees + angleOffset).wrap()
            }

        override val treadWear: Double
            get() = linearMap(0.0, 10000.0, 1.0, 0.96, odometer).coerceIn(0.96, 1.0)

        val driveCurrent: Double
            get() = driveMotor.current

        private val pdController = PDController(P, D)

        override val speed: Double
            get() = driveMotor.velocity

        val power: Double
            get() {
                return driveMotor.output
            }

        override val currDistance: Double
            get() = driveMotor.position

        override var prevDistance: Double = 0.0

        override var odometer: Double
            get() = odometerEntry.getDouble(0.0)
            set(value) { odometerEntry.setDouble(value) }

        override fun zeroEncoder() {
            driveMotor.position = 0.0
        }

        override var angleSetpoint: Angle = 0.0.degrees
            set(value) = turnMotor.setPositionSetpoint((angle + (value - angle).wrap()).asDegrees)

        override fun setDrivePower(power: Double) {
            driveMotor.setPercentOutput(power)
        }

        val error: Angle
            get() = turnMotor.closedLoopError.degrees

        init {
            turnMotor.config(20) {
                // this was from lil bois bench test of swerve
                feedbackCoefficient = 360.0 / 2048.0 / 21.65  // ~111 ticks per degree // spark max-neo 360.0 / 42.0 / 19.6 // degrees per tick
                setRawOffsetConfig(analogAngle.asDegrees)
                inverted(true)
                setSensorPhase(false)
                pid {
                    p(0.000002)
//                    d(0.0000025)
                }
//                burnSettings()
            }

            val moduleContLimit: Int = when ((driveMotor.motorID as FalconID).value) {
                Falcons.DRIVE_FRONTLEFT -> 60     //added 5 to all
                Falcons.DRIVE_FRONTRIGHT -> 60   //
                Falcons.DRIVE_BACKRIGHT -> 70     //
                Falcons.DRIVE_BACKLEFT -> 70
                else -> 55
            }

            val modulePeakLimit: Int = when ((driveMotor.motorID as FalconID).value) {
                Falcons.DRIVE_FRONTLEFT -> 65    //added 5 to all
                Falcons.DRIVE_FRONTRIGHT -> 65    //
                Falcons.DRIVE_BACKRIGHT -> 75        //
                Falcons.DRIVE_BACKLEFT -> 75
                else -> 60
            }

            driveMotor.config {
                brakeMode()
                feedbackCoefficient = 1.0 / 2048.0 / 5.857 / 1.09 * 6.25 / 8.0 // spark max-neo 1.0 / 42.0/ 5.857 / fudge factor * 8ft test 2022
                currentLimit(moduleContLimit, modulePeakLimit, 1)
                openLoopRamp(0.2) //1.0
//                burnSettings()
            }

            GlobalScope.launch {
                val table = NetworkTableInstance.getDefault().getTable(name)
                val pSwerveEntry = table.getEntry("Swerve P").apply {
                    setPersistent()
                    setDefaultDouble(0.0075)
                }
                val dSwerveEntry = table.getEntry("Swerve D").apply {
                    setPersistent()
                    setDefaultDouble(0.00075)
                }

            }

        }

        override fun driveWithDistance(angle: Angle, distance: Length) {
            driveMotor.setPositionSetpoint(distance.asFeet)
            val error = (angle - this.angle).wrap()
            pdController.update(error.asDegrees)
        }

        override fun stop() {
            driveMotor.stop()
            //turnMotor.stop()
        }
    }
}

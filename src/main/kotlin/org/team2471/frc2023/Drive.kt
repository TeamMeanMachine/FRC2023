package org.team2471.bunnybots2022

import com.ctre.phoenix.sensors.CANCoder
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
import org.team2471.frc.lib.motion.following.*
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.following.SwerveParameters
import org.team2471.frc.lib.units.*
import org.team2471.frc2023.CANCoders
import org.team2471.frc2023.Falcons
import org.team2471.frc2023.NavxWrapper
import org.team2471.frc2023.OI
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin

@OptIn(DelicateCoroutinesApi::class)
object Drive : Subsystem("Drive"), SwerveDrive {

    val table = NetworkTableInstance.getDefault().getTable(name)
    val navXGyroEntry = table.getEntry("NavX Gyro")
    val limitingFactor : Double
        get() = 1.0
    val robotFieldEntry = table.getSubTable("Field").getEntry("Robot")
    val odometer0Entry = table.getEntry("Odometer 0")
    val odometer1Entry = table.getEntry("Odometer 1")
    val odometer2Entry = table.getEntry("Odometer 2")
    val odometer3Entry = table.getEntry("Odometer 3")
    val absoluteAngle0Entry = table.getEntry("Analog Angle 0")
    val absoluteAngle1Entry = table.getEntry("Analog Angle 1")
    val absoluteAngle2Entry = table.getEntry("Analog Angle 2")
    val absoluteAngle3Entry = table.getEntry("Analog Angle 3")

    val fieldObject = Field2d()
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
            Preferences.getDouble("Angle Offset 0",0.0).degrees,
            CANCoders.CANCODER_FRONTLEFT,
            odometer0Entry,
            0
        ),
        Module(
            MotorController(FalconID(Falcons.DRIVE_FRONTRIGHT)),
            MotorController(FalconID(Falcons.STEER_FRONTRIGHT)),
            Vector2(11.5, 14.0),
            Preferences.getDouble("Angle Offset 1",0.0).degrees,
            CANCoders.CANCODER_FRONTRIGHT,
            odometer1Entry,
            1
        ),
        Module(
            MotorController(FalconID(Falcons.DRIVE_REARRIGHT)),
            MotorController(FalconID(Falcons.STEER_REARRIGHT)),
            Vector2(11.5, -14.0),
            Preferences.getDouble("Angle Offset 2",0.0).degrees,
            CANCoders.CANCODER_REARRIGHT,
            odometer2Entry,
            2
        ),
        Module(
            MotorController(FalconID(Falcons.DRIVE_REARLEFT)),
            MotorController(FalconID(Falcons.STEER_REARLEFT)),
            Vector2(-11.5, -14.0),
            Preferences.getDouble("Angle Offset 3",0.0).degrees,
            CANCoders.CANCODER_REARLEFT,
            odometer3Entry,
            3
        )
    )

    private var navX: NavxWrapper = NavxWrapper()
    val gyro = navX
    private var gyroOffset = 0.0.degrees

    override var heading: Angle
        get() = gyroOffset - (gyro.angle.degrees.wrap())
        set(value) {
            gyroOffset = value // -gyro.angle.degrees + value
            gyro.reset()
        }

    override val headingRate: AngularVelocity
        get() = -gyro.rate.degrees.perSecond

    override var velocity = Vector2(0.0, 0.0)
    override var position = Vector2(0.0, -12.0)
    override var robotPivot = Vector2(0.0, 0.0)
    override var headingSetpoint = 0.0.degrees

    override val parameters: SwerveParameters = SwerveParameters(
        gyroRateCorrection = 0.0,
        kpPosition = 0.32,
        kdPosition = 0.6,
        kPositionFeedForward = 0.0,
        kpHeading = 0.008,
        kdHeading = 0.01,
        kHeadingFeedForward = 0.001
    )

    override val carpetFlow = if (DriverStation.getAlliance() == DriverStation.Alliance.Red) Vector2(0.0, 1.0) else Vector2(0.0, -1.0)
    override val kCarpet = 0.025 // how much downstream and upstream carpet directions affect the distance, for no effect, use  0.0 (2.5% more distance downstream)
    override val kTread = 0.0//.04 // how much of an effect treadWear has on distance (fully worn tread goes 4% less than full tread)  0.0 for no effect

    val autoPDController = PDConstantFController(0.015, 0.04, 0.05)
    val teleopPDController =  PDConstantFController(0.012, 0.09, 0.05)
    var aimPDController = teleopPDController

    var lastPosition : Pose2d = Pose2d()

    init {
        println("drive init")
        initializeSteeringMotors()

        GlobalScope.launch(MeanlibDispatcher) {
//            odometer0Entry.setPersistent()
//            odometer1Entry.setPersistent()
//            odometer2Entry.setPersistent()
//            odometer3Entry.setPersistent()
            println("in drive global scope")

            val headingEntry = table.getEntry("Heading")
            val xEntry = table.getEntry("X")
            val yEntry = table.getEntry("Y")

            val aimErrorEntry = table.getEntry("Aim Error")
            val useGyroEntry = table.getEntry("Use Gyro")

            SmartDashboard.setPersistent("Use Gyro")
            SmartDashboard.setPersistent("Gyro Type")
            SmartDashboard.putData("Field", fieldObject)
            SmartDashboard.setPersistent("Field")

            useGyroEntry.setBoolean(true)
            navXGyroEntry.setBoolean(false)

            val defaultXYPos = doubleArrayOf(0.0,0.0)

            val robotHalfWidth = (35.0/12.0)/2.0

            val reducedField = Vector2(fieldCenterOffset.x.meters.asFeet - robotHalfWidth, fieldCenterOffset.y.meters.asFeet - robotHalfWidth)
            lastPosition = Pose2d(position.x.feet.asMeters+fieldCenterOffset.x, position.y.feet.asMeters+fieldCenterOffset.y, -Rotation2d((heading-90.0.degrees).asRadians))

            println("in init just before periodic")
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
                absoluteAngle0Entry.setDouble((modules[0] as Module).absoluteAngle.asDegrees)
                absoluteAngle1Entry.setDouble((modules[1] as Module).absoluteAngle.asDegrees)
                absoluteAngle2Entry.setDouble((modules[2] as Module).absoluteAngle.asDegrees)
                absoluteAngle3Entry.setDouble((modules[3] as Module).absoluteAngle.asDegrees)

                val lastRobotFieldXY = robotFieldEntry.getDoubleArray(defaultXYPos)
                val lastX = lastRobotFieldXY[0]
                val lastY = lastRobotFieldXY[1]
                if (lastX != 0.0 && lastY != 0.0 && robotHalfWidth < lastX && lastX < fieldDimensions.x - robotHalfWidth && robotHalfWidth < lastY && lastY < fieldDimensions.y - robotHalfWidth && (lastPosition.x != lastX || lastPosition.y != lastY)) {
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
        initializeSteeringMotors()
        odometer0Entry.setDouble(Preferences.getDouble("odometer 0",0.0))
        odometer1Entry.setDouble(Preferences.getDouble("odometer 1",0.0))
        odometer2Entry.setDouble(Preferences.getDouble("odometer 2",0.0))
        odometer3Entry.setDouble(Preferences.getDouble("odometer 3",0.0))
        println("prefs at enable=${Preferences.getDouble("odometer 0",0.0)}")
    }
    override fun postEnable(){
        super.postEnable()
        initializeSteeringMotors()
        println("Initialized From Post Enable")
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

            printEncoderValues()

            headingSetpoint = OI.driverController.povDirection

            drive(
                OI.driveTranslation * 1.0,
                turn * 1.0,
                SmartDashboard.getBoolean("Use Gyro", true) && !DriverStation.isAutonomous(),
                false
            )
        }
    }



    fun printEncoderValues() {
//        var totalCurrent=0.0
//        var totalSpeed=0.0
//        for (module in 0..modules.size-1) {
//            totalCurrent += (modules[module] as Module).driveCurrent
//            totalSpeed += (modules[module] as Module).speed.absoluteValue
//        }
//        totalCurrent /= 4.0
//        totalSpeed /= 4.0
//        var odometryAcceleration = (totalSpeed - previousAverageSpeed).absoluteValue*50.0
//        val accel = Vector2(navX.getNavX().worldLinearAccelX.toDouble(), navX.getNavX().worldLinearAccelY.toDouble())
//  //      println("Accel=${round((accel.length)*32.1,2)},${round(odometryAcceleration,2)}  Current= ${round(totalCurrent,2)}   Speed= ${round(totalSpeed,2) } Current to Speed= ${round(totalCurrent/totalSpeed,2)}  ")
//        navxAccelEntry.setDouble((accel.length)*32.1)
//        wheelAccelEntry.setDouble(odometryAcceleration)
//        averageCurrentEntry.setDouble(totalCurrent)
//        averageWheelSpeedEntry.setDouble(totalSpeed)
//
//        previousAverageSpeed = totalSpeed
    }

    fun initializeSteeringMotors() {
        for (moduleCount in modules.indices) { //changed to modules.indices, untested
            val module = (modules[moduleCount] as Module)
            module.turnMotor.setRawOffset(module.absoluteAngle.asDegrees)
            println("Module: $moduleCount analogAngle: ${module.absoluteAngle}")
        }
    }

    fun resetDriveMotors() {
        for (moduleCount in modules.indices) {
            val module = (modules[moduleCount] as Module)
            module.driveMotor.restoreFactoryDefaults()
            println("For module $moduleCount, drive motor's factory defaults were restored.")
        }
    }

    fun resetSteeringMotors() {
        for (moduleCount in modules.indices) {
            val module = (modules[moduleCount] as Module)
            module.turnMotor.restoreFactoryDefaults()
            println("For module $moduleCount, turn motor's factory defaults were restored.")
        }
    }

    fun brakeMode() {
        for (moduleCount in modules.indices) {
            val module = (modules[moduleCount] as Module)
            module.driveMotor.brakeMode()
        }
    }

    fun coastMode() {
        for (element in modules) { //switched from element in 0..modules.size-1, untested
            val module = (element as Module)
            module.driveMotor.coastMode()
        }
    }

    class Module(
        val driveMotor: MotorController,
        val turnMotor: MotorController,
        override val modulePosition: Vector2,
        override val angleOffset: Angle,
        canCoderID: Int,
        private val odometerEntry: NetworkTableEntry,
        val index: Int
    ) : SwerveDrive.Module {
        companion object {
            private const val ANGLE_MAX = 983
            private const val ANGLE_MIN = 47

            private val P = 0.0075 //0.010
            private val D = 0.00075
        }

        override val angle: Angle
            get() = turnMotor.position.degrees

        val canCoder : CANCoder = CANCoder(canCoderID)

        val absoluteAngle: Angle
            get() {
                return (-canCoder.absolutePosition.degrees - angleOffset).wrap()
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
            println("Drive.module.init")
            turnMotor.config(20) {
                feedbackCoefficient = 360.0 / 2048.0 / 21.451  // ~111 ticks per degree // spark max-neo 360.0 / 42.0 / 19.6 // degrees per tick
                setRawOffsetConfig(absoluteAngle.asDegrees)
                inverted(false)
                setSensorPhase(true)
                pid {
                    p(0.000002)
//                    d(0.0000025)
                }
            }

//            val moduleContLimit: Int = when ((driveMotor.motorID as FalconID).value) {
//                Falcons.DRIVE_FRONTLEFT -> 60     //added 5 to all
//                Falcons.DRIVE_FRONTRIGHT -> 60
//                Falcons.DRIVE_BACKRIGHT -> 70
//                Falcons.DRIVE_BACKLEFT -> 70
//                else -> 55
//            }

//            val modulePeakLimit: Int = when ((driveMotor.motorID as FalconID).value) {
//                Falcons.DRIVE_FRONTLEFT -> 65    //added 5 to all
//                Falcons.DRIVE_FRONTRIGHT -> 65
//                Falcons.DRIVE_BACKRIGHT -> 75
//                Falcons.DRIVE_BACKLEFT -> 75
//                else -> 60
//            }

            driveMotor.config {
                brakeMode()
                feedbackCoefficient = 1.0 / 2048.0 / 5.857 / 1.09 * 6.25 / 8.0 // spark max-neo 1.0 / 42.0/ 5.857 / fudge factor * 8ft test 2022
                currentLimit(70, 75, 1)
                openLoopRamp(0.2)
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
        }

        fun setAngleOffset() {
            Preferences.setDouble("Angle Offset $index", -canCoder.absolutePosition)
            println("Angle Offset $index = ${-canCoder.absolutePosition}")
        }
    }

    fun setAngleOffsets() {
        for (element in modules) {
            val module = (element as Module)
            module.setAngleOffset()
        }
    }
}


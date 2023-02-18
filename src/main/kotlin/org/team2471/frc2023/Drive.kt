package org.team2471.frc2023

import com.ctre.phoenix.sensors.CANCoder
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
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller //Added by Jeremy on 1-30-23 for power testing
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion.following.*
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.motion_profiling.following.SwerveParameters
import org.team2471.frc.lib.units.*
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

@OptIn(DelicateCoroutinesApi::class)
object Drive : Subsystem("Drive"), SwerveDrive {
    val robotHalfWidth = (32.0/2.0).inches
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
    val motorAngle0Entry = table.getEntry("Motor Angle 0")
    val motorAngle1Entry = table.getEntry("Motor Angle 1")
    val motorAngle2Entry = table.getEntry("Motor Angle 2")
    val motorAngle3Entry = table.getEntry("Motor Angle 3")

    val motorPower0Entry = table.getEntry("Motor Power 0")
    val motorPower1Entry = table.getEntry("Motor Power 1")
    val motorPower2Entry = table.getEntry("Motor Power 2")
    val motorPower3Entry = table.getEntry("Motor Power 3")
    val useGyroEntry = table.getEntry("Use Gyro")

    val advantageSwerveStates = table.getEntry("SwerveStates")
    val advantageSwerveTargets = table.getEntry("SwerveTargets")
    val rateCurve = MotionCurve()

    val fieldObject = Field2d()
    val fieldDimensions = Vector2(26.9375.feet.asMeters,54.0.feet.asMeters)
    val fieldCenterOffset = fieldDimensions/2.0
    val stateArray : DoubleArray
        get() {
            val dblArray = DoubleArray(8)
            var i = 0
            for (mod in modules) {
                dblArray[i] = mod.speed
                i++
                dblArray[i] = mod.angle.asDegrees
                i++
            }
            return dblArray
        }
    val targetArray : DoubleArray
        get() {
            val dblArray = DoubleArray(8)
            var i = 0
            for (mod in modules) {
                dblArray[i] = (mod as Module).power
                i++
                dblArray[i] = mod.angleSetpoint.asDegrees
                i++
            }
            return dblArray
        }

    /**
     * Coordinates of modules
     * **/
    override val modules: Array<SwerveDrive.Module> = arrayOf(
        Module(
            MotorController(FalconID(Falcons.LEFT_FRONT_DRIVE)),
            MotorController(FalconID(Falcons.LEFT_FRONT_STEER)),
            Vector2(-11.5, 14.0),
            Preferences.getDouble("Angle Offset 0",-6.94).degrees,
            CANCoders.CANCODER_FRONTLEFT,
            odometer0Entry,
            0
        ),
        Module(
            MotorController(FalconID(Falcons.RIGHT_FRONT_DRIVE)),
            MotorController(FalconID(Falcons.RIGHT_FRONT_STEER)),
            Vector2(11.5, 14.0),
            Preferences.getDouble("Angle Offset 1",-252.59).degrees,
            CANCoders.CANCODER_FRONTRIGHT,
            odometer1Entry,
            1
        ),
        Module(
            MotorController(FalconID(Falcons.RIGHT_REAR_DRIVE)),
            MotorController(FalconID(Falcons.RIGHT_REAR_STEER)),
            Vector2(11.5, -14.0),
            Preferences.getDouble("Angle Offset 2",-345.23).degrees,
            CANCoders.CANCODER_REARRIGHT,
            odometer2Entry,
            2
        ),
        Module(
            MotorController(FalconID(Falcons.LEFT_REAR_DRIVE)),
            MotorController(FalconID(Falcons.LEFT_REAR_STEER)),
            Vector2(-11.5, -14.0),
            Preferences.getDouble("Angle Offset 3",-335.48).degrees,
            CANCoders.CANCODER_REARLEFT,
            odometer3Entry,
            3
        )
    )

    private var navX: NavxWrapper = NavxWrapper()
    val gyro = navX
    private var gyroOffset = 0.0.degrees

    override var heading: Angle
        get() = (gyroOffset + gyro.angle.degrees).wrap()
        set(value) {
            gyro.reset()
            gyroOffset = -gyro.angle.degrees + value
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
        kPositionFeedForward = 0.05,
        kpHeading = 0.008,
        kdHeading = 0.01,
        kHeadingFeedForward = 0.001
    )

    override val carpetFlow : Vector2
        get() {
            if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                return Vector2(0.0, 1.0)
            }
            else {
                return Vector2(0.0, -1.0)
            }
        }
    override val kCarpet = 0.0256 //0.025 // how much downstream and upstream carpet directions affect the distance, for no effect, use  0.0 (2.5% more distance downstream)
    override val kTread = 0.0 //.04 // how much of an effect treadWear has on distance (fully worn tread goes 4% less than full tread)  0.0 for no effect

    val autoPDController = PDConstantFController(0.015, 0.04, 0.05)
    val teleopPDController =  PDConstantFController(0.012, 0.09, 0.05)
    var aimPDController = teleopPDController

    var lastPosition : Pose2d = Pose2d()
    var chargeMode = false


    val isHumanDriving
        get() = OI.driveTranslation.length != 0.0 || OI.driveRotation != 0.0

    init {
        println("drive init")
        initializeSteeringMotors()

        GlobalScope.launch(MeanlibDispatcher) {
//            odometer0Entry.setPersistent()
//            odometer1Entry.setPersistent()
//            odometer2Entry.setPersistent()
//            odometer3Entry.setPersistent()
            println("in drive global scope")
//            println(${setAngleOffsets()})
            val headingEntry = table.getEntry("Heading")
            val xEntry = table.getEntry("X")
            val yEntry = table.getEntry("Y")
            val poseEntry = table.getEntry("advantageScopePose")

            val aimErrorEntry = table.getEntry("Aim Error")
            val useGyroEntry = table.getEntry("Use Gyro")

            SmartDashboard.setPersistent("Use Gyro")
            SmartDashboard.setPersistent("Gyro Type")
            SmartDashboard.putData("Field", fieldObject)
            SmartDashboard.setPersistent("Field")

            navXGyroEntry.setBoolean(true)
            rateCurve.setMarkBeginOrEndKeysToZeroSlope(false)
            rateCurve.storeValue(1.0, 2.0)  // distance, rate
            rateCurve.storeValue(8.0, 6.0)  // distance, rate

            val defaultXYPos = doubleArrayOf(0.0,0.0)

            val robotHalfWidthFeet = robotHalfWidth.asFeet

            val reducedField = Vector2(fieldCenterOffset.x.meters.asFeet - robotHalfWidthFeet, fieldCenterOffset.y.meters.asFeet - robotHalfWidthFeet)
            lastPosition = Pose2d(position.x.feet.asMeters+fieldCenterOffset.x, position.y.feet.asMeters+fieldCenterOffset.y, -Rotation2d((heading-90.0.degrees).asRadians))

            println("in init just before periodic")
            periodic {
                var (x, y) = position
//                if (x.absoluteValue > reducedField.x || y.absoluteValue > reducedField.y ){
//                    println("Coercing x inside field dimensions")
//                    x = x.coerceIn(-reducedField.x, reducedField.x)
//                    y = y.coerceIn(-reducedField.y, reducedField.y)
////                    position = Vector2(x, y)
//                }
                xEntry.setDouble(x)
                yEntry.setDouble(y)
                headingEntry.setDouble(heading.asDegrees)
                val poseWPI = FieldManager.convertTMMtoWPI(x.feet, y.feet, heading)
                poseEntry.setDoubleArray(doubleArrayOf(poseWPI.x, poseWPI.y, poseWPI.rotation.degrees))
                absoluteAngle0Entry.setDouble((modules[0] as Module).absoluteAngle.asDegrees)
                absoluteAngle1Entry.setDouble((modules[1] as Module).absoluteAngle.asDegrees)
                absoluteAngle2Entry.setDouble((modules[2] as Module).absoluteAngle.asDegrees)
                absoluteAngle3Entry.setDouble((modules[3] as Module).absoluteAngle.asDegrees)

                motorPower0Entry.setDouble((modules[0] as Module).driveCurrent)
                motorPower1Entry.setDouble((modules[1] as Module).driveCurrent)
                motorPower2Entry.setDouble((modules[2] as Module).driveCurrent)
                motorPower3Entry.setDouble((modules[3] as Module).driveCurrent)

                advantageSwerveStates.setDoubleArray(stateArray)
                advantageSwerveTargets.setDoubleArray(stateArray)
//                motorAngle0Entry.setDouble((modules[0] as Module).angle.wrap().asDegrees)
//                motorAngle1Entry.setDouble((modules[1] as Module).angle.wrap().asDegrees)
//                motorAngle2Entry.setDouble((modules[2] as Module).angle.wrap().asDegrees)
//                motorAngle3Entry.setDouble((modules[3] as Module).angle.wrap().asDegrees)
           //     for (moduleCount in 0..3) { //changed to modules.indices, untested
             //       val module = (modules[moduleCount] as Module)
            //    }
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
        //initializeSteeringMotors()
        odometer0Entry.setDouble(Preferences.getDouble("odometer 0",0.0))
        odometer1Entry.setDouble(Preferences.getDouble("odometer 1",0.0))
        odometer2Entry.setDouble(Preferences.getDouble("odometer 2",0.0))
        odometer3Entry.setDouble(Preferences.getDouble("odometer 3",0.0))
        println("prefs at enable=${Preferences.getDouble("odometer 0",0.0)}")
    }
    override fun postEnable(){
        super.postEnable()
       // initializeSteeringMotors()
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
            if (!useGyroEntry.exists()) {
                useGyroEntry.setBoolean(true)
            }
            val useGyro2 = useGyroEntry.getBoolean(true) && !DriverStation.isAutonomous()

            drive(
                OI.driveTranslation * 1.0,
                turn * 1.0,
                useGyro2,
                useGyro2
            )
            //println("heading=$heading useGyro=$useGyro2")
        }
    }
    fun initializeSteeringMotors() {
        for (moduleCount in 0..3) { //changed to modules.indices, untested
            val module = (modules[moduleCount] as Module)
            module.turnMotor.setRawOffset(module.absoluteAngle.asDegrees)
            println("Module: $moduleCount analogAngle: ${module.absoluteAngle}")
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
            module.turnMotor.coastMode()
        }
    }

    class Module(
        val driveMotor: MotorController,
        val turnMotor: MotorController,
        override val modulePosition: Vector2,
        override var angleOffset: Angle,
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
                feedbackCoefficient = 360.0 / 2048.0 / 21.428  // 21.451 for bunnybot with same gearing
                inverted(false)
                setSensorPhase(false)
                setRawOffsetConfig(absoluteAngle.asDegrees)
                pid {
                    p(0.000002)
//                    d(0.0000025)
                }
            }
            driveMotor.config {
                brakeMode()
                //                    wheel diam / 12 in per foot * pi / ticks / gear ratio
                feedbackCoefficient = 4.0 / 12.0 * Math.PI / 2048.0 / 5.42 * (94.0 / 96.0)
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
            val canAngle = -canCoder.absolutePosition
            Preferences.setDouble("Angle Offset $index", canAngle)
            angleOffset = canAngle.degrees
            println("Angle Offset $index = $canAngle")
        }
    }
    fun setAngleOffsets() {
        for (element in modules) {
            val module = (element as Module)
            module.setAngleOffset()
        }
        initializeSteeringMotors()
    }
    suspend fun rampTest() = use(Drive) {
        var stage=0
        val driveTimer = Timer()
        driveTimer.start()
        println("drive until not level")
        periodic {
            drive(Vector2(0.0, 0.3), 0.0, fieldCentric = false, true)
            if (gyro.getPitch() > 3.0) {
                stop()
            }
        }
        driveDistance(Vector2(0.0, 0.25), 62.0.inches)
        driveDistance(Vector2(0.0, -0.18), 2.5.inches)
        delay(0.5.seconds)
        autoBalance()
        xPose()
    }

    suspend fun driveDistance(speed: Vector2, distance: Length) = use(Drive) {
        println("Drive to the center of ramp")
        var prevPosition = position
        periodic {
            drive(speed, 0.0, fieldCentric = false, true)
            val distanceTraveled = (position - prevPosition).length.feet
            println("distance = $distanceTraveled")
            if (distanceTraveled > distance) {
                stop()
            }
        }
        drive(Vector2(0.0, 0.0), 0.0)
    }

    suspend fun autoBalance() = use(Drive) {
        val driveTimer = Timer()
        driveTimer.start()
        periodic {
            if (gyro.getPitch().absoluteValue > 2.5) {
                // constant part for a feed forward, so there's always enough power to move.
                // plus a proportional part so that the power is higher when steep and less as it flattens.
                drive(Vector2(0.0, gyro.getPitch().sign * 0.10 + gyro.getPitch() / 200.0), 0.0, fieldCentric = false)
                println("pitch = ${gyro.getPitch()}")
                if (driveTimer.get() > 0.5) {
                    drive(Vector2(0.0, 0.0), 0.0)
                    if(driveTimer.get() > 1.0) {
                        driveTimer.reset()
                    }
                }
            }
            else {
                drive(Vector2(0.0, 0.0), 0.0)
                stop()
            }
            //  println("Pitch = ${gyro.getNavX().pitch}, Time = ${driveTimer.get()}")
        }
    }
    suspend fun dynamicDriveThreeFeetY() = use(Drive){
        val newPath = Path2D("newPath")
        newPath.addEasePoint(0.0,0.0)
        newPath.addEasePoint(2.5, 1.0)
        newPath.addVector2(position)
        newPath.addPoint(position.x,position.y+3.0)
        newPath.addHeadingPoint(0.0, heading.asDegrees)
        Drive.driveAlongPath(newPath){ abortPath() }
    }

    suspend fun dynamicGoToFeeder() = use(Drive){
        val newPath = Path2D("newPath")
        newPath.addEasePoint(0.0,0.0)
        val p1 = position
        val p2 = Vector2(11.0,16.0)
        val distance = (p2 - p1).length
        val rateCurve = MotionCurve()
        rateCurve.setMarkBeginOrEndKeysToZeroSlope(false)
        rateCurve.storeValue(1.0, 2.0)  // distance, rate
        rateCurve.storeValue(8.0, 6.0)  // distance, rate
        val rate = rateCurve.getValue(distance) // ft per sec
        val time = distance / rate
        newPath.addEasePoint(time, 1.0)
        newPath.addVector2(p1)
        newPath.addVector2(p2)
        newPath.addHeadingPoint(0.0, heading.asDegrees)
        newPath.addHeadingPoint(time, 0.0)
        Drive.driveAlongPath(newPath) { abortPath() }
    }

    suspend fun dynamicGoToScore(goalPosition: Vector2) = use(Drive){
        val newPath = Path2D("newPath")
        newPath.addEasePoint(0.0,0.0)
        val p1 = position
        val p2 = Vector2(2.0,-10.0)
        val p3 = Vector2(2.0,-19.0)
        val p4 = Vector2(goalPosition.x,-21.0)
        val distance = (p2 - p1).length + (p4 - p3).length + (p3 - p2).length
        val rateCurve = MotionCurve()
        rateCurve.setMarkBeginOrEndKeysToZeroSlope(false)
        rateCurve.storeValue(1.0, 2.0)  // distance, rate
        rateCurve.storeValue(8.0, 6.0)  // distance, rate
        val rate = rateCurve.getValue(distance) // ft per sec
        val time = distance / rate
        newPath.addEasePoint(time, 1.0)
        newPath.addVector2(p1)
        newPath.addVector2(p2)
        newPath.addVector2(p3)
        newPath.addVector2(p4)
        newPath.addHeadingPoint(0.0, heading.asDegrees)
        newPath.addHeadingPoint(time, 0.0)
        Drive.driveAlongPath(newPath){ abortPath() }
    }
    suspend fun gotoScoringPosition() = use(Drive) {
        val scoreNode = FieldManager.getSelectedNode()
        if (scoreNode == null) {
            println("Invalid node selected")
        }
        else {
            val newPath = Path2D("newPath")
            newPath.addEasePoint(0.0,0.0)
            newPath.addEasePoint(3.5, 1.0)
            newPath.addVector2(position)
            newPath.addVector2(scoreNode.alignPosition)
            val distance = newPath.length
            val rate = rateCurve.getValue(distance) // ft per sec
            val time = distance / rate
            newPath.addEasePoint(time, 1.0)
            newPath.addHeadingPoint(0.0, heading.asDegrees)
            println("${scoreNode.alignPosition}")
            Drive.driveAlongPath(newPath){ abortPath() }
        }
    }

    suspend fun calibrateRobotPosition() = use(Drive) {
        position = Vector2(-11.5, -21.25)
        heading= 0.0.degrees
    }
}

fun Drive.abortPath(): Boolean {
    return isHumanDriving
}

suspend fun Drive.currentTest() = use(this) {
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
            power += 0.001
        }
        if (OI.driverController.dPad != Controller.Direction.DOWN && downPressed) {
            downPressed = false
            power -= 0.001
        }
//        for (moduleCount in 0..3) {
//            val module = modules[moduleCount] as Drive.Module
//        }
//        println()
//        println("power: $power")
        var currModule = modules[0] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        currModule = modules[1] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        currModule = modules[2] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        currModule = modules[3] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        
        println("current: ${round(currModule.driveCurrent, 2)}  power: $power")
    //    val currModule2 = modules[3] as Drive.Module
      //  currModule2.driveMotor.setPercentOutput(power)
        //currModule2.turnMotor.setPositionSetpoint(0.0)
       // println("current: ${round(currModule.driveCurrent, 2)}  power: $power")

    //        drive(
//            Vector2(0.0, power),
//            0.0,
//            false
//        )
    }
}
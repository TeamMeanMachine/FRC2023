package org.team2471.frc2022

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc2022.Drive.gyro
import org.team2471.frc2022.Drive.heading
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.halt
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.*
import kotlin.math.*

@OptIn(DelicateCoroutinesApi::class)
object Limelight : Subsystem("Front Limelight") {
    private val combinedTable = NetworkTableInstance.getDefault().getTable("limelight")
    private val frontTable = NetworkTableInstance.getDefault().getTable("limelight-front")
    private val backTable = NetworkTableInstance.getDefault().getTable("limelight-back")
//    private val thresholdTable = frontTable.getSubTable("thresholds")
    private val frontXEntry = frontTable.getEntry("tx")
    private val backXEntry = backTable.getEntry("tx")
    private val frontYEntry = frontTable.getEntry("ty")
    private val backYEntry = backTable.getEntry("ty")
    private val areaEntry = frontTable.getEntry("ta")
    private var camModeEntry = frontTable.getEntry("camMode")
    private val frontLedModeEntry = frontTable.getEntry("ledMode")
    private val backLedModeEntry = backTable.getEntry("ledMode")
    private val frontTargetValidEntry = frontTable.getEntry("tv")
    private val backTargetValidEntry = backTable.getEntry("tv")

    private val currentPipelineEntry = frontTable.getEntry("getpipe")
//    private val frontCurrentPipelineEntry = frontTable.getEntry("getpipe")
//    private val backCurrentPipelineEntry = backTable.getEntry("getpipe")
    private val setPipelineEntry = frontTable.getEntry("pipeline")
//    private val frontSetPipelineEntry = frontTable.getEntry("pipeline")
//    private val backSetPipelineEntry = backTable.getEntry("pipeline")
    private val heightToDistance = MotionCurve()
    private var distanceEntry = combinedTable.getEntry("Distance")
    private var positionXEntry = combinedTable.getEntry("PositionX")
    private var positionYEntry = combinedTable.getEntry("PositionY")
    private var aimErrorEntry = combinedTable.getEntry("Aim Error")

    private var angleOffsetEntry = Limelight.frontTable.getEntry("Angle Offset Entry")

    enum class LimelightEnum {
        AUTO, BACK, FRONT
    }
    val LimelightSelected : LimelightEnum
        get() {
            return LimelightEnum.valueOf(SmartDashboard.getString("LimelightSelection/selected", "auto").uppercase())
        }
    private val LimelightChooser = SendableChooser<String?>().apply {
        setDefaultOption("AUTO", "auto")
        addOption("BACK", "back")
        addOption("FRONT", "front")
    }
    val useFrontLimelight: Boolean
    get() {
        var angleFromCenter = Drive.position.angle.radians
        var isFacingShooter = (angleFromCenter - heading).wrap().asDegrees.absoluteValue >= 90.0  //if the robot is facing toward (angleFromCenter opposite from heading), don't use front
//        println("isFacingShooter: $isFacingShooter   heading: ${heading.asDegrees.roundToInt()}    angleFromCenter: ${angleFromCenter.asDegrees.roundToInt()}     x: ${Drive.position.x.roundToInt()}     y: ${Drive.position.y.roundToInt()}")
        var returnThis = false
        if (DriverStation.isAutonomous()) {
            returnThis = false
        } else if (LimelightSelected == LimelightEnum.AUTO) {
            returnThis = isFacingShooter
        } else {
            LimelightSelected != LimelightEnum.BACK
        }
        return returnThis
    }

    val distance: Length
        get() {
            var llDistance = (9.0.feet - 30.5.inches) / (34.0.degrees + xTranslation.degrees).tan()
            var driveDistance = Drive.position.length.feet
            return if (((hasValidBackTarget && backLedEnabled) || (hasValidFrontTarget && frontLedEnabled)) && (llDistance - driveDistance) < maxPositionError.feet) {
                llDistance
            } else {
                driveDistance
            }
        }

    private val tempPIDTable = NetworkTableInstance.getDefault().getTable("fklsdajklfjsadlk;")

    private val rotationPEntry = tempPIDTable.getEntry("Rotation P").apply {
        setPersistent()
        setDefaultDouble(0.012)
    }

    private val rotationDEntry = tempPIDTable.getEntry("Rotation D").apply {
        setPersistent()
        setDefaultDouble(0.1)
    }

    private val useAutoPlaceEntry = frontTable.getEntry("Use Auto Place").apply {
        setPersistent()
        setDefaultBoolean(true)
    }

    var angleOffset: Double = 0.0
        get() = angleOffsetEntry.getDouble(0.0)
        set(value) {
            field = value
            angleOffsetEntry.setDouble(value)
        }

    val position: Vector2
        get() {
            var theta = heading.asDegrees + yTranslation
            if (useFrontLimelight) {
                theta += 180.0
            }
            return Vector2(
                ((distance.asFeet+2.25) * theta.degrees.sin()),
                ((distance.asFeet+2.25) * theta.degrees.cos())
            )
        }

    val targetAngle: Angle
        get() {
            return -gyro.angle.degrees + xTranslation.degrees
        } //verify that this changes? or is reasonablej

    val targetPoint
        get() = Vector2(
            (distance.asFeet+2.25) * sin(targetAngle.asRadians),
            (distance.asFeet+2.25) * cos(targetAngle.asRadians)
        ) + Drive.position


    var backLedEnabled = true
        get() = backLedModeEntry.getDouble(1.0) == 0.0
        set(value) {
            field = value
            backLedModeEntry.setDouble(if (value) 0.0 else 1.0)
        }

    var frontLedEnabled = true
        get() = frontLedModeEntry.getDouble(1.0) == 0.0
        set(value) {
            field = value
            frontLedModeEntry.setDouble(if (value) 0.0 else 1.0)
        }


    val xTranslation
        get() = if (useFrontLimelight) frontXEntry.getDouble(0.0) else -backXEntry.getDouble(0.0)

    val yTranslation: Double
        get() {
            return if (useFrontLimelight) frontYEntry.getDouble(0.0) else -backYEntry.getDouble(0.0)
        }

    val area
        get() = areaEntry.getDouble(0.0)

    val rotationP
        get() = rotationPEntry.getDouble(0.012)

    val rotationD
        get() = rotationDEntry.getDouble(0.1)

    var hasValidFrontTarget = false
        get() {
            return frontTargetValidEntry.getDouble(0.0) == 1.0
        }

    var hasValidBackTarget = false
        get() {
            return backTargetValidEntry.getDouble(0.0) == 1.0
        }

    var hasValidTarget = false
        get() = frontTargetValidEntry.getDouble(0.0) == 1.0 || backTargetValidEntry.getDouble(0.0) == 1.0



    var pipeline = 0.0
        get() = currentPipelineEntry.getDouble(0.0)
        set(value) {
            setPipelineEntry.setDouble(value)
            field = value
        }

    val aimError: Double
        get() {
//            if (hasValidBackTarget) {
                return -yTranslation + Limelight.angleOffset + Drive.aimFlyOffset
//            } else {
//                return based on odom
//            }
        }
    var aimFilter = LinearFilter.movingAverage(4)
    val filteredAimError
        get() = aimFilter.calculate(aimError)



    fun leftAngleOffset() {
        Limelight.angleOffset -= 0.1
    }

    fun rightAngleOffset() {
        Limelight.angleOffset += 0.1
    }
    var maxPositionError = 3.0


    init {
        backLedEnabled = false
        frontLedEnabled = false
        backLedModeEntry.setDouble(1.0)
        frontLedModeEntry.setDouble(1.0)
        heightToDistance.storeValue(33.0, 3.0)
        heightToDistance.storeValue(22.0, 7.2)
        heightToDistance.storeValue(9.6, 11.5)
        heightToDistance.storeValue(-4.1, 22.2)
        heightToDistance.storeValue(-20.0, 35.0)

        distanceEntry = combinedTable.getEntry("Distance")
        positionXEntry = combinedTable.getEntry("PositionX")
        positionYEntry = combinedTable.getEntry("PositionY")
        SmartDashboard.putData("LimelightSelection", LimelightChooser)


        //        var i = -4.1
        //        while (i < 22.5) {
        //            val tmpDistance = heightToDistance.getValue(i).feet
        //            //println("$i, ${tmpDistance.asFeet}")
        //            i += 0.5
        //        }
        GlobalScope.launch(MeanlibDispatcher) {
            aimFilter.calculate(aimError)
            var prevRightTrigger = false
            periodic {
                backLedEnabled = Shooter.shootMode && !useFrontLimelight
                frontLedEnabled = Shooter.shootMode && useFrontLimelight
                distanceEntry.setDouble(distance.asFeet)
                val savePosition = position
                positionXEntry.setDouble(savePosition.x)
                positionYEntry.setDouble(savePosition.y)
                aimErrorEntry.setDouble(aimError)

//                var leftPressed = false
//                var rightPressed = false
//
//                if (OI.operatorController.dPad == Controller.Direction.LEFT) {
//                    leftPressed = true
//                }
//
//                if (OI.operatorController.dPad == Controller.Direction.RIGHT) {
//                    rightPressed = true
//                }
//
//                if (OI.operatorController.dPad != Controller.Direction.LEFT && leftPressed) {
//                    //  leftPressed = false
//                    leftAngleOffset()
//                }
//
//                if (OI.operatorController.dPad != Controller.Direction.RIGHT && rightPressed) {
//                    //  rightPressed = false
//                    rightAngleOffset()
//                }



                if (Shooter.shootMode && hasValidTarget && !Feeder.isAuto && Shooter.aimGood && Drive.position.x.absoluteValue > 1.0 && Drive.position.y.absoluteValue > 1.0 && prevRightTrigger && OI.driveRightTrigger > 0.1) {
//                    if (!(!prevRightTrigger && OI.driveRightTrigger > 0.1)) println("${prevRightTrigger}          ${OI.driveRightTrigger > 0.1}")
                    val alpha = 0.0
                    val prev = Drive.position
//                    if (prev.distance(position) < maxPositionError) {
                        Drive.position = Drive.position * alpha + position * (1.0 - alpha)
                        println("Reset odometry based on limelight to ${Drive.position} from ${prev}. Hi.")
//                    } else {
//                        println("LL odom too far away to reset")
//                    }
                    //println("Reset odometry based on limelight to ${Drive.position} from ${prev}. Hi.")
                    if (prev.distance(Drive.position) > 5.0) {
                        println("Distance changed > 5.0....whoops .. that was probably an error. Might want to examine the logic for limelight distances?")
                    }
                }
//                println("${prevRightTrigger}          ${OI.driveRightTrigger > 0.1}")
                prevRightTrigger = OI.driveRightTrigger > 0.1
            }
        }
    }

//    fun startUp() {
//        distanceEntry = combinedTable.getEntry("Distance")
//        positionXEntry = combinedTable.getEntry("PositionX")
//        positionYEntry = combinedTable.getEntry("PositionY")
//
//        ledEnabled = true
//
//        GlobalScope.launch(MeanlibDispatcher) {
//            periodic {
//                distanceEntry.setDouble(distance.asFeet)
//                val savePosition = position
//                positionXEntry.setDouble(savePosition.x)
//                positionYEntry.setDouble(savePosition.y)
//                aimErrorEntry.setDouble(aimError)
//            }
//        }
//    }

    override suspend fun default() {
//        backLedEnabled = false
        halt()
    }

    override fun reset() {
    }

}


//suspend fun visionDrive() = use(Drive, FrontLimelight, name = "Vision Drive") {
//    val timer = Timer()
//    var prevTargetHeading = FrontLimelight.targetAngle
//    var prevTargetPoint = FrontLimelight.targetPoint
//    var prevTime = 0.0
//    timer.start()
//    val rotationPDController = PDController(rotationP, rotationD)
//    periodic {
//        val t = timer.get()
//        val dt = t - prevTime
//
//        // position error
//        val targetPoint = FrontLimelight.targetPoint * 0.5 + prevTargetPoint * 0.5
//        val positionError = targetPoint - Drive.position
//        prevTargetPoint = targetPoint
//
//        val robotHeading = heading
//        val targetHeading = if (BackLimelight.hasValidTarget) positionError.angle.radians else prevTargetHeading
//        val headingError = (targetHeading - robotHeading).wrap()
//        prevTargetHeading = targetHeading
//
//        val turnControl = rotationPDController.update(headingError.asDegrees )
//
//        // send it
//
//
//        Drive.drive(
//            OI.driveTranslation,
//            OI.driveRotation + turnControl,
//            SmartDashboard.getBoolean("Use Gyro", true) && !DriverStation.getInstance().isAutonomous)
//    }
//}
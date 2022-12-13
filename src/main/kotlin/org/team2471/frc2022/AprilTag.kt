package org.team2471.frc2022

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.PhotonCamera
import org.photonvision.PhotonUtils
import org.photonvision.targeting.PhotonTrackedTarget
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.halt
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion.following.*
import org.team2471.frc.lib.units.*
import org.team2471.frc2022.OI.driverController
import kotlin.math.absoluteValue
import kotlin.math.atan


@OptIn(DelicateCoroutinesApi::class)
object AprilTag : Subsystem("AprilTag") {
    private val photonVisionTable = NetworkTableInstance.getDefault().getTable("photonvision")
    private val tagTable = photonVisionTable.getSubTable("HD_USB_Camera")
    private val tagIdEntry = photonVisionTable.getEntry("tagid")
    private val translationDampenAmount = photonVisionTable.getEntry("Tranlsation Dampen Amount")
    private val rotationDampenAmount = photonVisionTable.getEntry("Rotation Dampen Amount")
    private val horizontalOffset = photonVisionTable.getEntry("Horizontal Offset M")
    private val distanceOffset = photonVisionTable.getEntry("Distance Offset In")
    var autoLock = false
    var lockedOn = false
    var xOffset = 0.0
    var validTarget = false
    val xErrorLinearFilter = LinearFilter.movingAverage(4)
    var xErrorAverage = 0.0
    val yawErrorLinearFilter = LinearFilter.movingAverage(4)
    var yawErrorAverage = 0.0
    var camera = PhotonCamera("HD_USB_Camera")


    //    private val thresholdTable = frontTable.getSubTable("thresholds")
    private val hasTargetEntry =  tagTable.getEntry("hasTarget")

    private val lockModeChooser = SendableChooser<String?>().apply {
        setDefaultOption("Rotation", "Rotation")
        addOption("Translation", "Translation")
        addOption("Rolation","Rolation")
        addOption("To_Target", "To_Target")
        addOption("Field_Centric", "Field_Centric")
    }

    private var last_result_time = 999999999990.0

    val lockMode: String
    get()= SmartDashboard.getString("AprilTag LockMode/selected", "Rotation")

    val tagId: Int
        get()= tagIdEntry.getNumber(0).toInt()

    val tda: Int
        get()= translationDampenAmount.getNumber(4).toInt()

    val rda: Int
        get()= rotationDampenAmount.getNumber(90).toInt()

    val hOffset: Double
        get()= horizontalOffset.getNumber(0.0).toDouble()

    val dOffset: Double
        get()= distanceOffset.getNumber(0.0).toDouble().inches.asMeters

    val isAligned: Boolean
        get() = xErrorAverage.absoluteValue < 0.15 && yawErrorAverage.absoluteValue < 2.0

//    val position: Vector2
//        get() {
//            var theta = heading.asDegrees + yTranslation
//            if (useFrontLimelight) {
//                theta += 180.0
//            }
//            return Vector2(
//                ((distance.asFeet + 2.25) * theta.degrees.sin()),
//                ((distance.asFeet + 2.25) * theta.degrees.cos())
//            )
//        }
//
//    val targetAngle: Angle
//        get() {
//            return -gyro.angle.degrees + xTranslation.degrees
//        } //verify that this changes? or is reasonablej
//
//    val targetPoint
//        get() = Vector2(
//            (distance.asFeet + 2.25) * sin(targetAngle.asRadians),
//            (distance.asFeet + 2.25) * cos(targetAngle.asRadians)
//        ) + Drive.position
//
//    val aimError: Double
//        get() {
////            if (hasValidBackTarget) {
//            return -yTranslation + Limelight.angleOffset + Drive.aimFlyOffset
////            } else {
////                return based on odom
////            }
//        }


    var maxPositionError = 3.0


    init {
        tagIdEntry.setNumber(24)
        tagIdEntry.setPersistent()

        translationDampenAmount.setNumber(4)
        rotationDampenAmount.setNumber(90)
        horizontalOffset.setNumber(0)
        distanceOffset.setNumber(48)
        println("AprilTags Initialized")

        SmartDashboard.putData("AprilTag LockMode", lockModeChooser)
//        GlobalScope.launch(MeanlibDispatcher) {
//            periodic {
//                //println(hasTargetEntry.getBoolean(false))
//                var result = camera.getLatestResult()
//                val hasTargets: Boolean = result.hasTargets()
//                if (hasTargets) {
//                    val targets: List<PhotonTrackedTarget> = result.getTargets()
//                    println(targets)
//                }
//            }
//        }
    }
    fun hasTarget(): Boolean {
        return camera.latestResult.hasTargets()
    }

    fun resetLastResult() {
        last_result_time = 999999999999999.0
    }

//

    override suspend fun default() {
        periodic {
            validTarget = false
            //println(hasTargetEntry.getBoolean(false))
            if(driverController.a) {
                resetLastResult()
            }
            var result = camera.latestResult
            val time = Timer.getFPGATimestamp()
            val latencyPose = Drive.lookupPose(time - result.latencyMillis)
            val positionDiff = Drive.pose.position - latencyPose.position
            val headingDiff = Drive.pose.heading - latencyPose.heading

            val hasTargets: Boolean = result.hasTargets()


           // println("Current Time: ${result.timestampSeconds} last time: $last_result_time")
            if (result.timestampSeconds <= last_result_time + 0.5) {
               // println("${hasTargets}")
                if (hasTargets) {

                    last_result_time = result.timestampSeconds
                    val targets: List<PhotonTrackedTarget> = result.getTargets().filter {it.fiducialId == tagId && it.poseAmbiguity < 0.2}
                    for (target in targets){
                        //println(target.fiducialId)
                        validTarget = true
                        xOffset = target.bestCameraToTarget.y/ tda
                        if (driverController.a || autoLock) {
                            val yOffset = target.bestCameraToTarget.y
                            var yawOffset = 0.0
                            var angleOffset = 0.0
                            var xDistanceError = 0.0
                            var yDistanceError = 0.0
                            val lockModeTemp = lockMode
                            if (lockModeTemp == "Rotation" || lockModeTemp == "Rolation" || lockModeTemp == "To_Target") {
                                angleOffset = -atan(yOffset/xOffset).radians.asDegrees
                                //println("Angle: $angleOffset")
                            }
                            if (lockModeTemp == "Translation" || lockModeTemp == "Rolation") {
                                xDistanceError = xOffset - dOffset
                                println("Distance: $xDistanceError")
                            }
                            if (lockModeTemp == "To_Target") {
                                xDistanceError = xOffset - dOffset
                                yawOffset = target.bestCameraToTarget.rotation.z.radians.asDegrees
                                if (yawOffset > 0) {
                                    yawOffset = 180 - yawOffset
                                } else if (yawOffset < 0) {
                                    yawOffset = -180 - yawOffset
                                }
                                //println(hOffset)
                                if (hOffset != 0.0) {
                                    yawOffset -= atan(hOffset/ dOffset).radians.asDegrees
                                    println(atan(hOffset/ dOffset).radians.asDegrees)
                                }
                                println("${xOffset}")
                            val latencyError = Drive.poseDiff(result.latencyMillis)
                                xDistanceError -= latencyError.position.length.feet.asMeters
                                angleOffset -= latencyError.heading.asDegrees
                            }
                            xErrorAverage = xErrorLinearFilter.calculate(xDistanceError)
                            yawErrorAverage = yawErrorLinearFilter.calculate(yawOffset)
                            Drive.drive(Vector2((yDistanceError + yawOffset / 60) / (tda / 2), (-xDistanceError) / (tda / 2) ), angleOffset / rda, false)
                            lockedOn = xDistanceError.absoluteValue < 0.1 && yawOffset.absoluteValue < 2.0
                            println("XDistanceError: $xDistanceError,(${xErrorAverage}), (${yawErrorAverage}) YawOffset: $yawOffset")
                            /* if (lockModeTemp == "Field_Centric") {
                             yawOffset = target.bestCameraToTarget.rotation.z.radians.asDegrees
                             if (yawOffset > 0) {
                                 yawOffset = 180 - yawOffset
                             } else if (yawOffset < 0) {
                                 yawOffset = -180 - yawOffset
                             }
                             var aprilTagOffset = Vector2((xOffset * (90 - yawOffset).degrees.cos() ), (xOffset * (90 - yawOffset).degrees.sin() ))
                             println("aprilTagOffset $aprilTagOffset")
                             yawOffset = 0.0

                         }*/
                        }
                    }
                }
                //println(targets)
            }
        }
    }

    override fun reset() {
    }

}
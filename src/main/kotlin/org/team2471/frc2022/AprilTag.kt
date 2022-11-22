package org.team2471.frc2022

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonTrackedTarget
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.halt
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion.following.drive
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.radians
import org.team2471.frc2022.OI.driverController
import kotlin.math.atan


@OptIn(DelicateCoroutinesApi::class)
object AprilTag : Subsystem("AprilTag") {
    private val photonVisionTable = NetworkTableInstance.getDefault().getTable("photonvision")
    private val tagTable = photonVisionTable.getSubTable("pi-cam")
    private val tagIdEntry = photonVisionTable.getEntry("tagid")
    private val translationDampenAmount = photonVisionTable.getEntry("Tranlsation Dampen Amount")

    var camera = PhotonCamera("pi-cam")

    //    private val thresholdTable = frontTable.getSubTable("thresholds")
    private val hasTargetEntry =  tagTable.getEntry("hasTarget")

    private val lockModeChooser = SendableChooser<String?>().apply {
        setDefaultOption("Rotation", "Rotation")
        addOption("Translation", "Translation")
        addOption("Both","Both")
        addOption("To_Target", "To_Target")
    }

    val lockMode: String
    get()= SmartDashboard.getString("AprilTag LockMode/selected", "Rotation")

    val tagId: Int
        get()= tagIdEntry.getNumber(0).toInt()

    val tda: Int
        get()= translationDampenAmount.getNumber(5).toInt()

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
        tagIdEntry.setNumber(0)
        tagIdEntry.setPersistent()

        translationDampenAmount.setNumber(5)
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

//

    override suspend fun default() {
        periodic {
            //println(hasTargetEntry.getBoolean(false))
            var result = camera.latestResult
            val hasTargets: Boolean = result.hasTargets()

            if (hasTargets) {
                val targets: List<PhotonTrackedTarget> = result.getTargets()
                for (target in targets){
                    if (target.fiducialId == tagId && driverController.a){
                        val xOffset = target.bestCameraToTarget.x
                        val yOffset = target.bestCameraToTarget.y
                        val yawOffset = target.bestCameraToTarget.rotation.z.radians.asDegrees
                        var angleOffset = 0.0
                        var xDistanceError = 0.0
                        var yDistanceError = 0.0
                        val lockModeTemp = lockMode
                        if (lockModeTemp == "Rotation" || lockModeTemp == "Both") {
                            angleOffset = -atan(yOffset/xOffset).radians.asDegrees
                            println("Angle: $angleOffset")
                        }
                        if (lockModeTemp == "Translation" || lockModeTemp == "Both") {
                            xDistanceError = xOffset - 2
                            println("Distance: $xDistanceError")
                        }
                        if (lockModeTemp == "To_Target") {
                            xDistanceError = xOffset - 1
                            yDistanceError = yOffset
                            println("Yaw offset ${round(yawOffset,2)} x offset ${round(xOffset,2)} y offset ${round(yOffset,2)}")

                        }

                        Drive.drive(Vector2(yDistanceError / (tda / 2), -xDistanceError / tda), angleOffset / 65, false)
                    }
                }
                //println(targets)
            }
        }
    }

    override fun reset() {
    }

}
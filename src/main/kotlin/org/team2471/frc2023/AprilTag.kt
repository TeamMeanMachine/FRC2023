package org.team2471.frc2023

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import java.util.*
import kotlin.math.abs

object AprilTag {
    private val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")

    private val frontCamSelectedEntry = pvTable.getEntry("Front Camera Selected")

    private val tagPoseEntry = pvTable.getEntry("tagPose")
    private val pvXEntry = pvTable.getEntry("xpos")
    private val pvYEntry = pvTable.getEntry("ypos")
    private val aprilTagFieldLayout : AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)
    private val camFront = PhotonCamera("camFront")
    private val camBack = PhotonCamera("camBack")

    private const val maxAmbiguity = 0.1

    private var robotToCamFront: Transform3d = Transform3d(
        Translation3d(6.5.inches.asMeters, -16.0.inches.asMeters, 2.0.inches.asMeters),
        Rotation3d(0.0, 18.0.degrees.asRadians, 0.0)
    )

    private var robotToCamBack = Transform3d(
        Translation3d(6.5.inches.asMeters, -16.0.inches.asMeters, 2.0.inches.asMeters),
        Rotation3d(0.0, 18.0.degrees.asRadians, 0.0)
    )

    //true means front cam
    private fun useFrontCam(): Boolean {
        return if (Drive.position.y > 0) {
            Drive.position.angle < 90 || Drive.position.angle > 270
        } else{
            90 < Drive.position.angle && Drive.position.angle < 270
        }
    }

    private fun customEstimatedPose(useFrontCam: Boolean): EstimatedRobotPose?{
        val cameraResult = if (useFrontCam) {
            camFront.latestResult
        } else {
            camBack.latestResult
        }

        if (!cameraResult.hasTargets()) {
            return null
        }
        val bestResult = cameraResult.bestTarget
        if (bestResult.poseAmbiguity >= maxAmbiguity) {
            return null
        }
        val targetPosition: Optional<Pose3d> = aprilTagFieldLayout.getTagPose(bestResult.fiducialId)

        if (targetPosition.isEmpty) {
            return null
        }

        val robotToCam = if (useFrontCam) {
            robotToCamFront.inverse()
        } else {
            robotToCamBack.inverse()
        }

        return EstimatedRobotPose(
            targetPosition
                .get()
                .transformBy(bestResult.bestCameraToTarget.inverse())
                .transformBy(robotToCam),
            cameraResult.timestampSeconds
        )
        //val filterResults = cameraResult.getTargets().filter { it -> it.poseAmbiguity < maxAmbiguity }
    }
    init {
        GlobalScope.launch {
            periodic {
                val frontCamSelected = useFrontCam()
                frontCamSelectedEntry.setBoolean(frontCamSelected)
                val maybePose = customEstimatedPose(frontCamSelected)
                if (maybePose != null) {
                    val currentPose = maybePose.estimatedPose  //maybePose.get().estimatedPose
                    val tagX = currentPose.x
                    val tagY = currentPose.y
                    val tagRot = currentPose.rotation
                    // println("X: $tagX Y: $tagY")
                    pvXEntry.setDouble(tagX)
                    pvYEntry.setDouble(tagY)
                    tagPoseEntry.setDoubleArray(doubleArrayOf(tagX, tagY, tagRot.angle))
//                    Drive.position = Vector2(curepos.get().estimatedPose.x.meters.asFeet - (13 + 3.5/12), curepos.get().estimatedPose.y.meters.asFeet - (26 + 0.5/12))
//                    println(Vector2((26 + 0.5/12) - curepos.get().estimatedPose.x.meters.asFeet , curepos.get().estimatedPose.y.meters.asFeet - (13 + 3.5/12)))
                }
            }
        }
    }
}
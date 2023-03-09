package org.team2471.frc2023

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonPipelineResult
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.motion.following.SwerveDrive
import org.team2471.frc.lib.units.*
import kotlin.math.abs
import kotlin.math.absoluteValue


object AprilTag {
    private val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")

    private val frontCamSelectedEntry = pvTable.getEntry("Front Camera Selected")
    private val advantagePoseEntry = pvTable.getEntry("April Advantage Pose")

    private val aprilTagFieldLayout : AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)
    private val validTags : List<Int> = aprilTagFieldLayout.tags.map { it.ID }

    private var detectedDepthYEntry = pvTable.getEntry("AprilTag Y")
    private var singleTagMinYEntry = pvTable.getEntry("SingleTag Min Y in Feet")

    private var camFront: PhotonCamera? = null
    private var camBack: PhotonCamera? = null


    private var frontPoseEstimator : PhotonPoseEstimator? = null
    private var backPoseEstimator : PhotonPoseEstimator? = null
    private const val maxAmbiguity = 0.1
    private var lastPose = Pose2d(0.0,0.0, Rotation2d(0.0))
    private var lastDetectionTime = 0.0

    val lastDetection: AprilDetection
        get() = AprilDetection(lastDetectionTime, lastPose.toTMMField())

    private var robotToCamFront: Transform3d = Transform3d(
        Translation3d(5.25.inches.asMeters, -11.5.inches.asMeters, 7.5.inches.asMeters),
        Rotation3d(0.0, -11.0.degrees.asRadians, 0.0)
    )

    private var robotToCamBack = Transform3d(
        Translation3d(-5.25.inches.asMeters, -11.25.inches.asMeters, 7.5.inches.asMeters),
        Rotation3d(0.0.degrees.asRadians, -11.0.degrees.asRadians, 180.0.degrees.asRadians)
    )
    init {
        singleTagMinYEntry.setDouble((FieldManager.chargeFromCenterY + FieldManager.chargingStationDepth).asFeet)
        resetCameras()
//        frontPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE)
//        backPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE)
        GlobalScope.launch {
            periodic {

//                frontPoseEstimator.referencePose = Pose3d(Pose2d(PoseEstimator.currentPose.toWPIField(), Rotation2d(Drive.heading.asRadians)))
//                backPoseEstimator.referencePose = Pose3d(Pose2d(PoseEstimator.currentPose.toWPIField(), Rotation2d(Drive.heading.asRadians)))
                try {
                    val frontCamSelected = useFrontCam()
                    frontCamSelectedEntry.setBoolean(frontCamSelected)
                    var maybePose: Pose2d?
                    var numTarget = 0
                    if (frontCamSelected) {
                        maybePose = frontPoseEstimator?.let { camFront?.let { it1 -> getEstimatedGlobalPose(it1, it) } }
                        numTarget = camFront?.latestResult?.targets?.count() ?: 0
                    } else {
                        maybePose = backPoseEstimator?.let { camBack?.let { it1 -> getEstimatedGlobalPose(it1, it) } }
                        numTarget = camBack?.latestResult?.targets?.count() ?: 0
                    }
                    if (maybePose != null) {
    //                        println("MaybePose: $maybePose")
                        advantagePoseEntry.setDoubleArray(
                            doubleArrayOf(
                                maybePose.x,
                                maybePose.y,
                                maybePose.rotation.degrees
                            )
                        )
                        lastPose = maybePose

                        PoseEstimator.addVision(lastDetection, numTarget)
                    }
                } catch (ex:Exception) {
                    println("Error in apriltag")
                }
            }
        }
    }


    //true means front cam
    private fun useFrontCam(): Boolean {
//        println("Angle: ${Drive.heading.asDegrees}")
        return if (Drive.combinedPosition.y > 0) {
            abs(Drive.heading.asDegrees) < 90
        } else {
            abs(Drive.heading.asDegrees) > 90
        }
    }

    fun getEstimatedGlobalPose(camera: PhotonCamera, estimator: PhotonPoseEstimator): Pose2d? {
        try {
            val cameraResult: PhotonPipelineResult = camera.latestResult
        val validTargets = cameraResult.targets//.filter{ validTags.contains(it.fiducialId) && it.poseAmbiguity < maxAmbiguity }
        if (validTargets.isEmpty()) {
            // println("AprilTag: Empty Tags")
            return null
        }
        for (target in validTargets) {
            if (target.fiducialId > 8) {
         //       println("AprilTag: Invalid Tag")
                return null
            }
        }
        if (validTargets.count() < 2 && validTargets.first().poseAmbiguity > 0.05) {
        //    println("AprilTag: Pose Ambiguity too low")
            return null
        }
        //println("at least 2 valid targets found ${poseList}")
        val newPose = estimator.update(cameraResult)
//                println("newPose: $newPose")
         if (newPose?.isPresent == true) {
            val result = newPose.get()
            detectedDepthYEntry.setDouble(result.estimatedPose.toPose2d().toTMMField().y)
            if (validTargets.count() < 2 && result.estimatedPose.toPose2d().toTMMField().y.absoluteValue < singleTagMinYEntry.getDouble((FieldManager.chargeFromCenterY + FieldManager.chargingStationDepth).asFeet)) {

//                println("AprilTag: Single target too far away ${result.estimatedPose.toPose2d().toTMMField().y.absoluteValue} vs ${(FieldManager.chargeFromCenterY + FieldManager.chargingStationDepth).asFeet}")
                return null
            }
//            println("Valid target found ${validTargets.count()}")
            lastDetectionTime = result.timestampSeconds
            return result.estimatedPose.toPose2d()
        } else {
            return null
        }
        } catch (ex: Exception) {
            return null
        }
    }

    fun resetCameras() {
        try {
            if (pvTable.containsSubTable("PVFront")) {
                camFront = PhotonCamera("PVFront")
                frontPoseEstimator = PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, camFront, robotToCamFront)
            }
        } catch (ex:Exception) {
            println("Front pose failed")
        }
        try {
            if (pvTable.containsSubTable("PVBack")) {
                camBack = PhotonCamera("PVBack")
                backPoseEstimator = PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, camBack, robotToCamBack)
            }
        } catch (ex:Exception) {
            println("Back pose failed")
        }
    }
//
//    private fun customEstimatedPose(useFrontCam: Boolean): EstimatedRobotPose?{
//        val cameraResult = if (useFrontCam) {
//            camFront.latestResult
//        } else {
//            camBack.latestResult
//        }
//
//        if (!cameraResult.hasTargets()) {
//            return null
//        }
//        val bestResult = cameraResult.bestTarget
//        if (bestResult.poseAmbiguity >= maxAmbiguity) {
//            return null
//        }
//        val targetPosition: Optional<Pose3d> = aprilTagFieldLayout.getTagPose(bestResult.fiducialId)
//
//        if (targetPosition.isEmpty) {
//            return null
//        }
//
//        val robotToCam = if (useFrontCam) {
//            robotToCamFront.inverse()
//        } else {
//            robotToCamBack.inverse()
//        }
//
//        return EstimatedRobotPose(
//            targetPosition
//                .get()
//                .transformBy(bestResult.bestCameraToTarget.inverse())
//                .transformBy(robotToCam),
//            cameraResult.timestampSeconds,
//            cameraResult.targets
//        )
//        //val filterResults = cameraResult.getTargets().filter { it -> it.poseAmbiguity < maxAmbiguity }
//    }


}

data class AprilDetection (
    val timestamp: Double,
    val pose: Pose2d
)
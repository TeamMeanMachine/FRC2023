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
import org.team2471.frc.lib.units.*
import kotlin.math.abs
import kotlin.math.absoluteValue


object AprilTag {
    private val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")

    private val frontCamSelectedEntry = pvTable.getEntry("Front Camera Selected")
    private val advantagePoseEntry = pvTable.getEntry("April Advantage Pose")

    private val aprilTagFieldLayout : AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)
    private val validTags : List<Int> = aprilTagFieldLayout.tags.map { it.ID }

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
        Rotation3d(0.0.degrees.asRadians, 11.0.degrees.asRadians, 180.0.degrees.asRadians)
    )
    init {
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
//        frontPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE)
//        backPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE)
        GlobalScope.launch {
            periodic {

//                frontPoseEstimator.referencePose = Pose3d(Pose2d(PoseEstimator.currentPose.toWPIField(), Rotation2d(Drive.heading.asRadians)))
//                backPoseEstimator.referencePose = Pose3d(Pose2d(PoseEstimator.currentPose.toWPIField(), Rotation2d(Drive.heading.asRadians)))
                try {
                val frontCamSelected = useFrontCam()
                frontCamSelectedEntry.setBoolean(frontCamSelected)
                val maybePose = if (frontCamSelected) {

                        frontPoseEstimator?.let { camFront?.let { it1 -> getEstimatedGlobalPose(it1, it) } }

                } else {
                        backPoseEstimator?.let { camBack?.let { it1 -> getEstimatedGlobalPose(it1, it) } }
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
                    PoseEstimator.addVision(lastDetection)
//                        val stdDevs = Matrix(Nat.N3(), Nat.N1())
//                        stdDevs.fill(0.3)
//                        MAPoseEstimator.addVisionData(listOf(TimestampedVisionUpdate(lastDetection.timestamp, FieldManager.convertTMMtoWPI(
//                            lastDetection.pose.x.feet, lastDetection.pose.y.feet,  lastDetection.pose.rotation.degrees.degrees), stdDevs)))
                }} catch (ex:Exception) {
                    println("Error in apriltag")
                }
            }
        }
    }


    //true means front cam
    private fun useFrontCam(): Boolean {
//        println("Angle: ${Drive.heading.asDegrees}")
        return if (Drive.position.y > 0) {
            abs(Drive.heading.asDegrees) < 90
        } else{
            abs(Drive.heading.asDegrees) > 90
        }
    }

    fun getEstimatedGlobalPose(camera: PhotonCamera, estimator: PhotonPoseEstimator): Pose2d? {
        try {
            val cameraResult: PhotonPipelineResult = camera.latestResult
        val validTargets = cameraResult.targets//.filter{ validTags.contains(it.fiducialId) && it.poseAmbiguity < maxAmbiguity }
        //val poseList = cameraResult.targets.map { it.poseAmbiguity }.toString()
        for (target in validTargets) {
            if (target.fiducialId > 8) {
                println("Invalid Tag")
                return null
            }
        }
        if (validTargets.isEmpty()) {
            return null
        }
        if (validTargets.count() < 2 && !(PoseEstimator.currentPose.y.absoluteValue > FieldManager.chargeFromCenterY.asFeet && validTargets.first().poseAmbiguity < 0.05)) {// || validTargets.count() != cameraResult.targets.count()) {
//            if (validTargets.count() < cameraResult.targets.count()) {
//                print("invalid target detected : ")
//                if (cameraResult.targets.any { !validTags.contains(it.fiducialId) }) {
//                    print("invalid id  ")
//                }
//                if (cameraResult.targets.any {it.poseAmbiguity >= maxAmbiguity}) {
//                    print("ambiguity too high ${poseList} ")
//                }
//                println(" ")
//            }
            return null
        }
        //println("at least 2 valid targets found ${poseList}")
        val newPose = estimator.update(cameraResult)
//                println("newPose: $newPose")
        return if (newPose?.isPresent == true) {
            val result = newPose.get()
            lastDetectionTime = result.timestampSeconds
            result.estimatedPose.toPose2d()
        } else {
            null
        }
        } catch (ex: Exception) {
            return null
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
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
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import kotlin.math.abs


object AprilTag {
    private val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")

    private val frontCamSelectedEntry = pvTable.getEntry("Front Camera Selected")
    private val advantagePoseEntry = pvTable.getEntry("April Advantage Pose")

    private val aprilTagFieldLayout : AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)
    private val validTags : List<Int> = aprilTagFieldLayout.tags.map { it.ID }
    private val camFront = PhotonCamera("PVFront")
    private val camBack = PhotonCamera("PVBack")
    private val frontPoseEstimator : PhotonPoseEstimator
    private val backPoseEstimator : PhotonPoseEstimator
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
        frontPoseEstimator = PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, camFront, robotToCamFront)
        backPoseEstimator = PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, camBack, robotToCamBack)
        GlobalScope.launch {
            periodic {
//                if (PoseEstimator.currentPose.y < 23.52083 || PoseEstimator.currentPose.y > 31.7916) {
                val frontCamSelected = useFrontCam()
                frontCamSelectedEntry.setBoolean(frontCamSelected)
                val maybePose = if (frontCamSelected) {
                    getEstimatedGlobalPose(camFront, frontPoseEstimator)
                } else {
                    getEstimatedGlobalPose(camBack, backPoseEstimator)
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
                }
//                }
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

        val cameraResult: PhotonPipelineResult = camera.latestResult
        val validTargets = cameraResult.targets//.filter{ validTags.contains(it.fiducialId) && it.poseAmbiguity < maxAmbiguity }
        //val poseList = cameraResult.targets.map { it.poseAmbiguity }.toString()

        if (validTargets.count() < 2) {// || validTargets.count() != cameraResult.targets.count()) {
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
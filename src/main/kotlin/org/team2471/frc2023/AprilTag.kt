package org.team2471.frc2023

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonPipelineResult
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.motion.following.lookupPose
import org.team2471.frc.lib.motion.following.poseDiff
import org.team2471.frc.lib.units.*
import java.util.*
import kotlin.math.abs


object AprilTag {
    private val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")

    private val frontCamSelectedEntry = pvTable.getEntry("Front Camera Selected")
    private val backcamErrorEntryY = pvTable.getEntry("backcam error entry y")
    private val backcamErrorEntryX = pvTable.getEntry("backcam error entry x")
    private val latencyOffsetEntry = pvTable.getEntry("Latency Offset")
    private val backcamlatencyadjustedErrorX = pvTable.getEntry("backcam error latency")
    private val backcamlatencyadjustedErrorX1 = pvTable.getEntry("backcam error latency 1")
    private val frontAdvantagePoseEntry = pvTable.getEntry("Front Advantage Pose")
    private val backAdvantagePoseEntry = pvTable.getEntry("Back Advantage Pose")

    private val tagPoseEntry = pvTable.getEntry("tagPose")
    private val pvXEntry = pvTable.getEntry("xpos")
    private val pvYEntry = pvTable.getEntry("ypos")
    private val aprilTagFieldLayout : AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)
    private val camFront = PhotonCamera("PVFront")
    private val camBack = PhotonCamera("PVBack")
    private val frontPoseEstimator : PhotonPoseEstimator
    private val backPoseEstimator : PhotonPoseEstimator
    private const val maxAmbiguity = 0.1
    var lastPose = Pose2d(0.0,0.0, Rotation2d(0.0))
    var lastbackdetection = 0.0

    private var robotToCamFront: Transform3d = Transform3d(
        Translation3d(5.25.inches.asMeters, -11.5.inches.asMeters, 7.0.inches.asMeters),
        Rotation3d(0.0, -11.0.degrees.asRadians, 0.0)
    )

    private var robotToCamBack = Transform3d(
        Translation3d(-5.25.inches.asMeters, -11.25.inches.asMeters, 7.0.inches.asMeters),
        Rotation3d(0.0.degrees.asRadians, 11.0.degrees.asRadians, 180.0.degrees.asRadians)
    )

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

        val cameraResult: PhotonPipelineResult = camera.getLatestResult()

        if (cameraResult.targets.count() < 2) {
            return null
        }
        val newPose = estimator.update(cameraResult)
//                println("newPose: $newPose")
        return if (newPose?.isPresent == true) {
            val result = newPose.get()
            if (camera.name == "PVBack"){
                lastbackdetection = result.timestampSeconds
            }
            result.estimatedPose.toPose2d()
        } else {
            null
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
            cameraResult.timestampSeconds,
            cameraResult.targets
        )
        //val filterResults = cameraResult.getTargets().filter { it -> it.poseAmbiguity < maxAmbiguity }
    }

    init {
        latencyOffsetEntry.setDouble(0.0)
        frontPoseEstimator = PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, camFront, robotToCamFront)
        backPoseEstimator = PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, camBack, robotToCamBack)
        GlobalScope.launch {
            periodic {
                val frontCamSelected = useFrontCam()
                frontCamSelectedEntry.setBoolean(frontCamSelected)
                val maybePoseFront = getEstimatedGlobalPose(camFront, frontPoseEstimator)
                val maybePoseBack = getEstimatedGlobalPose(camBack, backPoseEstimator)
                if (maybePoseFront != null) {
                    frontAdvantagePoseEntry.setDoubleArray(doubleArrayOf(maybePoseFront.x,maybePoseFront.y,maybePoseFront.rotation.degrees))
                }
                if (maybePoseBack != null) {
                    backAdvantagePoseEntry.setDoubleArray(doubleArrayOf(maybePoseBack.x,maybePoseBack.y,maybePoseBack.rotation.degrees))
                    lastPose = maybePoseBack
                    var poseError = lastPose.toTMMField()
                    val yError = Drive.position.y - poseError.y
                    backcamErrorEntryY.setDouble(yError)
                    val xError = Drive.position.x - poseError.x
                    backcamErrorEntryX.setDouble(xError)
                    val latency = Timer.getFPGATimestamp() -lastbackdetection
//                    println("latency is $latency")
                    try {
                        val latencyPose = Drive.lookupPose(lastbackdetection)
                        val latencyPose1 = Drive.lookupPose(lastbackdetection + latencyOffsetEntry.getDouble(0.0))
                        if (latencyPose != null) {
                            val posehistoryError = latencyPose.position.x - poseError.x
                            backcamlatencyadjustedErrorX.setDouble(posehistoryError)
//                            println("pose history error: $posehistoryError")
                        } else {
                            println("pose history is null")
                        }
                        if (latencyPose1 != null) {
                            val posehistoryError1 = latencyPose1.position.x - poseError.x
                            backcamlatencyadjustedErrorX1.setDouble(posehistoryError1)
//                            println("pose history error: $posehistoryError1")
                        }
                    } catch (ex:Error) {
                        println("error caught")
                    }


                }
            }
        }
    }
}
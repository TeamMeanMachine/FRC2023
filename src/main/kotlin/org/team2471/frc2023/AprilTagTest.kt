package org.team2471.frc2023

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.jetbrains.kotlin.com.google.common.graph.Network
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.radians

object AprilTagTest : Subsystem("AprilTagTest") {

    private val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")
    //private val llTable = NetworkTableInstance.getDefault().getTable("limelight-shoot")

    private val outputTable = NetworkTableInstance.getDefault().getTable("AprilTagTest")

    private val camTable = pvTable.getSubTable("Arducam_OV9281_USB_Camera")

    val targetPoseEntry: NetworkTableEntry = camTable.getEntry("targetPose")

//    val fieldPosEntry : NetworkTableEntry = llTable.getEntry("botpose_wpiblue")

    val poseEntry: NetworkTableEntry = outputTable.getEntry("pvtest_pose")
    val llPoseEntry: NetworkTableEntry = outputTable.getEntry("lltest_pose")

    private val aprilTagFieldLayout: AprilTagFieldLayout =
        AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)

    var cam: PhotonCamera = PhotonCamera("Arducam_OV9281_USB_Camera")
    var photonPoseEstimator: PhotonPoseEstimator = PhotonPoseEstimator(
        aprilTagFieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        cam,
        Transform3d(0.0, 0.0, 0.0, Rotation3d(0.0, 0.0, 0.0))
    )

    init {
        GlobalScope.launch {
            periodic {
                var res = cam.latestResult
                val multiTagResult = res.multiTagResult
                val numTargets = res.targets.size
                val targetPose = targetPoseEntry.getDoubleArray(doubleArrayOf())
                val estimatedPose = multiTagResult.estimatedPose
                println("running")
                if (estimatedPose.isPresent) {
                    println("have estimated pose")
                    if (estimatedPose.best.x != 0.0 && estimatedPose.best.y != 0.0) {
                        println("hasbestpose")
                        poseEntry.setDoubleArray(
                            doubleArrayOf(
                                estimatedPose.best.x,
                                estimatedPose.best.y,
                                estimatedPose.best.rotation.angle
                            )
                        )
                    }
                }
            }
        }
    }

//    init {


//        GlobalScope.launch {
//            periodic {
//                var res = cam.latestResult
//                val multiTagResult = res.multiTagResult
//                val numTargets = res.targets.size
//                val targetPose = targetPoseEntry.getDoubleArray(doubleArrayOf())
//                val estimatedPose = multiTagResult.estimatedPose
//                println("running")
//                if (estimatedPose.isPresent) {
//                    println("have estimated pose")
//                    if (estimatedPose.best.x != 0.0 && estimatedPose.best.y != 0.0) {
//                        println("hasbestpose")
//                        poseEntry.setDoubleArray(
//                            doubleArrayOf(
//                                estimatedPose.best.x,
//                                estimatedPose.best.y,
//                                estimatedPose.best.rotation.angle
//                            )
//                        )
//                    }
//                } else if (res.hasTargets()) {
//                    var camToTargetTrans = res.bestTarget.bestCameraToTarget;
//                    val id = res.bestTarget.fiducialId
//                    var camPose = aprilTagFieldLayout.getTagPose(id).get().transformBy(camToTargetTrans.inverse());
//                    poseEntry.setDoubleArray(doubleArrayOf(camPose.x, camPose.y, camPose.rotation.angle))
//                }
//
//                val botpose = fieldPoseEntry.getDoubleArray(doubleArrayOf())
//                if (botpose.size != 0) {
//                    llPoseEntry.setDoubleArray(doubleArrayOf(botpose[0], botpose[1], botpose[5].degrees.asRadians))
//                }
//
//            }
//        }
//    }

    override suspend fun default() {
        println("inside apriltagtest default")
        periodic{
            println("inside default")
//            var res = cam.latestResult
//            val multiTagResult = res.multiTagResult
//            val numTargets = res.targets.size
//            val targetPose = targetPoseEntry.getDoubleArray(doubleArrayOf())
//            val estimatedPose = multiTagResult.estimatedPose
//            println("running")
//            if (estimatedPose.isPresent) {
//                println("have estimated pose")
//                if (estimatedPose.best.x != 0.0 && estimatedPose.best.y != 0.0) {
//                    println("hasbestpose")
//                    poseEntry.setDoubleArray(
//                        doubleArrayOf(
//                            estimatedPose.best.x,
//                            estimatedPose.best.y,
//                            estimatedPose.best.rotation.angle
//                        )
//                    )
//                }
//            }
        }
    }

}
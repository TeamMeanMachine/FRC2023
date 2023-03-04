package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion.following.lookupPose
import org.team2471.frc.lib.units.feet

object PoseEstimator {

    val poseTable = NetworkTableInstance.getDefault().getTable("Pose Estimator")

    private val advantagePoseEntry = poseTable.getEntry("Combined Advantage Pose")
    private val maAdvantagePoseEntry = poseTable.getEntry("MA Combined Advantage Pose")
    private val kAprilEntry = poseTable.getEntry("kApril")

    private val offsetEntry = poseTable.getEntry("Offset")

    private var offset = Vector2(0.0, 0.0)
    private var lastZeroTimestamp = 0.0
    var currentPose = Vector2(0.0, 0.0)
        get() = Drive.position - offset

    init {
        kAprilEntry.setDouble(0.25)
        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                val combinedWPIField = FieldManager.convertTMMtoWPI(currentPose.x.feet, currentPose.y.feet, Drive.heading)
                advantagePoseEntry.setDoubleArray(doubleArrayOf(combinedWPIField.x,  combinedWPIField.y, combinedWPIField.rotation.degrees))
                offsetEntry.setDoubleArray(doubleArrayOf(offset.x, offset.y))

//                val maPose = MAPoseEstimator.latestPose
//                maAdvantagePoseEntry.setDoubleArray(doubleArrayOf(maPose.x, maPose.y, maPose.rotation.degrees))

            }
        }
    }
    fun addVision(detection: AprilDetection) {
        //Ignoring Vision data if timestamp is before the last zero
        if (detection.timestamp < lastZeroTimestamp) {
//            println("Stopping...")
            return
        }

        try {
            val kApril = kAprilEntry.getDouble(0.25)
            val latencyPose = Drive.lookupPose(detection.timestamp)?.position
            if (latencyPose != null) {
                val odomDiff = Drive.position - latencyPose
                val apriltagPose = Vector2(detection.pose.x, detection.pose.y) + odomDiff
                offset = offset * (1.0 - kApril) + (Drive.position - apriltagPose) * kApril
//        println(offset)
            }
        } catch(ex:Exception) {
            println("error in vision")
        }
    }
    fun zeroOffset() {
        offset = Vector2(0.0, 0.0)
        lastZeroTimestamp = Timer.getFPGATimestamp()
    }
}
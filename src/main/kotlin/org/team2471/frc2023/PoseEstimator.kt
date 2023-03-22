package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion.following.lookupPose
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.radians

object PoseEstimator {

    val poseTable = NetworkTableInstance.getDefault().getTable("Pose Estimator")

    private val advantagePoseEntry = poseTable.getEntry("Combined Advantage Pose")
    private val maAdvantagePoseEntry = poseTable.getEntry("MA Combined Advantage Pose")
    private val kAprilEntry = poseTable.getEntry("kApril")

    private val offsetEntry = poseTable.getEntry("Offset")
    private val lastResetEntry = poseTable.getEntry("LastResetTime")
    private val startingPosEntry = poseTable.getEntry("Starting Pose")
    private val startingHeadingEntry = poseTable.getEntry("Starting Heading")
    private var offset = Vector2(0.0, 0.0)
    private var lastZeroTimestamp = 0.0
    val currentPose
        get() = Drive.position - offset

    init {
        kAprilEntry.setDouble(0.25)
        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                startingHeadingEntry.setBoolean((FieldManager.isBlueAlliance && (Drive.heading > 179.0.degrees || Drive.heading < -179.0.degrees)) || (FieldManager.isRedAlliance && Drive.heading > -1.0.degrees && Drive.heading < 1.0.degrees))
                startingPosEntry.setBoolean((FieldManager.startingPosition - Drive.combinedPosition).length < 1)
                //untested ^

                val combinedWPIField = FieldManager.convertTMMtoWPI(currentPose.x.feet, currentPose.y.feet, Drive.heading)
                advantagePoseEntry.setDoubleArray(doubleArrayOf(combinedWPIField.x,  combinedWPIField.y, combinedWPIField.rotation.degrees))
                offsetEntry.setDoubleArray(doubleArrayOf(offset.x, offset.y))

//                val maPose = MAPoseEstimator.latestPose
//                maAdvantagePoseEntry.setDoubleArray(doubleArrayOf(maPose.x, maPose.y, maPose.rotation.degrees))

            }
        }
    }
    fun addVision(detection: AprilDetection, numTarget: Int, kApril: Double? = null) {
        //Ignoring Vision data if timestamp is before the last zero
        if (detection.timestamp < (lastZeroTimestamp + 0.5)) { // || Drive.position == Vector2(0.0,0.0)) {
            println("Ignoring update during reset") // and initialization ...")
            return
        } else {
            try {
                val kAprilFinal = (kApril ?: kAprilEntry.getDouble(0.5)) * if (numTarget < 2) 0.7 else 1.0
                val latencyPose = Drive.lookupPose(detection.timestamp)?.position
                if (DriverStation.isDisabled() && latencyPose == null && FieldManager.beforeFirstEnable){
                    val apriltagPose = Vector2(detection.pose.x, detection.pose.y)
                    Drive.position = apriltagPose
                    Drive.heading = detection.pose.rotation.radians.radians
                }
                if (latencyPose != null ) {
                    val odomDiff = Drive.position - latencyPose
                    val apriltagPose = Vector2(detection.pose.x, detection.pose.y) + odomDiff
                    offset = offset * (1.0 - kAprilFinal) + (Drive.position - apriltagPose) * kAprilFinal
                    val coercedOffsetX = offset.x.coerceIn(-FieldManager.fieldHalfInFeet.x + Drive.position.x, FieldManager.fieldHalfInFeet.x + Drive.position.x)
                    val coercedOffsetY = offset.y.coerceIn(-FieldManager.fieldHalfInFeet.y + Drive.position.y, FieldManager.fieldHalfInFeet.y + Drive.position.y)
                    val coercedOffset = Vector2(coercedOffsetX, coercedOffsetY)
                    if (coercedOffset.distance(offset) > 0.0) {
                        offset = coercedOffset
                        DriverStation.reportWarning("PoseEstimator: Offset coerced onto field",false)
                        println("PoseEstimator: Offset coerced onto field")
                    }

                //        println(offset)
                }
            } catch (ex: Exception) {
                println("error in vision")
            }
        }
    }
    fun zeroOffset() {
        lastZeroTimestamp = Timer.getFPGATimestamp()
        offset = Vector2(0.0, 0.0)
        lastResetEntry.setDouble(lastZeroTimestamp)

    }
}
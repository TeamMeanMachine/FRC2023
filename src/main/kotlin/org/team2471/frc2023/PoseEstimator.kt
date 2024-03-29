package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.motion.following.lookupPose
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.radians

object PoseEstimator {

    val poseTable = NetworkTableInstance.getDefault().getTable("Pose Estimator")

    private val advantagePoseEntry = poseTable.getEntry("Combined Advantage Pose")
    private val maAdvantagePoseEntry = poseTable.getEntry("MA Combined Advantage Pose")
    private val kAprilEntry = poseTable.getEntry("kApril")
    private val kHeadingEntry = poseTable.getEntry("kHeading")

    private val offsetEntry = poseTable.getEntry("Offset")
    private val lastResetEntry = poseTable.getEntry("LastResetTime")
    private val startingPosEntry = poseTable.getEntry("Starting Pose Check")
    private val startingHeadingEntry = poseTable.getEntry("Starting Heading Check")
    private val apriltagHeadingEntry = poseTable.getEntry("Apriltag Heading")
    private var offset = Vector2(0.0, 0.0)
    private var kAprilValue: Double = 0.3
    var headingOffset = 0.0.degrees
    private var lastZeroTimestamp = 0.0
    val currentPose
        get() = Drive.position - offset
    var preEnableHadTarget = false
   // val heading
     //   get() = (Drive.heading - headingOffset).wrap()

    init {
        if(FieldManager.homeField) {
            kAprilEntry.setDouble(kAprilValue)
        }
        kHeadingEntry.setDouble(0.001)
        apriltagHeadingEntry.setDouble(0.0)
        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                startingHeadingEntry.setBoolean((FieldManager.isBlueAlliance && (Drive.heading > 179.0.degrees || Drive.heading < -179.0.degrees)) || (FieldManager.isRedAlliance && Drive.heading > -1.0.degrees && Drive.heading < 1.0.degrees))
                startingPosEntry.setBoolean((FieldManager.startingPosition - Drive.combinedPosition).length < 0.25)
                //untested ^

                val combinedWPIField = FieldManager.convertTMMtoWPI(currentPose.x.feet, currentPose.y.feet, Drive.heading)
                advantagePoseEntry.setDoubleArray(doubleArrayOf(combinedWPIField.x,  combinedWPIField.y, combinedWPIField.rotation.degrees))
                offsetEntry.setDoubleArray(doubleArrayOf(offset.x, offset.y))
                if (DriverStation.isDisabled() && FieldManager.beforeFirstEnable && !preEnableHadTarget && !Drive.demoMode){
                    Drive.position = FieldManager.startingPosition
                    Drive.heading = if (FieldManager.isBlueAlliance) 180.0.degrees else 0.0.degrees
                }
//                val maPose = MAPoseEstimator.latestPose
//                maAdvantagePoseEntry.setDoubleArray(doubleArrayOf(maPose.x, maPose.y, maPose.rotation.degrees))

            }
        }
    }
//    fun addVision(detection: AprilDetection, numTarget: Int, kApril: Double? = null) {
//        //Ignoring Vision data if timestamp is before the last zero
//        if (!Drive.demoMode) {
//            if (detection.timestamp < (lastZeroTimestamp + 0.3)) { // || Drive.position == Vector2(0.0,0.0)) {
//                println("Ignoring update during reset") // and initialization ...")
//                return
//            } else {
//                try {
//                    val kAprilFinal = ((kApril ?: (kAprilValue * if (numTarget < 2) 0.7 else 1.0)))
////                val kHeading = if (kotlin.math.abs(currentPose.y) > 15.0) kHeadingEntry.getDouble(0.001) else 0.0
//                    val latencyPose = Drive.lookupPose(detection.timestamp)
//                    if (DriverStation.isDisabled() && latencyPose == null && FieldManager.beforeFirstEnable) {
//                        val apriltagPose = Vector2(detection.pose.x, detection.pose.y)
//                        preEnableHadTarget = true
//                        Drive.position = apriltagPose
//                        Drive.heading = detection.pose.rotation.radians.radians
//                    }
//                    if (latencyPose != null) {
//                        val odomDiff = Drive.position - latencyPose.position
//                        val headingDiff = Drive.heading - latencyPose.heading
//                        val apriltagPose = Vector2(detection.pose.x, detection.pose.y) + odomDiff
//                        //val apriltagHeading = (-(detection.pose.rotation.degrees.degrees + headingDiff)).wrap180()
//                        offset = offset * (1.0 - kAprilFinal) + (Drive.position - apriltagPose) * kAprilFinal
//                        //apriltagHeadingEntry.setDouble(apriltagHeading.asDegrees)
//                        //headingOffset = headingOffset * (1.0 - kHeading) + apriltagHeading.unWrap180(Drive.heading) * kHeading
//                        val coercedOffsetX = offset.x.coerceIn(
//                            -FieldManager.fieldHalfInFeet.x + Drive.position.x,
//                            FieldManager.fieldHalfInFeet.x + Drive.position.x
//                        )
//                        val coercedOffsetY = offset.y.coerceIn(
//                            -FieldManager.fieldHalfInFeet.y + Drive.position.y,
//                            FieldManager.fieldHalfInFeet.y + Drive.position.y
//                        )
//                        val coercedOffset = Vector2(coercedOffsetX, coercedOffsetY)
//                        if (coercedOffset.distance(offset) > 0.0) {
//                            offset = coercedOffset
//                            DriverStation.reportWarning("PoseEstimator: Offset coerced onto field", false)
//                            println("PoseEstimator: Offset coerced onto field")
//                        }
//                        //println("Heading Offset: ${apriltagHeading.unWrap(Drive.heading)}")
//
//                        //        println(offset)
//                    }
//                } catch (ex: Exception) {
//                }
//            }
//        }
//    }
    fun zeroOffset() {
        lastZeroTimestamp = Timer.getFPGATimestamp()
        offset = Vector2(0.0, 0.0)
        if (FieldManager.homeField) {
            lastResetEntry.setDouble(lastZeroTimestamp)
        }
    }
}
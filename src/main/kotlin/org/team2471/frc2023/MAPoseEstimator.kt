package org.team2471.frc2023

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import java.util.*
import java.util.concurrent.locks.Lock

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.


object MAPoseEstimator {

    private const val historyLengthSecs = 0.3
    var stateStdDevs = Matrix(Nat.N3(), Nat.N1())
    private var basePose = Pose2d()

    /** Returns the latest robot pose based on drive and vision data.  */
    var latestPose = Pose2d()
        private set
    private val updates: NavigableMap<Double, PoseUpdate> = TreeMap()
    private val q = Matrix(Nat.N3(), Nat.N1())

    init {

        stateStdDevs.fill(0.1)
        for (i in 0..2) {
            q[i, 0] = stateStdDevs[i, 0] * stateStdDevs[i, 0]
        }
    }

    /** Resets the odometry to a known pose.  */
    fun resetPose(pose: Pose2d) {
        basePose = pose
        updates.clear()
        update()
    }

    /** Records a new drive movement.  */
    fun addDriveData(timestamp: Double, twist: Twist2d) {
        updates[timestamp] = PoseUpdate(twist, ArrayList<VisionUpdate>())
        update()
    }

    /** Records a new set of vision updates.  */
    fun addVisionData(visionData: List<TimestampedVisionUpdate>) {
        for (timestampedVisionUpdate in visionData) {
            val timestamp: Double = timestampedVisionUpdate.timestamp
            val visionUpdate = VisionUpdate(timestampedVisionUpdate.pose, timestampedVisionUpdate.stdDevs)
            if (updates.containsKey(timestamp) ) {
                // There was already an update at this timestamp, add to it
                val oldUpdate = updates[timestamp]
                if (oldUpdate != null) {
                    val oldVisionUpdates = oldUpdate.visionUpdates
                    oldVisionUpdates.add(visionUpdate)
                    oldVisionUpdates.sortedWith(compareBy { it.stdDevs.get(0, 0) + it.stdDevs.get(1, 0) })
                    oldVisionUpdates.reverse()
                }
            } else {
                // Insert a new update
                val prevUpdate = updates.floorEntry(timestamp)
                val nextUpdate = updates.ceilingEntry(timestamp)
                if (prevUpdate == null || nextUpdate == null) {
                    // Outside the range of existing data
                    return
                }

                // Create partial twists (prev -> vision, vision -> next)
                val twist0 = multiplyTwist(
                    nextUpdate.value.twist,
                    (timestamp - prevUpdate.key) / (nextUpdate.key - prevUpdate.key)
                )
                val twist1= multiplyTwist(
                    nextUpdate.value.twist,
                    (nextUpdate.key - timestamp) / (nextUpdate.key - prevUpdate.key)
                )

                // Add new pose updates
                val newVisionUpdates = ArrayList<VisionUpdate>()
                newVisionUpdates.add(visionUpdate)
                newVisionUpdates.sortedWith(compareBy {it.stdDevs.get(0,0) + it.stdDevs.get(1,0)})
                newVisionUpdates.reverse()
                if (twist0 != null && twist1 != null) {
                updates[timestamp] = PoseUpdate(twist0, newVisionUpdates)
                updates[nextUpdate.key] = PoseUpdate(twist1, nextUpdate.value.visionUpdates)
            }}
        }

        // Recalculate latest pose once
        update()
    }

    /** Clears old data and calculates the latest pose.  */
    private fun update() {
        // Clear old data and update base pose
        synchronized(updates) {
            while (updates.size > 1
                && updates.firstKey() < Timer.getFPGATimestamp() - historyLengthSecs
            ) {
                val (_, value) = updates.pollFirstEntry()
                basePose = value.apply(basePose, q)
            }

            // Update latest pose
            latestPose = basePose
            for ((_, value) in updates) {
                latestPose = value.apply(latestPose, q)
            }
        }
    }

    /**
     * Represents a sequential update to a pose estimate, with a twist (drive movement) and list of
     * vision updates.
     */
    data class PoseUpdate(val twist: Twist2d, var visionUpdates: ArrayList<VisionUpdate>)
        fun PoseUpdate.apply(lastPose: Pose2d, q: Matrix<N3, N1>): Pose2d {
            // Apply drive twist
            var pose = lastPose.exp(twist)

            // Apply vision updates
            for (visionUpdate in visionUpdates) {
                // Calculate Kalman gains based on std devs
                // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)
                val visionK = Matrix(Nat.N3(), Nat.N3())
                val r = DoubleArray(3)
                for (i in 0..2) {
                    r[i] = visionUpdate.stdDevs.get(i, 0) * visionUpdate.stdDevs.get(i, 0)
                }
                for (row in 0..2) {
                    if (q[row, 0] == 0.0) {
                        visionK[row, row] = 0.0
                    } else {
                        visionK[row, row] = q[row, 0] / (q[row, 0] + Math.sqrt(q[row, 0] * r[row]))
                    }
                }

                // Calculate twist between current and vision pose
                val visionTwist = pose.log(visionUpdate.pose)

                // Multiply by Kalman gain matrix
                val twistMatrix = visionK.times(VecBuilder.fill(visionTwist.dx, visionTwist.dy, visionTwist.dtheta))

                // Apply twist
                pose = pose.exp(
                    Twist2d(twistMatrix[0, 0], twistMatrix[1, 0], twistMatrix[2, 0])
                )
            }
            return pose
        }
    }

    /** Represents a single vision pose with associated standard deviations.  */
    data class VisionUpdate(val pose: Pose2d, val stdDevs: Matrix<N3, N1>)


    /** Represents a single vision pose with a timestamp and associated standard deviations.  */
    data class TimestampedVisionUpdate(
        val timestamp: Double,
        val pose: Pose2d,
        val stdDevs: Matrix<N3, N1>
    )

    fun multiplyTwist(twist: Twist2d, factor: Double): Twist2d? {
        return Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor)
    }

package org.team2471.frc2023

import edu.wpi.first.wpilibj.Timer
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.motion_profiling.Path2DPoint
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import kotlin.math.absoluteValue

data class Pose(val wristPosition: Vector2, val wristAngle: Angle, val pivotAngle: Angle) {
    companion object {
        val current: Pose
            get() = Pose(Arm.wristPosition, Intake.wristAngle, Intake.pivotAngle)
        val START_POSE = Pose(Vector2(0.0, 9.0), -90.0.degrees, -90.0.degrees)
        val GROUND_INTAKE_FRONT_CONE = if(Robot.isCompBot) Pose(Vector2(22.0, 16.0), 90.0.degrees, -90.0.degrees) else Pose(Vector2(22.0, 17.0), 90.0.degrees, -90.0.degrees)
        val GROUND_INTAKE_CONE_NEAR = Pose(Vector2(19.5, 6.0), 90.0.degrees, 0.0.degrees)
        val GROUND_INTAKE_CONE_FAR = Pose(Vector2(40.0, 12.0), 90.0.degrees, 0.0.degrees)
        val GROUND_INTAKE_FRONT_CUBE = Pose(Vector2(17.0, 16.0), 90.0.degrees, -180.0.degrees)
        val GROUND_INTAKE_CUBE_NEAR = Pose(Vector2(15.5, -9.0), 75.0.degrees, -180.0.degrees)
        val GROUND_INTAKE_CUBE_FAR = Pose(Vector2(40.0, -3.0), 75.0.degrees, -180.0.degrees)

        val BACK_LOW_SCORE_CONE_TOWARD = Pose(Vector2(-5.0, 6.0), -40.0.degrees, 0.0.degrees)
        val BACK_LOW_SCORE_CONE_AWAY = Pose(Vector2(-2.0, 9.0), -60.0.degrees, 0.0.degrees)
        val BACK_LOW_SCORE_CUBE = Pose(Vector2(-3.0, 9.0), -90.0.degrees, 0.0.degrees)

        val BACK_MIDDLE_SCORE_CONE_AWAY_MID = Pose(Vector2(-25.0, 29.0), -180.0.degrees, -180.0.degrees)
        val BACK_MIDDLE_SCORE_CONE_AWAY = Pose(Vector2(-28.5, 28.5), -180.0.degrees, -180.0.degrees)
        val BACK_MIDDLE_SCORE_CONE_TOWARD_MID = Pose(Vector2(-22.0, 20.0), -80.0.degrees, 0.0.degrees)
        val BACK_MIDDLE_SCORE_CONE_TOWARD = Pose(Vector2(-26.0, 29.75), -90.0.degrees, 0.0.degrees)
        val BACK_MIDDLE_SCORE_CUBE_MID = Pose(Vector2(-16.0, 32.0), -90.0.degrees, 0.0.degrees)
        val BACK_MIDDLE_SCORE_CUBE = Pose(Vector2(-20.0, 30.0), -90.0.degrees, 0.0.degrees)

        val BACK_HIGH_SCORE_CONE_TOWARD_MID = Pose(Vector2(-28.0, 48.0), -90.0.degrees, 0.0.degrees)
        val BACK_HIGH_SCORE_CONE_TOWARD = Pose(Vector2(-39.25, 41.5), -90.0.degrees, 0.0.degrees)
        val BACK_HIGH_SCORE_CONE_AWAY_MID = Pose(Vector2(-29.0, 45.0),-180.0.degrees, -180.0.degrees)
        val BACK_HIGH_SCORE_CONE_AWAY = if(Robot.isCompBot) Pose(Vector2(-43.0, 42.0), -180.0.degrees, -180.0.degrees) else Pose(Vector2(-42.0, 45.0), -180.0.degrees, -180.0.degrees)
        val BACK_HIGH_SCORE_CUBE_MID = Pose(Vector2(-18.0, 42.0), -90.0.degrees, 0.0.degrees)
        val BACK_HIGH_SCORE_CUBE = Pose(Vector2(-32.25, 42.0), -90.0.degrees, 0.0.degrees)

        val BACK_START_POSE = Pose(Vector2(0.0, 9.0), -92.0.degrees, 0.0.degrees)

        val FRONT_DRIVE_POSE_CENTER = Pose(Vector2(0.0, 9.0), 92.0.degrees, -90.0.degrees)
        val BACK_DRIVE_POSE_CENTER = Pose(Vector2(0.0, 9.0), -92.0.degrees, -90.0.degrees)

        val FRONT_DRIVE_POSE = Pose(Vector2(-7.5, 10.0), 92.0.degrees, -90.0.degrees)
        val BACK_DRIVE_POSE = Pose(Vector2(7.5, 10.0), -92.0.degrees, -90.0.degrees)
        val FLIP_INTAKE_TO_BACK_POSE = Pose(Vector2(-28.0, 26.0), 90.0.degrees, -90.0.degrees)
        val FLIP_INTAKE_TO_BACK_WRIST = Pose(Vector2(-28.0, 26.0), -90.0.degrees, 0.0.degrees)
        val FLIP_INTAKE_TO_FRONT_POSE = Pose(Vector2(28.0, 20.0), -90.0.degrees, -90.0.degrees)
        val FLIP_INTAKE_TO_FRONT_WRIST = Pose(Vector2(28.0, 20.0), 90.0.degrees, -180.0.degrees)
        val FLIP_FRONT_UP = Pose(Vector2(-1.0, 16.0), -90.0.degrees, -90.0.degrees)
        val FLIP_FRONT_WRIST = Pose(Vector2(-1.0, 16.0), 90.0.degrees, -90.0.degrees)
        val FLIP_BACK_UP =  Pose(Vector2(1.0, 16.0), 90.0.degrees, -90.0.degrees)
        val FLIP_BACK_WRIST =  Pose(Vector2(1.0, 16.0), -90.0.degrees, -90.0.degrees)

        val HIGH_SCORE_TO_PREFLIP
            get() = Pose(Vector2(-26.5, 42.5), current.wristAngle, current.pivotAngle)
        val MIDDLE_SCORE_CONE_TO_PREFLIP
            get() = Pose(Vector2(-29.0, 30.0), current.wristAngle, current.pivotAngle)
        val MIDDLE_SCORE_CUBE_TO_PREFLIP
            get() = Pose(Vector2(-29.0, 32.0), current.wristAngle, current.pivotAngle)

        val SCORE_TO_FLIP = Pose(Vector2(-10.0, 28.0), 90.0.degrees, -90.0.degrees)

        val GROUND_TO_DRIVE_SAFE_CUBE = Pose(Vector2(38.0, 12.0), 80.0.degrees, -180.0.degrees)
        val GROUND_TO_DRIVE_SAFE_CONE = Pose(Vector2(37.0, 24.0), 100.0.degrees, 0.0.degrees)
        val GROUND_TO_DRIVE_SAFE  = Pose(Vector2(35.0, 21.0), -90.0.degrees, -90.0.degrees)
        val GROUND_TO_DRIVE_SAFE_EMPTY = Pose(Vector2(35.0, 21.0), 90.0.degrees, -90.0.degrees)
    }

    override fun toString(): String {
        return "Pose($wristPosition, $wristAngle, $pivotAngle)"
    }
    operator fun plus(otherPose: Pose) = Pose(wristPosition + otherPose.wristPosition, wristAngle + otherPose.wristAngle, pivotAngle + otherPose.pivotAngle)
}

suspend fun animateToPose(pose: Pose, minTime: Double = 0.0, waituntilDone: Boolean = false) = use(Arm, Intake) {
  animateThroughPoses(waituntilDone, Pair(minTime, pose))
}

suspend fun animateThroughPoses(waituntilDone: Boolean = false, vararg poses: Pose) {
    animateThroughPoses(waituntilDone, *poses.map{Pair(0.0, it)}.toTypedArray())
}
suspend fun animateThroughPoses(vararg poses: Pose) {
    animateThroughPoses(false, *poses.map{Pair(0.0, it)}.toTypedArray())
}

suspend fun animateThroughPoses(vararg poses: Pair<Double, Pose>) {
    animateThroughPoses(false, *poses)
}
suspend fun animateThroughPoses(waituntilDone: Boolean = false, vararg poses: Pair<Double, Pose>) = use(Arm, Intake) {
    println("Starting animation through ${poses.size} poses")
    val path = Path2D("Path")

    val pivotRate = 350.0 // deg per second
    val wristPosRate = 120.0  //  inches per second
    val wristAngleRate = 320.0 // deg per second
    val times = ArrayList<Double>(poses.size)
    val previousPose = Pose.current
    val timeMap = HashMap<String,Double>(poses.count())
    val wristCurve = MotionCurve()
    val pivotCurve = MotionCurve()

    path.addVector2(Pose.current.wristPosition)
    pivotCurve.storeValue(0.0, previousPose.pivotAngle.asDegrees)
    wristCurve.storeValue(0.0, previousPose.wristAngle.asDegrees)

    var prevLength = 0.0
    for (i in poses.indices) {
        val pose = poses[i].second
        println("WristPos Setpoint: ${pose.wristPosition}")
        path.addVector2(pose.wristPosition)
        val minTime = poses[i].first
        val wristPosTime = (path.length - prevLength) / wristPosRate
        val wristTime = ((pose.wristAngle - previousPose.wristAngle).asDegrees.absoluteValue) / wristAngleRate
        val pivotTime = ((pose.pivotAngle - previousPose.pivotAngle).asDegrees.absoluteValue) / pivotRate
        timeMap["minTime"] = minTime
        timeMap["wristPosTime"] = wristPosTime
        timeMap["wristTime"] = wristTime
        timeMap["pivotTime"] = pivotTime
        val maxTime = timeMap.values.max()
        val secondMaxTime = timeMap.values.reversed()[1]
        val timeSavings = maxTime - secondMaxTime
        val maxName = timeMap.filter { it.value == maxTime }.keys.first()
        //val secondMaxName = timeMap.filter {it.value == secondMaxTime}.keys.first()
        println("you can save $timeSavings by tuning $maxName")
        println(" ${pose.toString()} min time: ${round(minTime, 2)},  wrist pos time: ${round(wristPosTime, 2)}, wrist time: ${round(wristTime, 2)} , pivot time: ${round(pivotTime, 2)}")
        times.add(maxTime)
        val currentTime = times.sum()
        pivotCurve.storeValue(currentTime, pose.pivotAngle.asDegrees)
        wristCurve.storeValue(currentTime, pose.wristAngle.asDegrees)
        prevLength = path.length
    }

    println("times: $times")
    val totalT = times.sum()

    path.addEasePoint(0.0,0.0)
    val pathLength = path.length
    if (pathLength > 0.0) {
        var partialLength = 0.0
        var point: Path2DPoint? = path._xyCurve.headPoint
        var i = 0
        var time = 0.0
        while (point != null && point.nextPoint != null) {
            time += times[i]
            partialLength += point.segmentLength
            point = point.nextPoint
            path.addEasePoint(time, partialLength / pathLength)
//            println("time: $time Ease: ${partialLength / pathLength}")
            i++
        }
    }
    path.addEasePoint(totalT,1.0)

    val timer = Timer()
    timer.start()
    periodic {
        val t = timer.get()
//        println("t: $t  pivot: ${Intake.pivotAngle}")
        Arm.wristPosition = path.getPosition(t)
        Intake.wristSetpoint = wristCurve.getValue(t).degrees
        Intake.pivotSetpoint = pivotCurve.getValue(t).degrees
        if (t > totalT) {
            this.stop()
        }
    }

    if (waituntilDone) {
        timer.reset()
        timer.start()
        periodic {
            println("waiting for error values shoulder: ${Arm.shoulderError.asDegrees.toInt()}   elbow: ${Arm.elbowError.asDegrees.toInt()} pivot:${Intake.pivotError.asDegrees.toInt()} wrist:${Intake.wristError.asDegrees.toInt()}}")
          if ( (Arm.shoulderError.asDegrees.absoluteValue < 10.0
                    && Arm.elbowError.asDegrees.absoluteValue < 10.0
                    && Intake.pivotError.asDegrees.absoluteValue < 15.0
                    && Intake.wristError.asDegrees.absoluteValue < 10.0)
                    || timer.get() > 1.0 ) {
              this.stop()
          }
        }
        println("waited ${timer.get()} for the animation to finish")
    }
    fun toString() {

    }
}

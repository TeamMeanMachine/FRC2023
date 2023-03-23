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
        val GROUND_INTAKE_FRONT_CONE = Pose(Vector2(18.0, 14.5), 90.0.degrees, -90.0.degrees)
        val GROUND_INTAKE_CONE_NEAR = Pose(Vector2(19.5, 6.2), 90.0.degrees, 0.0.degrees)
        val GROUND_INTAKE_CONE_FAR = Pose(Vector2(40.0, 10.25), 90.0.degrees, 0.0.degrees) //y: 10
        val GROUND_INTAKE_FRONT_CUBE = Pose(Vector2(18.0, 14.0), 90.0.degrees, -180.0.degrees)
        val GROUND_INTAKE_CUBE_NEAR = Pose(Vector2(18.0, -12.0), 75.0.degrees, -180.0.degrees)//y: -8
        val GROUND_INTAKE_CUBE_FAR = Pose(Vector2(40.0, -6.0), 75.0.degrees, -180.0.degrees) // y: -8

        val BACK_LOW_SCORE_CONE_TOWARD = Pose(Vector2(-10.0, 9.0), -60.0.degrees, 0.0.degrees)
        val BACK_LOW_SCORE_CONE_AWAY = Pose(Vector2(-10.0, 9.0), -80.0.degrees, 0.0.degrees)
        val BACK_LOW_SCORE_CUBE = Pose(Vector2(-10.0, 9.0), -90.0.degrees, 0.0.degrees)

        val BACK_MIDDLE_SCORE_CONE_AWAY_MID = Pose(Vector2(-25.0, 29.0), -180.0.degrees, -180.0.degrees)
        val BACK_MIDDLE_SCORE_CONE_AWAY = Pose(Vector2(-30.5, 29.0), -180.0.degrees, -180.0.degrees)
        val BACK_MIDDLE_SCORE_CONE_TOWARD_MID = Pose(Vector2(-22.0, 30.5), -100.0.degrees, 0.0.degrees)
        val BACK_MIDDLE_SCORE_CONE_TOWARD = Pose(Vector2(-31.5, 29.0), -100.0.degrees, 0.0.degrees)
        val BACK_MIDDLE_SCORE_CUBE = Pose(Vector2(-24.5, 30.0), -100.0.degrees, 0.0.degrees)

        val BACK_HIGH_SCORE_CONE_TOWARD_MID = Pose(Vector2(-28.0, 48.0), -100.0.degrees, 0.0.degrees)
        val BACK_HIGH_SCORE_CONE_TOWARD = Pose(Vector2(-39.25, 43.0), -100.0.degrees, 0.0.degrees)
        val BACK_HIGH_SCORE_CONE_AWAY_MID = Pose(Vector2(-29.0, 45.0), -200.0.degrees, -180.0.degrees)
        val BACK_HIGH_SCORE_CONE_AWAY = Pose(Vector2(-44.0, 44.0), -190.0.degrees, -180.0.degrees)

        val BACK_START_POSE = Pose(Vector2(0.0, 9.0), -92.0.degrees, 0.0.degrees)

        val FRONT_DRIVE_POSE_CENTER = Pose(Vector2(0.0, 9.0), 92.0.degrees, -90.0.degrees)
        val BACK_DRIVE_POSE_CENTER = Pose(Vector2(0.0, 9.0), -92.0.degrees, -90.0.degrees)

        val FRONT_DRIVE_POSE = Pose(Vector2(-7.5, 10.0), 92.0.degrees, -90.0.degrees)
        val BACK_DRIVE_POSE = Pose(Vector2(7.5, 10.0), -92.0.degrees, -90.0.degrees)
        val FLIP_INTAKE_TO_BACK_POSE = Pose(Vector2(-28.0, 26.0), 90.0.degrees, -90.0.degrees)
        val FLIP_INTAKE_TO_BACK_WRIST = Pose(Vector2(-28.0, 26.0), -90.0.degrees, 0.0.degrees)
        val FLIP_INTAKE_TO_FRONT_POSE = Pose(Vector2(28.0, 20.0), -90.0.degrees, -90.0.degrees)
        val FLIP_INTAKE_TO_FRONT_WRIST = Pose(Vector2(28.0, 20.0), 90.0.degrees, -180.0.degrees)
        val FLIP_FRONT_UP = Pose(Vector2(-2.0, 13.5), -90.0.degrees, -90.0.degrees)
        val FLIP_FRONT_WRIST = Pose(Vector2(-2.0, 13.5), 90.0.degrees, -90.0.degrees)
        val FLIP_BACK_UP =  Pose(Vector2(2.0, 13.5), 90.0.degrees, -90.0.degrees)
        val FLIP_BACK_WRIST =  Pose(Vector2(2.0, 13.5), -90.0.degrees, -90.0.degrees)

        val HIGH_SCORE_TO_PREFLIP
            get() = Pose(Vector2(-29.0, 40.0), current.wristAngle, current.pivotAngle)
        val MIDDLE_SCORE_TO_PREFLIP
            get() = Pose(Vector2(-29.0, 30.0), current.wristAngle, current.pivotAngle)
        val SCORE_TO_FLIP = Pose(Vector2(-10.0, 28.0), 90.0.degrees, -90.0.degrees)

        val GROUND_TO_DRIVE_SAFE_CUBE = Pose(Vector2(37.0, 0.0), 75.0.degrees, -180.0.degrees)
        val GROUND_TO_DRIVE_SAFE_CONE = Pose(Vector2(37.0, 0.0), 75.0.degrees, 0.0.degrees)
        val GROUND_TO_DRIVE_SAFE  = Pose(Vector2(35.0, 21.0), -90.0.degrees, -90.0.degrees)
        val GROUND_TO_DRIVE_SAFE_EMPTY = Pose(Vector2(35.0, 21.0), 90.0.degrees, -90.0.degrees)
    }

    operator fun plus(otherPose: Pose) = Pose(wristPosition + otherPose.wristPosition, wristAngle + otherPose.wristAngle, pivotAngle + otherPose.pivotAngle)
}

suspend fun animateToPose(pose: Pose, minTime: Double = 0.0) = use(Arm, Intake) {
    println("Starting Animation $pose")
    val path = Path2D("newPath")
    path.addVector2(Pose.current.wristPosition)
    path.addVector2(pose.wristPosition)
    var distance = path.length
    var rate = 65.0  //  inches per second
    var wristPosTime = distance / rate

    distance = (pose.wristAngle.asDegrees - Intake.wristAngle.asDegrees).absoluteValue
    rate = 200.0 // deg per second
    var wristTime = distance / rate

    distance = (pose.pivotAngle.asDegrees - Intake.pivotAngle.asDegrees).absoluteValue
    rate = 200.0 // deg per second
    var pivotTime = distance / rate

    val time = maxOf(wristPosTime, wristTime, pivotTime, minTime)
    println("wristPosT: ${round(wristPosTime, 1)}    wristT: ${round(wristTime, 1)}      pivotT: ${round(pivotTime, 1)}    minT: ${round(minTime, 1)}")

    path.addEasePoint(0.0,0.0)
    path.addEasePoint(time, 1.0)

    val wristCurve = MotionCurve()
    wristCurve.storeValue(0.0, Pose.current.wristAngle.asDegrees)
    wristCurve.storeValue(time, pose.wristAngle.asDegrees)

    val pivotCurve = MotionCurve()
    pivotCurve.storeValue(0.0, Pose.current.pivotAngle.asDegrees)
    pivotCurve.storeValue(time, pose.pivotAngle.asDegrees)

    val timer = Timer()
    timer.start()
    periodic {
        val t = timer.get()
        Arm.wristPosition = path.getPosition(t)
        Intake.wristSetpoint = wristCurve.getValue(t).degrees
        Intake.pivotSetpoint = pivotCurve.getValue(t).degrees
//        if (pose == Pose.FLIP_INTAKE_TO_BACK || pose == Pose.FLIP_INTAKE_TO_FRONT) println("t: ${round(t, 1)}  actualWrist: ${round(Intake.wristAngle.asDegrees, 1)} wristSetpoint: ${round(Intake.wristSetpoint.asDegrees, 1)}    actualWristPos: ${Arm.forwardKinematics(Arm.shoulderMotor.position.degrees, Arm.elbowAngle)}   wristPos: ${Arm.wristPosition}")
        if (t > time) {
            this.stop()
        }
    }
}

suspend fun animateThroughPoses(vararg poses: Pose) {
    animateThroughPoses(*poses.map{Pair(0.0, it)}.toTypedArray())
}
suspend fun animateThroughPoses(vararg poses: Pair<Double, Pose>) = use(Arm, Intake) {
    println("Starting animation through $poses")
    val path = Path2D("Path")
    path.addVector2(Pose.current.wristPosition)
    var distance = ArrayList<Double>(poses.size)
    for (pose in poses) {
        var prevLength = path.length
        path.addVector2(pose.second.wristPosition)
        distance.add(path.length - prevLength)
    }
    var rate = 65.0  //  inches per second
    var wristPosTime = ArrayList<Double>(poses.size)
    for (i in poses.indices) {
        wristPosTime.add(distance[i] / rate)
    }

    distance.clear()
    distance.add((poses[0].second.wristAngle.asDegrees - Intake.wristAngle.asDegrees).absoluteValue)
    for (i in 1..poses.indices.last) {
        distance.add((poses[i].second.wristAngle.asDegrees - poses[i-1].second.wristAngle.asDegrees).absoluteValue)
    }
    rate = 200.0 // deg per second
    var wristTime = ArrayList<Double>(poses.size)
    for (i in poses.indices) {
        wristTime.add(distance[i] / rate)
    }

    distance.clear()
    distance.add((poses[0].second.pivotAngle.asDegrees - Intake.pivotAngle.asDegrees).absoluteValue)
    for (i in 1..poses.indices.last) {
        distance.add((poses[i].second.pivotAngle.asDegrees - poses[i-1].second.pivotAngle.asDegrees).absoluteValue)
    }
    rate = 200.0 // deg per second
    var pivotTime = ArrayList<Double>(poses.size)
    for (i in poses.indices) {
        pivotTime.add(distance[i] / rate)
    }

    val times = ArrayList<Double>(poses.size)
    println("poses: ${poses[0].first}")
    println("wristPos: ${wristPosTime[0]}")
    println("wrist: ${wristTime[0]}")
    println("pivot: ${pivotTime[0]}")
    for (i in poses.indices) {
        times.add(maxOf(wristPosTime[i], wristTime[i], pivotTime[i], poses[i].first))
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

    val wristCurve = MotionCurve()
    var time = 0.0
    wristCurve.storeValue(time, Pose.current.wristAngle.asDegrees)
    for (i in poses.indices) {
        time += times[i]
        wristCurve.storeValue(time, poses[i].second.wristAngle.asDegrees)
    }
    time = 0.0
    val pivotCurve = MotionCurve()
    pivotCurve.storeValue(0.0, Pose.current.pivotAngle.asDegrees)
    for (i in poses.indices) {
        time += times[i]
        pivotCurve.storeValue(time, poses[i].second.pivotAngle.asDegrees)
    }

    val timer = Timer()
    timer.start()
    periodic {
        val t = timer.get()
        println("t: $t  pivot: ${Intake.pivotAngle}")
        Arm.wristPosition = path.getPosition(t)
        Intake.wristSetpoint = wristCurve.getValue(t).degrees
        Intake.pivotSetpoint = pivotCurve.getValue(t).degrees
        if (t > totalT) {
            this.stop()
        }
    }
}

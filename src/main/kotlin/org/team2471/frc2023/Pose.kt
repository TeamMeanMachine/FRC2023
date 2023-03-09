package org.team2471.frc2023

import edu.wpi.first.wpilibj.Timer
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import kotlin.math.absoluteValue

data class Pose(val wristPosition: Vector2, val wristAngle: Angle, val pivotAngle: Angle) {
    companion object {
        val current: Pose
            get() = Pose(Arm.wristPosition, Intake.wristAngle, Intake.pivotAngle)
        val START_POSE = Pose(Vector2(0.0, 9.0), -90.0.degrees, -90.0.degrees)
        val GROUND_INTAKE_FRONT_CONE = Pose(Vector2(18.0, 14.5), 90.0.degrees, -90.0.degrees)
        val GROUND_INTAKE_FRONT_CUBE = Pose(Vector2(18.0, 14.0), 90.0.degrees, -180.0.degrees)
        val GROUND_INTAKE_CONE_NEAR = Pose(Vector2(18.0, 10.5), 90.0.degrees, 0.0.degrees)
        val GROUND_INTAKE_CONE_FAR = Pose(Vector2(40.0, 9.5), 90.0.degrees, 0.0.degrees)
        val GROUND_INTAKE_CUBE_NEAR = Pose(Vector2(18.0, -8.0), 85.0.degrees, -180.0.degrees)
        val GROUND_INTAKE_CUBE_FAR = Pose(Vector2(40.0, -8.0), 85.0.degrees, -180.0.degrees)
//        val SHELF_INTAKE_POSE = Pose(Vector2(0.0, 9.0), -90.0.degrees, 180.0.degrees)
//        val LOW_SCORE = Pose(Vector2(0.0, 9.0), -90.0.degrees, 180.0.degrees)
        val BACK_LOW_SCORE_CONE_TOWARD = Pose(Vector2(-10.0, 9.0), -60.0.degrees, 0.0.degrees)
        val BACK_LOW_SCORE_CONE_AWAY = Pose(Vector2(-10.0, 9.0), -80.0.degrees, 0.0.degrees)
        val BACK_LOW_SCORE_CUBE = Pose(Vector2(-10.0, 9.0), -90.0.degrees, 0.0.degrees)
        val BACK_MIDDLE_SCORE_CONE_AWAY_MID = Pose(Vector2(-25.0, 29.0), -180.0.degrees, -180.0.degrees)
        val BACK_MIDDLE_SCORE_CONE_AWAY = Pose(Vector2(-32.0, 27.5), -180.0.degrees, -180.0.degrees)
        val BACK_MIDDLE_SCORE_CONE_TOWARD_MID = Pose(Vector2(-22.0, 30.5), -100.0.degrees, 0.0.degrees)
        val BACK_MIDDLE_SCORE_CONE_TOWARD = Pose(Vector2(-33.5, 30.5), -100.0.degrees, 0.0.degrees)
        val BACK_MIDDLE_SCORE_CUBE = Pose(Vector2(-24.5, 30.0), -100.0.degrees, 0.0.degrees)
        val BACK_HIGH_SCORE_CONE_TOWARD_MID = Pose(Vector2(-28.0, 48.0), -100.0.degrees, 0.0.degrees)
        val BACK_HIGH_SCORE_CONE_TOWARD = Pose(Vector2(-44.0, 45.0), -100.0.degrees, 0.0.degrees)
        val BACK_HIGH_SCORE_CONE_AWAY_MID = Pose(Vector2(-29.0, 45.0), -200.0.degrees, -180.0.degrees)
        val BACK_HIGH_SCORE_CONE_AWAY = Pose(Vector2(-44.0, 43.0), -200.0.degrees, -180.0.degrees)
        val FRONT_DRIVE_POSE = Pose(Vector2(0.0, 9.0), 92.0.degrees, -180.0.degrees)
        val BACK_DRIVE_POSE = Pose(Vector2(0.0, 9.0), -92.0.degrees, 0.0.degrees)
        val FLIP_INTAKE_TO_BACK_POSE = Pose(Vector2(-28.0, 26.0), 90.0.degrees, -90.0.degrees)
        val FLIP_INTAKE_TO_BACK_WRIST = Pose(Vector2(-28.0, 26.0), -90.0.degrees, 0.0.degrees)
        val FLIP_INTAKE_TO_FRONT_POSE = Pose(Vector2(28.0, 20.0), -90.0.degrees, -90.0.degrees)
        val FLIP_INTAKE_TO_FRONT_WRIST = Pose(Vector2(28.0, 20.0), 90.0.degrees, -180.0.degrees)
    }

    operator fun plus(otherPose: Pose) = Pose(wristPosition + otherPose.wristPosition, wristAngle + otherPose.wristAngle, pivotAngle + otherPose.pivotAngle)
}

suspend fun animateToPose(pose: Pose, minTime: Double = 0.0) = use(Arm, Intake) {
    println("Starting Animation $pose")
    val path = Path2D("newPath")
    path.addVector2(Pose.current.wristPosition)
    path.addVector2(pose.wristPosition)
    var distance = path.length
    var rate = 55.0  //  inches per second
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

package org.team2471.frc2023

import edu.wpi.first.wpilibj.Timer
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees

data class Pose(val wristPosition: Vector2, val wristAngle: Angle, val pivotAngle: Angle) {

    companion object {
        val current: Pose
            get() = Pose(Arm.endEffectorPosition, Intake.wristAngle, Intake.pivotAngle)
        val START_POSE = Pose(Vector2(0.0, 9.0), -90.0.degrees, 180.0.degrees)
        val GROUND_INTAKE_POSE = Pose(Vector2(0.0, 9.0), -90.0.degrees, 180.0.degrees)
        val SHELF_INTAKE_POSE = Pose(Vector2(0.0, 9.0), -90.0.degrees, 180.0.degrees)
        val LOW_SCORE = Pose(Vector2(0.0, 9.0), -90.0.degrees, 180.0.degrees)
        val MIDDLE_SCORE = Pose(Vector2(0.0, 9.0), -90.0.degrees, 180.0.degrees)
        val HIGH_SCORE = Pose(Vector2(0.0, 9.0), -90.0.degrees, 180.0.degrees)
    }
}

suspend fun animateToPose(pose: Pose) {
    val path = Path2D("newPath")
    path.addVector2(Pose.current.wristPosition)
    path.addVector2(pose.wristPosition)
    val distance = path.length
    val rate = 12.0  //  inches per second
    val time = distance / rate
    path.addEasePoint(0.0,0.0)
    path.addEasePoint(time, 1.0)

    val wristCurve = MotionCurve()
    wristCurve.storeValue(0.0, Pose.current.wristAngle.asDegrees)
    wristCurve.storeValue(time, pose.wristAngle.asDegrees)

    val pivotCurve = MotionCurve()
    pivotCurve.storeValue(0.0, Pose.current.pivotAngle.asDegrees)
    pivotCurve.storeValue(time, pose.pivotAngle.asDegrees)

    val timer = Timer()
    periodic {
        val t = timer.get()
        Arm.endEffectorPosition = path.getPosition(t)
        Intake.wristSetpoint = wristCurve.getValue(t).degrees
        Intake.pivotSetpoint = pivotCurve.getValue(t).degrees
        if (t>time) {
            this.stop()
        }
    }
}
package org.team2471.frc2023

import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.Length
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import java.text.FieldPosition

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

}
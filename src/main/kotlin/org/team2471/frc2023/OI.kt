package org.team2471.frc2023

import org.team2471.frc.lib.input.*
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.cube
import org.team2471.frc.lib.math.deadband
import org.team2471.frc.lib.math.squareWithSign
import org.team2471.frc.lib.units.degrees

object OI {
    val driverController = XboxController(0)
    val operatorController = XboxController(1)

    private val deadBandDriver = 0.1
    private val deadBandOperator = 0.1


    private val driveTranslationX: Double
        get() = (if (FieldManager.isRedAlliance) 1.0 else -1.0) * driverController.leftThumbstickX.deadband(deadBandDriver).squareWithSign()

    private val driveTranslationY: Double
        get() = (if (FieldManager.isRedAlliance) -1.0 else 1.0) * driverController.leftThumbstickY.deadband(deadBandDriver).squareWithSign()

    val driveTranslation: Vector2
        get() = Vector2(driveTranslationX, driveTranslationY) //does owen want this cubed?

    val driveRotation: Double
        get() = (driverController.rightThumbstickX.deadband(deadBandDriver)).cube() // * 0.6

    val driveLeftTrigger: Double
        get() = driverController.leftTrigger

    val driveRightTrigger: Double
        get() = driverController.rightTrigger

    val operatorLeftTrigger: Double
        get() = operatorController.leftTrigger

    val operatorLeftY: Double
        get() = operatorController.leftThumbstickY.deadband(0.2)

    val operatorLeftX: Double
        get() = operatorController.leftThumbstickX.deadband(0.2)

    val operatorRightTrigger: Double
        get() = operatorController.rightTrigger

    val operatorRightX: Double
        get() = operatorController.rightThumbstickX.deadband(0.25)

    val operatorRightY: Double
        get() = operatorController.rightThumbstickY.deadband(0.25)

    init {
//        driverController::y.whenTrue { println(FieldManager.nodeList[0]?.pos) }
        driverController::back.whenTrue { Drive.zeroGyro();
            Drive.initializeSteeringMotors() }
        driverController::start.whenTrue {Drive.calibrateRobotPosition() }
//        operatorController::x.whenTrue { scoreIfReady() }
       // driverController::a.whenTrue { Drive.dynamicDriveThreeFeetY()}
//        driverController::b.whenTrue { Drive.dynamicGoToFeeder()}
        driverController::y.whenTrue { Drive.dynamicGoToScoreCheck() }
        driverController::leftBumper.whenTrue {Arm.shoulderCoastMode()}
        driverController::rightBumper.whenTrue {Arm.shoulderBrakeMode()}

        operatorController::back.whenTrue { Arm.resetShoulderZero()}
        operatorController::start.whenTrue {
            Arm.wristPosOffset = Vector2(0.0, 0.0)
            Intake.wristOffset = 0.0.degrees
            Intake.pivotOffset = 0.0.degrees
            if (Intake.wristAngle < -75.0.degrees) animateToPose(Pose.BACK_DRIVE_POSE) else if (Intake.wristAngle > 75.0.degrees) animateToPose(Pose.FRONT_DRIVE_POSE)
        }
        operatorController::leftBumper.whenTrue { backScoreTowardCone() }
        operatorController::rightBumper.whenTrue { backScoreAwayCone() }
        operatorController::a.whenTrue {
            if (Intake.wristAngle < -75.degrees) {
                animateToPose(Pose.FLIP_INTAKE_TO_FRONT_POSE)
                animateToPose(Pose.FLIP_INTAKE_TO_FRONT_WRIST)
                animateToPose(Pose.FRONT_DRIVE_POSE)
            } else if (Intake.wristAngle > 75.0.degrees) {
                animateToPose(Pose.FLIP_INTAKE_TO_BACK_POSE)
                animateToPose(Pose.FLIP_INTAKE_TO_BACK_WRIST)
                animateToPose(Pose.BACK_DRIVE_POSE)
            }
        }
        operatorController::b.whenTrue {
            animateToPose(Pose.current + Pose(Vector2(-4.0, -3.0), 30.0.degrees, 0.0.degrees))
            Intake.intakeMotor.setPercentOutput(0.2)
        }
        ({operatorController.rightTrigger > 0.1}).whenTrue { intakeFromGround() }
    }
}

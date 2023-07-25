package org.team2471.frc2023

import edu.wpi.first.wpilibj.DriverStation
import org.jetbrains.kotlin.gradle.utils.`is`
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.input.*
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.cube
import org.team2471.frc.lib.math.deadband
import org.team2471.frc.lib.math.squareWithSign
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.motion.following.xPose
import org.team2471.frc.lib.units.degrees

object OI : Subsystem("OI") {
    val driverController = XboxController(0)
    val operatorController = XboxController(1)

    private val deadBandDriver = 0.1
    private val deadBandOperator = 0.1

    var controlledBy = PERSONINCONTROL.NONE
    var kidMode = false

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
        driverController::back.whenTrue {
            Drive.zeroGyro();
            Drive.initializeSteeringMotors()
        }
        driverController::start.whenTrue { Drive.calibrateRobotPosition() }
//        driverController::b.whenTrue { Drive.dynamicGoToFeeder()}
        driverController::x.whenTrue { Drive.xPose() }


        driverController::b.whenTrue {
            quickSpit()
        }
        driverController::a.whenTrue {
            if (kidMode) {
                backNod()
            }
        }
        driverController::leftBumper.whenTrue {
            if (!Drive.demoMode) {
                Drive.dynamicGoToScoreCheck()
            } else {
                println("In demo mode, i cannot do this")
            }
        }
        ({driveRightTrigger > 0.1}).whenTrue { //score testing time
            safeAnimationCheck(PERSONINCONTROL.DRIVER){
                if (kidMode) {
                    superQuickSpit()
                } else {
                    scoreObject()
                }
            }
        }
        ({driveLeftTrigger > 0.1}).whenTrue {
            safeAnimationCheck(PERSONINCONTROL.DRIVER) {
                flip()
            }
        }
//        driverController::rightBumper.whenTrue {
//            kidMode = Drive.demoMode && !kidMode
//        }
        operatorController::back.whenTrue { Arm.resetShoulderZero()}
        operatorController::start.whenTrue {
            safeAnimationCheck(PERSONINCONTROL.OPERATOR) {
                toDrivePose()
            }
        }
        operatorController::leftBumper.whenTrue {
            safeAnimationCheck(PERSONINCONTROL.OPERATOR) {
                if (kidMode) {
                    raiseArmFront()
                } else {
                    backScoreToward()
                }
            }
        }
        operatorController::rightBumper.whenTrue {
            safeAnimationCheck(PERSONINCONTROL.OPERATOR) {
                if (!kidMode) {
                    backScoreAway()
                }
            }
        }
        operatorController::x.whenTrue {
            resetArmVars()
            Arm.wristFrontOffset = Vector2(0.0, 0.0)
            Arm.wristBackOffset = Vector2(0.0, 0.0)
//            Arm.wristFrontOffset = Vector2(0.0, 0.0)
//            Arm.wristBackOffset = Vector2(0.0, 0.0)
//            Intake.wristOffset = 0.0.degrees
//            Intake.pivotOffset = 0.0.degrees
//            Arm.isFlipping = false
            println("Reset offsets")
        }
        ({operatorController.leftTrigger > 0.1}).whenTrue {
            safeAnimationCheck(PERSONINCONTROL.OPERATOR) {
                if (!Arm.isFlipping) {
                    intakeFromGround()
                } else {
                    println("currently flipping")
                }
            }
        }
        ({operatorController.dPad == Controller.Direction.DOWN}).whenTrue {
            AprilTag.resetCameras()
        }
        operatorController::b.whenTrue {
            if (kidMode) {
                Arm.pointToTag()
            }
        }
//        operatorController::a.whenTrue {
//            animateThroughPoses(Pose.SHORT_POSE_ONE, Pose.SHORT_POSE_TWO)
//        }
//        ({ driverController.dPad == Controller.Direction.DOWN }).whenTrue {
//            Intake.pivotSetpoint -= 1.0.degrees
//            println("changing pivot setpoint. setpoint: ${Intake.pivotSetpoint}  angle: ${Intake.pivotAngle}")
//        }
//        ({ driverController.dPad == Controller.Direction.UP}).whenTrue {
//            Intake.pivotSetpoint += 1.0.degrees
//            println("changing pivot setpoint. setpoint: ${Intake.pivotSetpoint}  angle: ${Intake.pivotAngle}")
//        }
    }

    override fun preEnable() {
        controlledBy = PERSONINCONTROL.NONE
    }
    suspend fun safeAnimationCheck (wantsControl: PERSONINCONTROL, actionWithAnimation : suspend() -> Unit) {
        if (controlledBy==PERSONINCONTROL.DRIVER && wantsControl==PERSONINCONTROL.OPERATOR){
            println("driver Is In Control")
        } else if (controlledBy==PERSONINCONTROL.OPERATOR && wantsControl==PERSONINCONTROL.DRIVER){
            println("operator Is In Control")
        } else {
            controlledBy = wantsControl
            actionWithAnimation ()
            controlledBy = PERSONINCONTROL.NONE
        }
    }
    enum class PERSONINCONTROL{
        DRIVER, OPERATOR, NONE
    }
}

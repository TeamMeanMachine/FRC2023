package org.team2471.frc2023

import edu.wpi.first.wpilibj.DriverStation
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.input.*
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.cube
import org.team2471.frc.lib.math.deadband
import org.team2471.frc.lib.math.squareWithSign
import org.team2471.frc.lib.motion.following.xPose

object OI : Subsystem("OI") {
    val driverController = XboxController(0)
    val operatorController = XboxController(1)

    private val deadBandDriver = 0.1
    private val deadBandOperator = 0.1

    var controlledBy = PERSONINCONTROL.NONE

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
        driverController::start.whenTrue { Drive.calibrateRobotPosition() }
//        driverController::b.whenTrue { Drive.dynamicGoToFeeder()}
        driverController::x.whenTrue { Drive.xPose() }
        driverController::b.whenTrue {
            Intake.intakeMotor.setPercentOutput(if (NodeDeckHub.isCone) Intake.CONE_TOWARD_SPIT else Intake.CUBE_SPIT)
        }
        driverController::leftBumper.whenTrue {
                Drive.dynamicGoToScoreCheck()
            }

        ({driveRightTrigger > 0.1}).whenTrue { //score testing time
            safeAnimationCheck(PERSONINCONTROL.DRIVER){
                scoreObject()
            }
        }
        ({driveLeftTrigger > 0.1}).whenTrue {
            safeAnimationCheck(PERSONINCONTROL.DRIVER) {
                flip()
            }
        }

        operatorController::back.whenTrue { Arm.resetShoulderZero()}
        operatorController::start.whenTrue {
            safeAnimationCheck(PERSONINCONTROL.OPERATOR) {
                toDrivePose()
            }
        }
        operatorController::leftBumper.whenTrue {
            safeAnimationCheck(PERSONINCONTROL.OPERATOR) {
                backScoreToward()
            }
        }
        operatorController::rightBumper.whenTrue {
            safeAnimationCheck(PERSONINCONTROL.OPERATOR) {
                backScoreAway()
            }
        }
        operatorController::x.whenTrue {
            Arm.wristFrontOffset = Vector2(0.0, 0.0)
            Arm.wristBackOffset = Vector2(0.0, 0.0)
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

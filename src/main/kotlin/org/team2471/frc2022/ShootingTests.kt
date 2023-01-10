package org.team2471.frc2022
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.TalonID
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem

object ShootingTests : Subsystem("ShootingTests") {
    private val bottomShootingMotor = MotorController(FalconID(Falcons.SHOOTER))
    private val topShootingMotor = MotorController(FalconID(Falcons.SHOOTER_TWO))
//    private val TalonTestMotor1 = MotorController(TalonID(Talons.TALON_TEST_ONE))
    private val TalonTestMotor2 = MotorController(TalonID(Talons.TALON_TEST_TWO))
    private val table = NetworkTableInstance.getDefault().getTable(name)
    val rpmEntry = table.getEntry("RPM")
    val rpmSetpointEntry = table.getEntry("RPM Setpoint")
    val rpmErrorEntry = table.getEntry("RPM Error")
    val rpmOffsetEntry = table.getEntry("RPM Offset")


    init {
        bottomShootingMotor.config {
            feedbackCoefficient = 60.0 / (2048 * 0.33)
            coastMode()
            pid {
                p(0.8e-5)
                i(0.0)//i(0.0)
                d(0.0)//d(1.5e-3) //1.5e-3  -- we tried 1.5e9 and 1.5e-9, no notable difference  // we printed values at the MotorController and the wrapper
                f(0.0555) //0.000045
            }
        }
        topShootingMotor.config {
            feedbackCoefficient = 60.0 / (2048 * 0.33)
            coastMode()
            inverted(true)
            pid {
                p(0.8e-5)
                i(0.0)//i(0.0)
                d(0.0)//d(1.5e-3) //1.5e-3  -- we tried 1.5e9 and 1.5e-9, no notable difference  // we printed values at the MotorController and the wrapper
                f(0.0555) //0.000045
            }
        }

         rpmSetpointEntry.setDouble(7500.0)

        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                rpmEntry.setDouble(rpm)
                rpmErrorEntry.setDouble(rpmSetpoint - rpm)
                if(OI.driverController.rightTrigger > 0.1) {
                    rpm = rpmSetpoint * OI.driverController.rightTrigger
                    // println("Got into rpm tests $rpm $rpmSetpoint. Hi.")
//                    shootingMotor.setPercentOutput(OI.driveRightTrigger)

                } else {
//                    rpm = 0.0
                    bottomShootingMotor.setPercentOutput(0.0)
                    topShootingMotor.setPercentOutput(0.0)
                }
            }
        }
    }

    var rpm: Double
        get() = bottomShootingMotor.velocity
        set(value) {
            println("rpm set to $value")
            bottomShootingMotor.setVelocitySetpoint(value)
            topShootingMotor.setVelocitySetpoint(value)

        }
    var rpmSetpoint: Double = 0.0
        get() {
            return rpmSetpointEntry.getDouble(1600.0)
        }
}
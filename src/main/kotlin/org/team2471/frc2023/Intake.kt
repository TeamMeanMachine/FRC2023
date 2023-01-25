package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableInstance

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.*
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller


object Intake : Subsystem("Intake") {
    val intakeMotor = MotorController(SparkMaxID(Sparks.INTAKE))
    val intakePivotMotor = MotorController(TalonID(Talons.INTAKE_PIVOT))

    private val table = NetworkTableInstance.getDefault().getTable(Intake.name)
    val currentEntry = table.getEntry("Current")
    val pivotEntry = table.getEntry("Pivot")
    val pivotSetpointEntry = table.getEntry("Pivot Setpoint")
    val pivotDriverOffsetEntry = table.getEntry("Pivot Controller")
    val intakeStateEntry = table.getEntry("Mode")

    var pivotDriverOffset
        get() = pivotDriverOffsetEntry.getDouble(0.0)
        set(value) { pivotDriverOffsetEntry.setDouble(value) }
    var pivotOffset = if (isCompBot) 0.0 else 0.0
    var pivotAngle : Double = 0.0
        get() = intakePivotMotor.position + pivotOffset//still have to convert to degrees
    var pivotSetpoint = pivotAngle
        get() = pivotSetpointEntry.getDouble(0.0)
        set(value) {
            field = value.coerceIn(PIVOT_BOTTOM, PIVOT_TOP) + pivotDriverOffset
            pivotSetpointEntry.setDouble(field)
        }

    const val INTAKE_POWER = 0.7
    const val PIVOT_BOTTOM = -70.0
    val PIVOT_TOP = 70.0

    enum class Mode {
        STOW, INTAKE, SCORE
    }

    var intakeState = Mode.STOW
    init {
        pivotDriverOffsetEntry.getDouble(0.0)
        intakePivotMotor.config(20) {
            feedbackCoefficient =
                360.0 / 2048.0 / 135.0 * 118.4 / 105.1   // degrees in a rotation, ticks per rotation
            brakeMode()
            pid {
                p(1e-5)
                d(0.00000005)
            }

            currentLimit(20, 30, 1)
        }
//        intakeMotor.config {
//            coastMode()
//            currentLimit(20, 40, 1)
//        }

        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
//                currentEntry.setDouble(intakeMotor.current)
                pivotEntry.setDouble(pivotAngle)
                intakeStateEntry.setString(intakeState.name)
            }
        }
    }

}

suspend fun Intake.powerTest() = use(this) {
    var power = 0.0
    var upPressed = false
    var downPressed = false
    periodic {
        if (OI.driverController.dPad == Controller.Direction.UP) {
            upPressed = true
        } else if (OI.driverController.dPad == Controller.Direction.DOWN) {
            downPressed = true
        }
        if (OI.driverController.dPad != Controller.Direction.UP && upPressed) {
            upPressed = false
            power += 0.001
            println("up power= ${power}")
        }
        if (OI.driverController.dPad != Controller.Direction.DOWN && downPressed) {
            downPressed = false
            power -= 0.001
            println("down power= ${power}")
        }
        intakePivotMotor.setPositionSetpoint(0.0, power)
        println("power= ${power}")
    }
}


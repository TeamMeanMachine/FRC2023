package org.team2471.frc2022

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DutyCycleEncoder

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.actuators.TalonID
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.parallel
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller
import org.team2471.frc.lib.input.whenTrue
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.degrees
import kotlin.math.absoluteValue
import edu.wpi.first.wpilibj.Timer as Timer


object Intake : Subsystem("Intake") {

    val intakeMotor = MotorController(TalonID(Talons.INTAKE))
    val intakePivotMotor = MotorController(FalconID(Falcons.INTAKE_PIVOT))


    private val table = NetworkTableInstance.getDefault().getTable(Intake.name)
    val currentEntry = table.getEntry("Current")
    val pivotEntry = table.getEntry("Pivot")
    val pivotSetpointEntry = table.getEntry("Pivot Setpoint")
    val pivotMotorEntry = table.getEntry("Pivot Motor")
    val pivotDriverOffsetEntry = table.getEntry("Pivot Controller")
    val intakeStateEntry = table.getEntry("Mode")
    val intakePresetEntry = table.getEntry("Intake Preset")

    var pivotDriverOffset
        get() = pivotDriverOffsetEntry.getDouble(0.0)
        set(value) { pivotDriverOffsetEntry.setDouble(value) }
    var pivotOffset = if (isCompBot) 0.0 else -56.0  //comp: if angle too high, increase offset 159.7   .0145
    val pivotEncoder = DutyCycleEncoder(if (isCompBot) DigitalSensors.INTAKE_PIVOT else DigitalSensors.INTAKE_PIVOT_PRACTICE)  // this encoder seems to give randomly changing answers - very naughty encoder
    var pivotAngle : Double = 0.0
        get() = ((if (isCompBot) -1.0 else 1.0) * ((pivotEncoder.absolutePosition - 0.427) * 360.0 * 118.4 / 105.8) + pivotOffset).degrees.wrap().asDegrees  //0.573  // (if (isCompBot) -1.0 else 1.0) * (((pivotEncoder.absolutePosition - 0.334) * 360.0 * 90.0 / 86.0) + pivotOffset).degrees.wrap().asDegrees

//        get() = intakePivotMotor.position
//        set(value) {
//            pivotOffset = 0.0
            //pivotOffset = value - pivotAngle
//        }

    val pivotPDController = PDController(0.05, 0.0)
    var pivotPDEnable = true
    var pivotSetpoint = pivotAngle
        get() = pivotSetpointEntry.getDouble(94.0)
        set(value) {
            field = value.coerceIn(PIVOT_BOTTOM, PIVOT_TOP) + pivotDriverOffset
            pivotSetpointEntry.setDouble(field)
        }

    const val INTAKE_POWER = 0.9
    const val PIVOT_BOTTOM = 0.0
    const val PIVOT_CATCH = 0.0

    val defaultPivotIntake = 26.5
    val PIVOT_INTAKE
        get() = if (isCompBot) intakePresetEntry.getDouble(defaultPivotIntake) else 16.0
    val PIVOT_STORE = if (isCompBot) 110.0 else 98.0
    val PIVOT_TOP = if (isCompBot) 118.0 else 98.0


//    val button = DigitalInput(9)
    var blue = 0

    val timer = Timer()
    val angleCurve = MotionCurve()

    var upPressed = false
    var downPressed = false
    var leftPressed = false

    var prevDriverControl = false

    enum class Mode {
        CATCH, INTAKE, STOW, POWERSAVE
    }

    var intakeState = Mode.STOW
    init {
        pivotDriverOffsetEntry.getDouble(0.0)
        intakePresetEntry.getDouble(defaultPivotIntake)
        intakePivotMotor.config(20) {
            feedbackCoefficient =
                360.0 / 2048.0 / 135.0 * 118.4 / 105.1  // * 111.0 / 103.0 // degrees in a rotation, ticks per rotation
            brakeMode()
//            inverted(true)
            pid {
                p(1e-5)
                d(0.00000005)
//                f(0.04)
            }

            currentLimit(20, 30, 1)//40, 60, 1)
        }
        intakeMotor.config {
            coastMode()
            currentLimit(20, 40, 1)
        }

        GlobalScope.launch(MeanlibDispatcher) {
            intakePresetEntry.setPersistent()
            parallel({
            periodic {
                if (pivotEncoder.isConnected && pivotAngle > (PIVOT_BOTTOM - 3.0) && pivotAngle < (PIVOT_TOP + 1.0)) {
                    println("connected ${pivotEncoder.isConnected}   pivotAngle ${pivotAngle > (PIVOT_BOTTOM - 1.0)} ")
                    resetPivotOffset()
                    println("setpoints pivotAngle")
                    this.stop()
                } else if (isCompBot) {
                    println("Intake not reset")
                }

//                else {
//                    intakePivotMotor.setRawOffset(PIVOT_BOTTOM.degrees)
//                    pivotSetpoint = PIVOT_BOTTOM
//                    println("setpoints PIVOT_BOTTOM")
//    // use this for competition...
//    //                    intakePivotMotor.setRawOffset(PIVOT_TOP.degrees)
//    //                    pivotSetpoint = PIVOT_TOP
//    //                    println("setpoints PIVOT_TOP")
//                }
            }},{
            periodic {
                currentEntry.setDouble(intakeMotor.current)
                pivotEntry.setDouble(pivotAngle) // intakePivotMotor.position)
                pivotMotorEntry.setDouble(intakePivotMotor.position)
//                println("pivotMotor ${intakePivotMotor.position}")
                intakeStateEntry.setString(intakeState.name)

                if (OI.driverController.rightBumper) {
                    pivotSetpoint -= 1.0
                    setIntakePower(0.0)
                }

                if (OI.driveLeftTrigger > 0.1) {
                    intake()
                }

                if (prevDriverControl && OI.driverController.leftTrigger < 0.1) {
                    catch()
                }

                prevDriverControl = OI.driverController.leftTrigger > 0.1

                if (intakeMotor.current > 12) OI.driverController.rumble = 1.0 else if (!Shooter.shootMode) OI.driverController.rumble = 0.0

                //println("$isCompBot intake angle: $pivotAngle ${pivotEncoder.absolutePosition}")
//                if (OI.operatorController.b) {
//                    setIntakePower(INTAKE_POWER)
//                    changeAngle(PIVOT_INTAKE)
//                }

//                if (OI.driverController.dPad == Controller.Direction.UP) {
//                    upPressed = true
//                } else if (OI.driverController.dPad == Controller.Direction.DOWN) {
//                    downPressed = true
//                }
//                if (OI.operatorController.dPad == Controller.Direction.LEFT) {
//                    leftPressed = true
//                }
//                if (OI.driverController.dPad != Controller.Direction.UP && upPressed) {
//                    upPressed = false
//                    pivotDriverOffset += 2
//                    //incrementRpmOffset()
//                    println("up. hi.")
//                }
//                if (OI.driverController.dPad != Controller.Direction.DOWN && downPressed) {
//                    downPressed = false
//                    pivotDriverOffset -= 2
//                    println("down. hi.")
//                }
            }})
        }
    }

    override fun preEnable() {
//        GlobalScope.launch(MeanlibDispatcher) {
//            val timer = Timer()
//            timer.start()
//            periodic {
//                if (pivotEncoder.isConnected && pivotAngle > PIVOT_BOTTOM && pivotAngle < PIVOT_TOP) {
//                    intakePivotMotor.setRawOffset(pivotAngle.degrees)
                    pivotSetpoint = pivotAngle
//                    println("setpoints pivotAngle")
//                }
//                else {
//                    intakePivotMotor.setRawOffset(PIVOT_BOTTOM.degrees)
//                    pivotSetpoint = PIVOT_BOTTOM
//                    println("setpoints PIVOT_BOTTOM")
//// use this for competition...
////                    intakePivotMotor.setRawOffset(PIVOT_TOP.degrees)
////                    pivotSetpoint = PIVOT_TOP
////                    println("setpoints PIVOT_TOP")
//                }
                // this code zeros the encoder against the top or bottom hard stop

//                val targetAngle = PIVOT_BOTTOM  // use PIVOT_TOP for competition
//                val power = -0.2  // for competition  use 0.2
//                intakePivotMotor.setPercentOutput(power)
//                if (intakePivotMotor.current > 35.0) {
//                    intakePivotMotor.setPercentOutput(0.0)
//                    intakePivotMotor.setRawOffset(targetAngle.degrees)
//                    pivotSetpoint = targetAngle
//                    println("Time 2 reset = ${timer.get()} targetAngle = $targetAngle")
//                    stop()
//                }

//            }
//        }

        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                if (pivotPDEnable) {
//                    val power = pivotPDController.update(pivotSetpoint - pivotAngle)
//                    setIntakePivotPower(power)
                    intakePivotMotor.setPositionSetpoint(pivotSetpoint)
                }
            }
        }
    }

//    val ballIsStaged: Boolean
//        get() = !button.get()

    fun resetPivotOffset(){
        println("resetting intake pivot rawOffset ${pivotAngle.degrees}")
        intakePivotMotor.setRawOffset(pivotAngle)
        pivotDriverOffset = 0.0
        pivotSetpoint = pivotAngle
    }

    fun setIntakePower(power: Double) {
        intakeMotor.setPercentOutput(power)
    }

    fun setIntakePivotPower(power: Double) {
        intakePivotMotor.setPercentOutput(power)
    }

    fun changeAngle(angle: Double) {
        if (!OI.driverController.rightBumper) {
            pivotSetpoint = angle
//            val angleCurve = MotionCurve()
//            print("angle currently at $pivotAngle ")
//            print(" going to $angle ")
//            val distance = (pivotAngle - angle).absoluteValue
//            val rate = 90.0 / 1.0  // degrees per sec
//            val time = distance / rate
//            println("intake angle $time")
//            angleCurve.storeValue(0.0, pivotAngle)
//            angleCurve.storeValue(time, angle)
//            val timer = Timer()
//            timer.start()
////        changeAngleSetUp(angle)
//            periodic {
//                val t = timer.get()
//                pivotSetpoint = angleCurve.getValue(t)
//                //println("${angleCurve.getValue(t)}")
//                if (t >= angleCurve.length) {
//                    stop()
//                }
//            }
        }
//                changeAnglePeriodic()
    }

//    fun changeAngleSetUp(angle: Double) {
//        print("angle currently at $pivotAngle ")
//        print(" going to $angle ")
//        val distance = (pivotAngle - angle).absoluteValue
//        val rate = 90.0 / 1.0  // degrees per sec
//        val time = distance / rate
//        println("intake angle $time")
//        angleCurve.storeValue(0.0, pivotAngle)
//        angleCurve.storeValue(time, angle)
//        timer.start()
//    }

//    fun changeAnglePeriodic() {
//        val t = timer.get()
//        pivotSetpoint = angleCurve.getValue(t)
//        //println("${angleCurve.getValue(t)}")
//        if (t >= angleCurve.length) {
//            stop()
//        }
//    }

    override suspend fun default() {
        periodic {
            pivotMotorEntry.setDouble(intakePivotMotor.position)

            //currentEntry.setDouble(Shooter.shootingMotor.current)
        }
    //    print(":)")
//        if (ballIsStaged) {
//            setIntakePower(INTAKE_POWER)
//        } else {
//            setIntakePower(0.5 * INTAKE_POWER)
//        }
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


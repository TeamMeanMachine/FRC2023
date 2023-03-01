package frc.robot

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.AnalogEncoder
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc2023.Arm
import org.team2471.frc2023.Pose


object ArmSim : Subsystem("ArmSim") {
    // The arm gearbox represents a gearbox containing two Vex 775pro motors.
    private val m_armGearbox = DCMotor.getVex775Pro(2)

    private const val kMotorPort = 0
    private const val kEncoderAChannel = 0
    private const val kEncoderBChannel = 1

    // The P gain for the PID controller that drives this arm.
    private const val kArmKp = 40.0
    private const val kArmKi = 0.0

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    //  = (2 * PI rads) / (4096 pulses)
    private const val kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096

    // Some constants for the simulation.
    private const val m_armReduction = 600.0
    private const val m_arm_topMass = 10.0 // Kilograms
    private val m_arm_topLength = Units.inchesToMeters(38.5)
    private const val m_arm_bottomMass = 4.0 // Kilograms
    private val m_arm_bottomLength = Units.inchesToMeters(27.0)
    private const val m_wristMass = 2.0 // Kilograms
    private val m_wristLength = Units.inchesToMeters(8.0)
    private const val m_arm_top_min_angle = -180
    private const val m_arm_top_max_angle = 260
    private const val m_arm_bottom_min_angle = -90
    private const val m_arm_bottom_max_angle = 190
    private const val m_wrist_min_angle = -360
    private const val m_wrist_max_angle = 360

    // Standard classes for controlling our arm
    private val m_topController = ProfiledPIDController(kArmKp, kArmKi, 0.0, TrapezoidProfile.Constraints(2.0, 5.0))
    private val m_bottomController = ProfiledPIDController(kArmKp, kArmKi, 0.0, TrapezoidProfile.Constraints(2.0, 5.0))
    private val m_wristController = ProfiledPIDController(kArmKp, kArmKi, 0.0, TrapezoidProfile.Constraints(2.0, 5.0))
    private val m_topEncoder = Encoder(kEncoderAChannel, kEncoderBChannel)
    private val m_bottomEncoder = Encoder(kEncoderAChannel + 2, kEncoderBChannel + 2)
//    private val m_wristEncoder = Encoder(kEncoderAChannel + 4, kEncoderBChannel + 4)
    private val m_topMotor = PWMSparkMax(kMotorPort)
    private val m_bottomMotor = PWMSparkMax(kMotorPort + 1)
    private val m_wristMotor = PWMSparkMax(kMotorPort + 2) //haha again

    private val m_arm_topSim: SingleJointedArmSim = SingleJointedArmSim(
        m_armGearbox,
        m_armReduction,
        SingleJointedArmSim.estimateMOI(m_wristLength, m_wristMass),
        m_wristLength,
        Units.degreesToRadians(m_wrist_min_angle.toDouble()),
        Units.degreesToRadians(m_wrist_max_angle.toDouble()),
//        m_arm_topMass,
        false,
        VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    )
    private val m_wristSim: SingleJointedArmSim = SingleJointedArmSim(
        m_armGearbox,
        m_armReduction,
        SingleJointedArmSim.estimateMOI(m_arm_topLength, m_wristMass),
        m_arm_topLength,
        Units.degreesToRadians(m_arm_top_min_angle.toDouble()),
        Units.degreesToRadians(m_arm_top_max_angle.toDouble()),
//        m_wristMass,
        false,
        VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    )
    private val m_arm_bottomSim: SingleJointedArmSim = SingleJointedArmSim(
        m_armGearbox,
        m_armReduction,
        SingleJointedArmSim.estimateMOI(m_arm_bottomLength, m_arm_bottomMass),
        m_arm_bottomLength,
        Units.degreesToRadians(m_arm_bottom_min_angle.toDouble()),
        Units.degreesToRadians(m_arm_bottom_max_angle.toDouble()),
//        m_arm_bottomMass,
        true,
        VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    )
    private val m_topEncoderSim = EncoderSim(m_topEncoder)
    private val m_bottomEncoderSim = EncoderSim(m_bottomEncoder)
//    private val m_wristEncoderSim = EncoderSim(m_wristEncoder)
    var presetChooser = SendableChooser<Int>()

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private val m_mech2d = Mechanism2d(90.0, 90.0)
    private val midNodeHome = m_mech2d.getRoot("Mid Node", 27.83, 0.0)
    private val MidNode =
        midNodeHome.append(MechanismLigament2d("Mid Cone Node", 34.0, 90.0, 10.0, Color8Bit(Color.kWhite)))
    private val highNodeHome = m_mech2d.getRoot("High Node", 10.58, 0.0)
    private val HighNode =
        highNodeHome.append(MechanismLigament2d("High Cone Node", 46.0, 90.0, 10.0, Color8Bit(Color.kWhite)))
    private val gridHome = m_mech2d.getRoot("Grid Home", 49.75, 0.0)
    private val GridNode =
        gridHome.append(MechanismLigament2d("Grid Wall", 49.75, 180.0, 50.0, Color8Bit(Color.kWhite)))
    private val dsHome = m_mech2d.getRoot("Double Substation Home", 49.75, 37.0)
    private val DSRamp = dsHome.append(
        MechanismLigament2d(
            "Double Substation Ramp", 13.75, 180.0, 10.0, Color8Bit(
                Color.kWhite
            )
        )
    )
    private val m_armPivot = m_mech2d.getRoot("ArmPivot", 65.0, 21.75)
    private val m_arm_bottom = m_armPivot.append(
        MechanismLigament2d(
            "Arm Bottom",
            27.0,
            -90.0,
            10.0,
            Color8Bit(Color.kGold)
        )
    )
    private val m_arm_tower =
        m_armPivot.append(MechanismLigament2d("ArmTower", 18.0, -90.0, 10.0, Color8Bit(Color.kSilver)))
    private val m_aframe_1 =
        m_armPivot.append(MechanismLigament2d("aframe1", 24.0, -50.0, 10.0, Color8Bit(Color.kSilver)))
    private val m_bumper = gridHome.append(MechanismLigament2d("Bumper", 30.5, 0.0, 60.0, Color8Bit(Color.kRed)))
    private val m_arm_top = m_arm_bottom.append(
        MechanismLigament2d(
            "Arm Top",
            28.5,
            Units.radiansToDegrees(m_arm_topSim.angleRads),
            10.0,
            Color8Bit(Color.kPurple)
        )
    )
    private val m_wrist = m_arm_top.append(
        MechanismLigament2d(
            "Intake",
            Units.metersToInches(m_wristLength),
            Units.radiansToDegrees(m_wristSim.angleRads),
            20.0,
            Color8Bit(Color.kGray)
        )
    )

    init {
        println("ArmSim.init")
        m_topEncoder.distancePerPulse = kArmEncoderDistPerPulse
        m_bottomEncoder.distancePerPulse = kArmEncoderDistPerPulse
//        m_wristEncoder.distancePerPulse = kArmEncoderDistPerPulse

        presetChooser.setDefaultOption("Starting Position", 0)
        presetChooser.addOption("Floor Intake Position", 1)
        presetChooser.addOption("Double Substation Intake", 2)
        presetChooser.addOption("Floor Node Score", 3)
        presetChooser.addOption("Mid Node Score", 4)
        presetChooser.addOption("High Node Score", 5)
        SmartDashboard.putData(presetChooser)

        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("Arm Sim", m_mech2d)

        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                // In this method, we update our simulation of what our arm is doing
                // First, we set our "inputs" (voltages)
                m_arm_topSim.setInput(m_topMotor.get() * RobotController.getBatteryVoltage())
                m_arm_bottomSim.setInput(m_bottomMotor.get() * RobotController.getBatteryVoltage())
                m_wristSim.setInput(m_wristMotor.get() * RobotController.getBatteryVoltage())

                // Next, we update it. The standard loop time is 20ms.
                m_arm_topSim.update(0.020)
                m_arm_bottomSim.update(0.020)
                m_wristSim.update(0.020)

                // Finally, we set our simulated encoder's readings and simulated battery voltage
                m_topEncoderSim.distance = m_arm_topSim.angleRads
                m_bottomEncoderSim.distance = m_arm_bottomSim.angleRads
//                m_wristEncoderSim.distance = m_wristSim.angleRads

                // SimBattery estimates loaded battery voltages
                RoboRioSim.setVInVoltage(
                    BatterySim.calculateDefaultBatteryLoadedVoltage(m_arm_topSim.currentDrawAmps + m_arm_bottomSim.currentDrawAmps + m_wristSim.currentDrawAmps)
                )

                // Update the Mechanism Arm angle based on the simulated arm angle
                m_arm_top.angle = Units.radiansToDegrees(m_arm_topSim.angleRads)
                m_arm_bottom.angle = Units.radiansToDegrees(m_arm_bottomSim.angleRads)
                m_wrist.angle = Units.radiansToDegrees(m_wristSim.angleRads)

                // this portion should get the values from Arm setpoints instead of this...

                val pose: Pose
                pose = when (presetChooser.selected) {
                    0 -> Pose.DRIVE_POSE
                    1 -> Pose.GROUND_INTAKE_FRONT
                    2 -> Pose.SHELF_INTAKE_POSE
                    3 -> Pose.LOW_SCORE
                    4 -> Pose.MIDDLE_SCORE
                    5 -> Pose.HIGH_SCORE
                    else -> Pose.DRIVE_POSE
                }

                val (shoulderAngle, elbowAngle) = Arm.inverseKinematics(pose.wristPosition)

                val pidOutputTop = m_topController.calculate(m_topEncoder.distance, shoulderAngle.asRadians)
                m_topMotor.setVoltage(pidOutputTop)

                val pidOutputBottom = m_bottomController.calculate(m_bottomEncoder.distance, elbowAngle.asRadians)
                m_bottomMotor.setVoltage(pidOutputBottom)

//                val pidOutputWrist = m_wristController.calculate(m_wristEncoder.distance, pose.wristAngle.asRadians)
//                m_wristMotor.setVoltage(pidOutputWrist)
            }
        }
    }

    override fun onDisable() {
        // This just makes sure that our simulation code knows that the motor's off.
        m_topMotor.set(0.0)
        m_bottomMotor.set(0.0)
        m_wristMotor.set(0.0)
    }

    override suspend fun default() {
        println("in ArmSim default() (spam)")
    }
}
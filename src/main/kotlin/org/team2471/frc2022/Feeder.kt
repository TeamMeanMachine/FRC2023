package org.team2471.frc2022

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DutyCycleEncoder

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.TalonID
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.util.Timer


object Feeder : Subsystem("Feeder") {

    val shooterFeedMotor = MotorController(TalonID(Talons.SHOOTER_FEED))
    val bedFeedMotor = MotorController(TalonID(Talons.BED_FEED))

    val button = DigitalInput(DigitalSensors.FEEDER_BUTTON)
    val feedDistanceEncoder = DutyCycleEncoder(DigitalSensors.FEEDER_DISTANCE)

    val secondShotDelayTimer = Timer()
    var feederPrint = false

    private val table = NetworkTableInstance.getDefault().getTable(Feeder.name)

    val feedUseFrontLimelightEntry = table.getEntry("useFrontLimelight")
    val distanceEntry = table.getEntry("Distance")
    val stageStatusEntry = table.getEntry("Mode")
    val isClearingEntry = table.getEntry("Clearing")

    val SHOOTER_FEED_POWER = 0.6
    val SHOOTER_STAGE_POWER = if (isCompBot) 0.5 else 1.0

    const val BED_FEED_POWER = 0.8

    val isAuto : Boolean
        get() {
            return DriverStation.isAutonomous()
        }
    var isClearing = false
    var cargoWasStaged = Shooter.cargoIsStaged
    var autoCargoShot = 0
    var waitASecond = false
    var secondShotDelay = false

    enum class Status {
        EMPTY,
        SINGLE_STAGED,
        DUAL_STAGED,
        ACTIVELY_SHOOTING,
        CLEARING
    }

    enum class Timer_Status {
        STOPPED,
        QUEUED,
        RUNNING,
        IGNORE
    }

    var currentFeedStatus : Status = Status.EMPTY
    var currentTimerStatus : Timer_Status = Timer_Status.STOPPED
    var autoFeedMode = false

    init {
        shooterFeedMotor.config {
            brakeMode()
            inverted(true)
        }
        bedFeedMotor.config {
            inverted(!isCompBot)
        }

        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                if (isAuto && feederPrint) {
                    println("feeder curr ${shooterFeedMotor.current}             status ${currentFeedStatus}  timerStatus ${currentTimerStatus}   allGood ${Shooter.allGood}   ")
                }

                currentFeedStatus = when {
                    isAuto && Shooter.allGood && Shooter.shootMode && Shooter.pastMinWait -> Status.ACTIVELY_SHOOTING  //pizza
                    Shooter.shootMode && OI.driveRightTrigger > 0.1 -> Status.ACTIVELY_SHOOTING
                    isClearing -> Status.CLEARING
                    Shooter.cargoIsStaged && Feeder.cargoIsStaged -> Status.DUAL_STAGED
                    Shooter.cargoIsStaged -> Status.SINGLE_STAGED
                    else -> Status.EMPTY
                }

                if (currentTimerStatus == Timer_Status.QUEUED && Shooter.cargoIsStaged) {
                    secondShotDelayTimer.start()
                    println("Second cargo detected. Wheeeeeee")
                    currentTimerStatus = Timer_Status.RUNNING
                }

//                if (currentFeedStatus == Status.ACTIVELY_SHOOTING) {
//                    println("autoshot! allGood rpmError: ${Shooter.rpmError.roundToInt()}    pitch: ${round(Shooter.pitchSetpoint - Shooter.pitch, 2)} aim: ${round(Limelight.aimError, 2)}")
//                }
//
//                if (currentFeedStatus != Status.ACTIVELY_SHOOTING)
//                    currentTimerStatus = Timer_Status.STOPPED

                stageStatusEntry.setString(currentFeedStatus.name)
                feedUseFrontLimelightEntry.setBoolean(Limelight.useFrontLimelight)
                distanceEntry.setDouble(feedDistance)
                isClearingEntry.setBoolean(isClearing)
//                if (!Shooter.isCargoAlignedWithAlliance && Shooter.cargoStageProximity > 200) {
//                    setShooterFeedPower(0.9)
//                    setBedFeedPower(0.0)
//                    println("shot other alliance's color")
//                }
                if (autoFeedMode) {
                    when (currentFeedStatus) {
                        Status.ACTIVELY_SHOOTING -> {
                            setBedFeedPower(BED_FEED_POWER)
                            //setShooterFeedPower(SHOOTER_FEED_POWER)
                            if (currentTimerStatus != Timer_Status.RUNNING) {
                                setShooterFeedPower(SHOOTER_FEED_POWER)
                            } else if (currentTimerStatus == Timer_Status.RUNNING && Shooter.cargoIsStaged &&  (secondShotDelayTimer.get()  < 0.05)) { //0.025 0.05
                                Shooter.rpmSecondOffset = -50.0
                                setShooterFeedPower(0.0)
                                println("Timer less than 100 mil. sec")
                            } else if (currentTimerStatus == Timer_Status.RUNNING && (secondShotDelayTimer.get() >= 0.05)) { // 0.025 0.5
                                Shooter.rpmSecondOffset = 0.0
                                setShooterFeedPower(SHOOTER_FEED_POWER)
                                println("Timer greater than 100 mil. sec.")
                                currentTimerStatus = Timer_Status.STOPPED
                            }
                            Intake.setIntakePower(0.0)
                            detectShots("autoFeed")
                        }
                        Status.DUAL_STAGED -> {
                            cargoWasStaged = true
                            setBedFeedPower(0.0)
                            if (Shooter.cargoStageProximity > Shooter.PROXMITY_STAGED_MAX_SAFE) {
                                // back out shot staged while pushing out 2nd staged at half power
                                setShooterFeedPower(-0.1)
                                setBedFeedPower(-BED_FEED_POWER/2.0)
                            } else {
                                setShooterFeedPower(0.0)
                            }
                        }
                        Status.SINGLE_STAGED -> {
                            cargoWasStaged = true
                            setBedFeedPower(BED_FEED_POWER)
                            if (Shooter.cargoStageProximity > Shooter.PROXMITY_STAGED_MAX_SAFE) {
                                setShooterFeedPower(-0.2)
                            } else {
                                setShooterFeedPower(0.0)
                            }
                        }
                        Status.EMPTY -> {
                            cargoWasStaged = false
                            setBedFeedPower(BED_FEED_POWER)
                            setShooterFeedPower(SHOOTER_STAGE_POWER)
                        }
                        Status.CLEARING -> {
                            cargoWasStaged = false
                            setBedFeedPower(-BED_FEED_POWER)
                            setShooterFeedPower(-SHOOTER_FEED_POWER)
                        }
                    }
                    if (currentFeedStatus != Status.ACTIVELY_SHOOTING) currentTimerStatus = Timer_Status.STOPPED
                } else if (!isAuto) {
                    if (Shooter.shootMode) {
                        setShooterFeedPower(OI.driveRightTrigger * 0.9)
                        setBedFeedPower(0.0)
                        detectShots("notautofeed")
                    } else if (isClearing) {
                        cargoWasStaged = false
                        setBedFeedPower(-BED_FEED_POWER)
                        setShooterFeedPower(-SHOOTER_FEED_POWER)
                    } else {
                        cargoWasStaged = false
                        setShooterFeedPower(0.0)
                        setBedFeedPower(0.0)
                    }
                } else {
                    setShooterFeedPower(0.0)
                }
            }
        }
    }

    override fun preEnable() {
        setShooterFeedPower(0.0)
    }
    fun detectShots(note : String = "") {
        if (cargoWasStaged && !Shooter.cargoIsStaged) {
            // handle cargo no longer staged while actively shooting (e.g. cargo has been shot)
            cargoWasStaged = false
            println("shot detected from $note ${Shooter.cargoStageProximity}")
            shotDetected()
        } else if (!cargoWasStaged && Shooter.cargoIsStaged) {
            // handle second ball feeding while actively shooting
            cargoWasStaged = true
        }
    }
    fun shotDetected() {
        if (currentTimerStatus != Timer_Status.IGNORE){
            currentTimerStatus = Timer_Status.QUEUED
            println("Shot is detected. Timer status is queued")
        }
        autoCargoShot += 1
        println("Shot has been detected rpm: ${Shooter.rpm} rpmError: ${Shooter.rpmError} aimError: ${Limelight.aimError} pitchError: ${Shooter.pitchSetpoint - Shooter.pitch}")
        println("Angular velocity: ${Drive.angularVelocity} radial velocity: ${Drive.radialVelocity} pitchFlyOffset: ${Shooter.distFlyOffset}")
    }

    val cargoIsStaged: Boolean
        get() = !button.get()

    val feedDistance: Double
        get() = -feedDistanceEncoder.absolutePosition / Math.PI * 20.0

    fun setShooterFeedPower(power: Double) {
        shooterFeedMotor.setPercentOutput(power)
    }

    fun setBedFeedPower(power: Double) {
        bedFeedMotor.setPercentOutput(power)
    }

    override suspend fun default() {
        periodic {
            feedUseFrontLimelightEntry.setBoolean(Limelight.useFrontLimelight)
        }
    }
}
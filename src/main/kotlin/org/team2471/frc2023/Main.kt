@file:JvmName("Main")

package org.team2471.frc2023

import FRC____.BuildConfig
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.team2471.frc.lib.framework.MeanlibRobot
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2023.testing.*
import java.net.NetworkInterface




@DelicateCoroutinesApi
object Robot : MeanlibRobot() {
    var startMeasureTime = System.nanoTime()
    var lastMeasureTime = startMeasureTime
    var isCompBot = true
    init {
        val logger = Logger.getInstance()
        logger.recordMetadata("ProjectName", "MyProject") // Set a metadata value

        if (isReal()/*true*/) {
            logger.addDataReceiver(WPILOGWriter("/home")) // Log to a USB stick
            logger.addDataReceiver(NT4Publisher()) // Publish data to NetworkTables
            PowerDistribution(1, PowerDistribution.ModuleType.kRev) // Enables power distribution logging
        } else {
            setUseTiming(false) // Run as fast as possible
            val logPath = LogFileUtil.findReplayLog() // Pull the replay log from AdvantageScope (or prompt the user)
            logger.setReplaySource(WPILOGReader(logPath)) // Read replay log
            logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))) // Save outputs to a new log
        }

        logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page

        logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        logger.start() // Start logging! No more data receivers, replay sources, or metadata values may be added.


        val networkInterfaces =  NetworkInterface.getNetworkInterfaces()
        println("retrieving network interfaces")
        for (iFace in networkInterfaces) {
            println("${iFace.name}")
            if (iFace.name == "eth0") {
                println("NETWORK NAME--->${iFace.name}<----")
                var macString = ""
                for (byteVal in iFace.hardwareAddress){
                    macString += String.format("%s", byteVal)
                }
                println("FORMATTED---->$macString<-----")

                isCompBot = (macString != "0-12847512372")
                println("I am compbot = $isCompBot")
            }
        }

        // i heard the first string + double concatenations were expensive...
        repeat(25) {
            println("RANDOM NUMBER: ${Math.random()}")
        }
        println("NEVER GONNA GIVE YOU UP")
        println(BuildConfig.BUILD_TIME)

        FieldManager
        OI
        println("Field Manager Active! Is Red ${FieldManager.isRedAlliance}")
        Drive
        println("Activating Drive!")
        Drive.zeroGyro()
        Drive.heading = 0.0.degrees

        AutoChooser
        println("Activating AutoChooser! Is Red ${AutoChooser.redSide}")
        NodeDeckHub
        println("Activating NodeDeckHub! startingPoint ${NodeDeckHub.startingPoint.name}")
        AprilTag
        println("Activating AprilTags! Last Detection: ${AprilTag.lastBackDetection}")
        PoseEstimator
        println("Activating PoseEstimator! currentPose ${PoseEstimator.currentPose}")

//        MAPoseEstimator
        Arm
        println("Activating Arm! currentPosition ${Arm.wristPosition}")
        Intake
        println("Activating Intake! wristAngle: ${Intake.wristAngle.asDegrees}")
//        PowerInfo
    }

    override suspend fun enable() {
        println("starting enable")
        FieldManager.beforeFirstEnable = false
        Drive.enable()
        Arm.enable()
        Intake.enable()
//        PowerInfo.enable()
//        SignalLights.enable()
        println("field centric? ${SmartDashboard.getBoolean("Use Gyro", true) && !DriverStation.isAutonomous()}")
        println("ending enable")
    }

    override suspend fun autonomous() {
        if (!Drive.demoMode) {
            initTimeMeasurement()
            println("autonomous starting")
            Drive.brakeMode()
            Drive.aimPDController = Drive.autoPDController
            println("autonomous Drive brakeMode ${totalTimeTaken()}")
            AutoChooser.autonomous()
            println("autonomous ending ${totalTimeTaken()}")
        } else {
            println("CANNOT RUN AUTO IN DEMO MODE!!!!!! (you're welcome for not killing anyone)")
        }
    }

    override suspend fun teleop() {
        println("telop begin")
        Drive.aimPDController = Drive.teleopPDController
        Drive.headingSetpoint = Drive.heading
    }

    override suspend fun test()  {
        println("test mode begin. Hi.")



//        Arm.feedForwardTest()

//        Intake.intakeTest()
//        Drive.setAngleOffsets()
    }


    override suspend fun disable() {
        OI.driverController.rumble = 0.0
        OI.operatorController.rumble = 0.0
        Drive.disable()
        Arm.disable()
        Intake.disable()
//        PowerInfo.disable()
//        SignalLights.disable()
    }

    private fun initTimeMeasurement(){
        startMeasureTime = System.nanoTime()
        lastMeasureTime = startMeasureTime
    }

    private fun updateNanosTaken(){
        lastMeasureTime = System.nanoTime()
    }

    fun totalTimeTaken(): Long {
        return System.nanoTime() - startMeasureTime
    }

    fun recentTimeTaken(): Long {
        val timeTaken = System.nanoTime() - lastMeasureTime
        updateNanosTaken()
        return timeTaken
    }
}

fun main() {
    println("start robot")
    RobotBase.startRobot { Robot }
}

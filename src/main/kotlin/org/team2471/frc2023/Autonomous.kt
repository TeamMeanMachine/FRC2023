package org.team2471.frc2023

import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.team2471.bunnybots2022.Drive
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.parallel
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.motion.following.driveAlongPath
import org.team2471.frc.lib.motion_profiling.Autonomi
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.measureTimeFPGA
import java.io.File
import java.util.*

private lateinit var autonomi: Autonomi


enum class Side {
    LEFT,
    RIGHT;

    operator fun not(): Side = when (this) {
        LEFT -> RIGHT
        RIGHT -> LEFT
    }
}

private var startingSide = Side.RIGHT


object AutoChooser {
    private val isRedAllianceEntry = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("isRedAlliance")
    private var autonomiEntryTopicSub =
        NetworkTableInstance.getDefault().getTable("PathVisualizer").getStringTopic("Autonomi").subscribe("")

    var cacheFile: File? = null
    var redSide: Boolean = true
        get() = isRedAllianceEntry.getBoolean(true)
        set(value) {
            field = value
            isRedAllianceEntry.setBoolean(value)
        }

    private val lyricsChooser = SendableChooser<String?>().apply {
        setDefaultOption("Country roads", "Country roads")
        addOption("take me home", "take me home")
    }

    private val testAutoChooser = SendableChooser<String?>().apply {
        addOption("None", null)
        addOption("20 Foot Test", "20 Foot Test")
        addOption("8 Foot Straight", "8 Foot Straight")
        addOption("2 Foot Circle", "2 Foot Circle")
        addOption("4 Foot Circle", "4 Foot Circle")
        addOption("8 Foot Circle", "8 Foot Circle")
        addOption("Hook Path", "Hook Path")
        setDefaultOption("90 Degree Turn", "90 Degree Turn")


    }

    private val autonomousChooser = SendableChooser<String?>().apply {
        setDefaultOption("Tests", "testAuto")
        addOption("Right Side 5 Auto", "right5")
        addOption("Middle 4 Ball", "middle4")
        addOption("Left Side 2 Auto", "leftSideAuto")
        addOption("Straight Back Shoot Auto", "straightBackShootAuto")
        addOption("Rotary", "rotaryAuto")
        addOption("Left Side Auto", "leftSideAuto")


    }

    init {
//        DriverStation.reportWarning("Starting auto init warning", false)
//        DriverStation.reportError("Starting auto init error", false)         //            trying to get individual message in event log to get timestamp -- untested

        SmartDashboard.putData("Best Song Lyrics", lyricsChooser)
        SmartDashboard.putData("Tests", testAutoChooser)
        SmartDashboard.putData("Autos", autonomousChooser)

        try {
            cacheFile = File("/home/lvuser/autonomi.json")
            if (cacheFile != null) {
                autonomi = Autonomi.fromJsonString(cacheFile?.readText())!!
                println("Autonomi cache loaded.")
            } else {
                println("Autonomi failed to load!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! RESTART ROBOT!!!!!!")
            }
        } catch (_: Throwable) {
            DriverStation.reportError("Autonomi cache could not be found", false)
            autonomi = Autonomi()
        }
        println("In Auto Init. Before AddListener. Hi.")
        NetworkTableInstance.getDefault().addListener(
            autonomiEntryTopicSub,
            EnumSet.of(
                NetworkTableEvent.Kind.kImmediate,
                NetworkTableEvent.Kind.kPublish,
                NetworkTableEvent.Kind.kValueAll
            )
        ) { event ->
            println("Automous change detected")
            val json = event.valueData.value.string
            if (json.isNotEmpty()) {
                val t = measureTimeFPGA {
                    autonomi = Autonomi.fromJsonString(json) ?: Autonomi()
                }
                println("Loaded autonomi in $t seconds")
                if (cacheFile != null) {
                    println("CacheFile != null. Hi.")
                    cacheFile!!.writeText(json)
                } else {
                    println("cacheFile == null. Hi.")
                }
                println("New autonomi written to cache")
            } else {
                autonomi = Autonomi()
                DriverStation.reportWarning("Empty autonomi received from network tables", false)
            }
        }
    }

    suspend fun leftSideAuto() = use(Intake, Arm, Drive) {
        val auto = autonomi["Left Side Auto"]
        if (auto != null) {
            Drive.driveAlongPath(auto["01- GetCube1"])
            Drive.driveAlongPath(auto["02- DropCube1"])
            Drive.driveAlongPath(auto["03- GetCube2"])
            Drive.driveAlongPath(auto["04- DropCube2"])
            Drive.driveAlongPath(auto["05- ToCharge"])
        }
    }

    suspend fun rightSideAuto() = use(Drive, Intake, Arm) {
        val auto = autonomi["right Side Auto"]
        if (auto != null) {
            Drive.driveAlongPath(auto["01- GetCube1"])
            Drive.driveAlongPath(auto["02- DropCube1"])
            Drive.driveAlongPath(auto["03- GetCube2"])
            Drive.driveAlongPath(auto["04- DropCube2"])
            Drive.driveAlongPath(auto["05- ToCharge"])
        }
        suspend fun autonomous() = use(Drive, name = "Autonomous") {
            println("Got into Auto fun autonomous. Hi. 888888888888888 ${Robot.recentTimeTaken()}")
            val selAuto = SmartDashboard.getString("Autos/selected", "no auto selected")
            SmartDashboard.putString("autoStatus", "init")
            println("Selected Auto = *****************   $selAuto ****************************  ${Robot.recentTimeTaken()}")
            when (selAuto) {
        //        "Tests" -> testAuto()
                else -> println("No function found for ---->$selAuto<-----  ${Robot.recentTimeTaken()}")
            }
            SmartDashboard.putString("autoStatus", "complete")
            println("finished autonomous  ${Robot.recentTimeTaken()}")
        }
    }
}

   /* private suspend fun testAuto() {
        val testPath = SmartDashboard.getString("Tests/selected", "no test selected") // testAutoChooser.selected
        if (testPath != null) {
            val testAutonomous = autonomi["Tests"]
            val path = testAutonomous?.get(testPath)
            if (path != null) {
                Drive.driveAlongPath(path, true)
            }
        }
    } *\
}
}

    */
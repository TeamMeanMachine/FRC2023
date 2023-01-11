package org.team2471.frc2022

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.DelicateCoroutinesApi
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonTrackedTarget
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion.following.*
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.meters
import org.team2471.frc2022.OI.driverController


@OptIn(DelicateCoroutinesApi::class)
object AprilTag : Subsystem("AprilTag") {
    private val photonVisionTable = NetworkTableInstance.getDefault().getTable("photonvision")
    private val translationDampenAmount = photonVisionTable.getEntry("Tranlsation Dampen Amount")
    private val tagTable = photonVisionTable.getSubTable("HD_USB_Camera")
    private val tagIdEntry = photonVisionTable.getEntry("tagid")
    var xOffset = 0.0
    var validTarget = false
    var xOffsetRepeat = false
    //    val xErrorLinearFilter = LinearFilter.movingAverage(4)
//    var xErrorAverage = 0.0
//    val yawErrorLinearFilter = LinearFilter.movingAverage(4)
//    var yawErrorAverage = 0.0
    var camera = PhotonCamera("HD_USB_Camera")


    //    private val thresholdTable = frontTable.getSubTable("thresholds")
//    private val hasTargetEntry =  tagTable.getEntry("hasTarget")


    private var last_result_time = 999999999990.0

    val tagId: Int
        get()= tagIdEntry.getNumber(0).toInt()

    val tda: Int
        get()= translationDampenAmount.getNumber(4).toInt()


    init {
        tagIdEntry.setNumber(24)
        translationDampenAmount.setNumber(4)
        tagIdEntry.setPersistent()

        println("AprilTags Initialized")
    }
    fun hasTarget(): Boolean {
        return camera.latestResult.hasTargets()
    }

    fun resetLastResult() {
        last_result_time = 999999999999999.0
    }

    fun resetOdometryWithTag() {
        var result = camera.latestResult
        if (result.hasTargets()) {

            last_result_time = result.timestampSeconds
            val targets: List<PhotonTrackedTarget> = result.getTargets().filter {it.fiducialId == tagId && it.poseAmbiguity < 0.2}
            for (target in targets){
                //println(target.fiducialId)
                validTarget = true
                xOffset = target.bestCameraToTarget.y
                Drive.position = Vector2(Drive.position.x + xOffset, Drive.position.y)
                println("X Difference: $xOffset")
            }
        }
    }

//

    override suspend fun default() {
        var lastXOffset = 0.0
        periodic {
            validTarget = false
            //println(hasTargetEntry.getBoolean(false))
            if(driverController.a) {
                resetLastResult()
            }
            var result = camera.latestResult
            val time = Timer.getFPGATimestamp()
            val positionDiff = Drive.poseDiff(time - result.latencyMillis)


            val hasTargets: Boolean = result.hasTargets()


            // println("Current Time: ${result.timestampSeconds} last time: $last_result_time")
                // println("${hasTargets}")
                if (hasTargets) {

                    last_result_time = result.timestampSeconds
                    val targets: List<PhotonTrackedTarget> = result.getTargets().filter {it.fiducialId == tagId && it.poseAmbiguity < 0.2}
                    for (target in targets){
                        //println(target.fiducialId)
                        validTarget = true
                        xOffset = target.bestCameraToTarget.y / tda
                        println("X Difference: $xOffset")
                        xOffsetRepeat = lastXOffset == xOffset
                        lastXOffset= xOffset
                    }
                }
                //println(targets)
            }
        }
    }

    fun reset() {
    }


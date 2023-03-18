package org.team2471.frc2022

import edu.wpi.first.wpilibj.DigitalInput
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem

object Test : Subsystem("Test") {
    val limitSwitch = DigitalInput(DigitalSensors.LIMIT_SWITCH)

    var near: Boolean = false
        get() = !limitSwitch.get() //defaults to true then is false when magnet is detected

    init {
        GlobalScope.launch {
            periodic {
//                println("near? $near")
            }
        }
    }

    override suspend fun default() {
        periodic {
//            println("Love defaults")
        }
    }
}
package org.team2471.frc2023

import edu.wpi.first.wpilibj.SerialPort
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.coroutines.periodic


object SignalLights : Subsystem("SignalLights") {
    private val serial = SerialPort(9600, SerialPort.Port.kUSB1)

    init {

    }

    override suspend fun default() {
        periodic {
            if (OI.operatorController.x)
                // Pattern - red, white, black scroll
                serial.write(byteArrayOf('t'.code.toByte()), 1)
            else if (OI.operatorController.y)
                // Pattern - yellow flashing
                serial.write(byteArrayOf('y'.code.toByte()), 1)
        }
    }

    override fun preEnable() {
        super.preEnable()

        // Set starting pattern - Red
        serial.write(byteArrayOf('s'.code.toByte()), 1)
    }

    override fun onDisable() {
        super.onDisable()

        // Blank LEDs
        serial.write(byteArrayOf('o'.code.toByte()), 1)
    }

}
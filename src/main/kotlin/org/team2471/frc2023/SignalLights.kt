package org.team2471.frc2023//package org.team2471.frc2023

import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.Timer
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.input.Controller


object SignalLights : Subsystem("SignalLights") {
    private var serial : SerialPort? = null
    private var ports = arrayListOf(SerialPort.Port.kUSB1, SerialPort.Port.kUSB2, SerialPort.Port.kUSB)

    var state: LightState = LightState.IDLE

    init {
        for (port in ports) {
            if (serial == null) {
                connectToSerial(port)
            }
        }
        idle()
        /*GlobalScope.launch {
            periodic {
                when (OI.operatorController.dPad) {
                    Controller.Direction.LEFT -> idle()
                    else -> {}
                }

                if (Intake.holdDetectedTime + 3.0 > Timer.getFPGATimestamp()) {
                    flashGreen()
                    println("Setting To Green!")
                } else if (Intake.holdDetectedTime + 3.0 < Timer.getFPGATimestamp() && state == LightState.GREEN) {
                    idle()
                }
            }
        }*/
    }
    fun flashYellow() {
        if (this.state != LightState.YELLOW) {
            println("flashing yellow")
            this.state = LightState.YELLOW
            serial?.write(byteArrayOf('y'.code.toByte()), 1)
        }
    }

    fun flashPurple(){
        if (this.state != LightState.PURPLE) {
            println("flashing purple")
            this.state = LightState.PURPLE
            serial?.write(byteArrayOf('p'.code.toByte()), 1)
        }
    }

    fun green(){
        if (this.state != LightState.GREEN) {
            println("flashing green")
            this.state = LightState.GREEN
            serial?.write(byteArrayOf('g'.code.toByte()), 1)
        }
    }

    fun idle(){
        if (this.state != LightState.IDLE) {
            println("starting signal lights")
            this.state = LightState.IDLE
            serial?.write(byteArrayOf('s'.code.toByte()), 1)
        }
    }

    fun off(){
        if (this.state != LightState.OFF) {
            println("signal lights off")
            this.state = LightState.OFF
            serial?.write(byteArrayOf('o'.code.toByte()), 1)
        }
    }

    override suspend fun default() {
        periodic {
            when (OI.operatorController.dPad) {
                Controller.Direction.LEFT -> idle()
//                Controller.Direction.RIGHT -> green()
                else -> {}
            }

            if (Intake.holdDetectedTime + 3.0 > Timer.getFPGATimestamp()) {
                green()
                println("Setting To Green!")
            } else if (Intake.holdDetectedTime + 3.0 < Timer.getFPGATimestamp() && state == LightState.GREEN) {
                idle()
            }
        }
    }

    private fun connectToSerial(port:SerialPort.Port) {
        try {
            serial = SerialPort(9600, port)
            println("connected to ${port.name}")
        } catch (ex : Exception) {
            println("failed to connect to serial port : ${port.name}")
        }
    }

    override fun preEnable() {
        // Set starting pattern - Red
        for (port in ports) {
            if (serial == null) {
                connectToSerial(port)
            }
        }

        idle()
    }

    override fun onDisable() {
        // Blank LEDs
        off()
    }

}

enum class LightState {
    PURPLE, YELLOW, GREEN, IDLE, RED, BLUE, OFF
}
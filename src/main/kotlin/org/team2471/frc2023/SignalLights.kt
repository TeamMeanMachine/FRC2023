package org.team2471.frc2023//package org.team2471.frc2023

import edu.wpi.first.wpilibj.SerialPort
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.input.Controller


object SignalLights : Subsystem("SignalLights") {
    private var serial : SerialPort? = null
    private var ports = arrayListOf(SerialPort.Port.kUSB1, SerialPort.Port.kUSB2, SerialPort.Port.kUSB)
    init {
        for (port in ports) {
            if (serial == null) {
                connectToSerial(port)
            }
        }
        start()
        GlobalScope.launch {
            periodic {
                when (OI.operatorController.dPad) {
                    Controller.Direction.DOWN -> flashYellow()
                    Controller.Direction.UP -> flashPurple()
                    Controller.Direction.LEFT -> start()
                    Controller.Direction.RIGHT -> off()
                    else -> {}
                }
            }
        }
    }
    fun flashYellow() {
        println("flashing yellow")
        serial?.write(byteArrayOf('y'.code.toByte()), 1)
    }
    fun flashPurple(){
        println("flashing purple")
        serial?.write(byteArrayOf('p'.code.toByte()), 1)
    }
    fun rainbow(){
        println("flashing rainbow")
        serial?.write(byteArrayOf('r'.code.toByte()), 1)
    }
    fun start(){
        println("starting signal lights")
        serial?.write(byteArrayOf('s'.code.toByte()), 1)
    }
    fun off(){
        println("signal lights off")
        serial?.write(byteArrayOf('o'.code.toByte()), 1)
    }
    fun doD(){
        println("signal lights off")
        serial?.write(byteArrayOf('d'.code.toByte()), 1)
    }
    override suspend fun default() {
        periodic {
            when (OI.operatorController.dPad) {
                Controller.Direction.DOWN -> flashYellow()
                    Controller.Direction.UP -> flashPurple()
                Controller.Direction.LEFT -> doD()
                Controller.Direction.RIGHT -> off()
                else -> {}
            }
//            if (OI.operatorController.y) {
//                // Pattern - yellow flashing
//                flashYellow()
//                //flashYellow()
//            }
//            else if (OI.operatorController.x) {
//                // Pattern - purple flashing
//                flashPurple()
//                //flashPurple()
//            }
//            else if (OI.operatorController.b){
//                if(OI.operatorController.y){
//                    rainbow()
//                }
//            }
        }
    }
    fun connectToSerial(port:SerialPort.Port){
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
        start()
    }

    override fun onDisable() {
        // Blank LEDs
        off()
    }

}
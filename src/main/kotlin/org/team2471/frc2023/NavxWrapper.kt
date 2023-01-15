package org.team2471.frc2023

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.interfaces.Gyro

class NavxWrapper : Gyro {
    private val navx = AHRS(SPI.Port.kMXP)

    override fun getAngle(): Double {
        return -navx.yaw.toDouble()
    }

    fun getRoll(): Double {
        return navx.roll.toDouble()
    }

    override fun getRate(): Double = navx.rawGyroZ.toDouble()

    override fun close() {
        navx.close()
    }

    override fun calibrate() = Unit

    override fun reset() = navx.reset()
}
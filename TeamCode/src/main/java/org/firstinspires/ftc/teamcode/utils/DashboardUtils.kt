//package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule

class DashboardUtils {
    companion object {
        val x0 = 0.0
        val y0 = 0.0
        val xOffset = 30.0
        val yOffset = 30.0
        val radius = 10.0
        fun drawServoModules(swerveModules: Array<SwerveModule>, dashboard:FtcDashboard){
            val packet = TelemetryPacket()
            packet
                .fieldOverlay()
                // frontLeft
                .setStroke("green")
                .strokeCircle(x0+xOffset,y0+yOffset, radius)
                .strokeLine(x0+xOffset,y0+yOffset,
                    getXCoord(x0+xOffset, swerveModules[0].turnMotor.moduleAngle),
                    getYCoord(y0+yOffset, swerveModules[0].turnMotor.moduleAngle)
                )

                // frontRight
                .setStroke("blue")
                .strokeCircle(x0+xOffset,y0-yOffset, radius)
                .strokeLine(x0+xOffset,y0-yOffset,
                    getXCoord(x0+xOffset, swerveModules[1].turnMotor.moduleAngle),
                    getYCoord(y0-yOffset, swerveModules[1].turnMotor.moduleAngle)
                )

                // backLeft
                .setStroke("orange")
                .strokeCircle(x0-xOffset,y0+yOffset, radius)
                .strokeLine(x0-xOffset,y0+yOffset,
                    getXCoord(x0-xOffset, swerveModules[2].turnMotor.moduleAngle),
                    getYCoord(y0+yOffset, swerveModules[2].turnMotor.moduleAngle)
                )

                // backRight
                .setStroke("purple")
                .strokeCircle(x0-xOffset,y0-yOffset, radius)
                .strokeLine(x0-xOffset,y0-yOffset,
                    getXCoord(x0-xOffset, swerveModules[3].turnMotor.moduleAngle),
                    getYCoord(y0-yOffset, swerveModules[3].turnMotor.moduleAngle)
                )

            dashboard.sendTelemetryPacket(packet)
        }

        private fun getXCoord(x:Double, angle:Double): Double {
            return x + radius * kotlin.math.cos(angle)
        }

        private fun getYCoord(y:Double, angle:Double):Double{
            return y + radius * kotlin.math.sin(angle)
        }

    }
}
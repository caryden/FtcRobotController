package org.firstinspires.ftc.teamcode.tests.subsystems

import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.hardware.AnalogInput
import org.junit.Test
import org.mockito.kotlin.*

class SwerveModuleTests {

    @Test
    fun initialize() {
        val driveMotor = mock<MotorEx>()
        val turnMotor = mock<CRServo>()
        val analogInput = mock<AnalogInput>() {
            on { maxVoltage } doReturn 3.3
            on { voltage } doReturn 1.65
        }
//        val testDispatcher = StandardTestDispatcher()

 //       val swerveModule = SwerveModule(driveMotor, turnMotor, analogInput, testDispatcher)
    }

}
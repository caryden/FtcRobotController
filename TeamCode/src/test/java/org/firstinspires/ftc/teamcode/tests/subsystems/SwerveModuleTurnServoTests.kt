package org.firstinspires.ftc.teamcode.tests.subsystems

import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.AnalogInput
import kotlinx.coroutines.test.StandardTestDispatcher
import org.firstinspires.ftc.teamcode.subsystems.LeapfrogTurnServo
import org.junit.Assert
import org.junit.Test
import org.mockito.ArgumentMatchers
import org.mockito.kotlin.*
import kotlin.math.max
import kotlin.math.min

class SwerveModuleTurnServoTests {
    private val maxAnalogInputVoltage = 3.3
    @Volatile private var turnServoAngle = 1.1/ maxAnalogInputVoltage * 2 * Math.PI

    @Test
    fun initialize() {
        val axonMaxPlusAt6VSecondsPer60Degrees = 0.115
        val turnMotorMaxRadiansPerSecond = (60.0 / axonMaxPlusAt6VSecondsPer60Degrees) * (2 * Math.PI  / 360.0)
        var turnServoSpeed  = 0.0

        val analogInput = mock<AnalogInput>() {
            on { maxVoltage } doReturn maxAnalogInputVoltage
            on { voltage } doReturn  turnServoAngle / (2 * Math.PI) * maxAnalogInputVoltage
        }

        val turnMotor = mock<CRServo>() {
            on { set(ArgumentMatchers.anyDouble()) } doAnswer {
                val power = it.getArgument<Double>(0)
                val elapsedTime = 20e-3 // this needs to match the delay used in the pid loop
                turnServoAngle = wrapAngle(turnServoAngle + (turnServoSpeed * elapsedTime))
                turnServoSpeed = max(min(power,1.0), -1.0) * turnMotorMaxRadiansPerSecond

                // update the test voltage to match the turnServoAngle
                val ans = whenever(analogInput.voltage).thenReturn(turnServoAngle / (2 * Math.PI) * maxAnalogInputVoltage)

                println("turnServoAngle: $turnServoAngle, power: $power, turnServoSpeed: $turnServoSpeed, elapsedTime: $elapsedTime")
            }
        }


        val testDispatcher = StandardTestDispatcher()
        val turnServo = LeapfrogTurnServo(turnMotor, analogInput, testDispatcher)
        turnServo.initialize()
        turnServo.startControlLoop()
//        testDispatcher.scheduler.advanceTimeBy(1000)
//        Assert.assertEquals(0.0, turnServo.moduleAngle, 0.0001)

        val newModuleAngle = 1.5 * Math.PI
        println("changing module angle to $newModuleAngle")
        turnServo.moduleAngle = newModuleAngle
        testDispatcher.scheduler.advanceTimeBy(10000)
        Assert.assertEquals(newModuleAngle, turnServo.moduleAngle, 0.0001)

    }


    private fun wrapAngle(angle: Double) : Double {
        return (2 * Math.PI + angle) % (2 * Math.PI )
    }

}
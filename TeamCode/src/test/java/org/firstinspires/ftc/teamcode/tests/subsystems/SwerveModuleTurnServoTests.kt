package org.firstinspires.ftc.teamcode.tests.subsystems

import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveConfiguration
import org.firstinspires.ftc.teamcode.subsystems.SwerveModuleTurnServo
import org.firstinspires.ftc.teamcode.utils.clamp
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import org.mockito.ArgumentMatchers
import org.mockito.kotlin.*
import kotlin.math.max
import kotlin.math.min

class SwerveModuleTurnServoTests {
    val maxAnalogInputVoltage = 3.3

    // note that drReturn sets a constant value for the return value of the function,
    // doAnswer allows you to set a function that will be called when the function is called
    val analogInput = mock<AnalogInput>() {
        on { maxVoltage } doReturn maxAnalogInputVoltage
        on { voltage } doAnswer { turnServoAngle / (2 * Math.PI) * maxAnalogInputVoltage }
    }

    var turnServoSpeed  = 0.0
    var turnServoAngle = 1.1/ maxAnalogInputVoltage * 2 * Math.PI

    val turnMotor = mock<CRServo>() {
        on { set(ArgumentMatchers.anyDouble()) } doAnswer {
            val power = it.getArgument<Double>(0)
            val elapsedTime = 20e-3 // this needs to match the loop time (roughly)

            // update the servo angle based on the preceding speed and elapsed time
            turnServoAngle = wrapAngle(turnServoAngle + (turnServoSpeed * elapsedTime))

            // then update the speed
            turnServoSpeed = power.clamp(-1.0,1.0) * SwerveDriveConfiguration.maxAngularSpeed
        }
    }


    @Test
    fun initialize_should_zero_module_angle() {
        val swerveModuleTurnServo = SwerveModuleTurnServo(turnMotor, analogInput)
        swerveModuleTurnServo.initialize()
        assertEquals(0.0, swerveModuleTurnServo.moduleAngle, 1e-6)
    }

    @Test
    fun update_module_angle_and_calling_periodic_should_run_the_pid_controller() {

        val swerveModuleTurnServo = SwerveModuleTurnServo(turnMotor, analogInput)
        swerveModuleTurnServo.initialize()

        val targetModuleAngle = Math.PI
        swerveModuleTurnServo.moduleAngle = targetModuleAngle

        // call periodic here simulates the scheduler calling periodic to run the PID controller
        var maxLoops = 1000
        while (!swerveModuleTurnServo.atSetPoint && --maxLoops > 0) {
            swerveModuleTurnServo.periodic()
            println("targerModuleAngle: $targetModuleAngle, moduleAngle: ${swerveModuleTurnServo.moduleAngle}, turnServoAngle: $turnServoAngle")
        }

        assertEquals(targetModuleAngle, swerveModuleTurnServo.moduleAngle, swerveModuleTurnServo.turnPIDAngleTolerance)
    }

    private fun wrapAngle(angle: Double) : Double {
        return (2 * Math.PI + angle) % (2 * Math.PI )
    }

}
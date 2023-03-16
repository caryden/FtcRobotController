package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.qualcomm.robotcore.hardware.AnalogInput
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlin.math.abs
import kotlin.math.sign

class SwerveModule(private val driveMotor: MotorEx, private val turnMotor: CRServo, private val angleAnalogInput: AnalogInput) {
    private var swerveModuleState : SwerveModuleState = SwerveModuleState(0.0, Rotation2d(0.0))
    private val turnPIDController = PIDController(1.0, 0.0, 0.0)
    private var servoWrapAngleOffset = 0.0
    private var servoAngleOffset = 0.0
    private var currentWrappedServoAngle = 0.0

    init {
        // launch a coroutine to run the PIDController to control the turnMotor Power until the measured angle is the desired angle (swerveModuleState.angle)
        CoroutineScope(Dispatchers.Default).launch {
            while (true) {

                val unwrappedServoAngle = getUnwrappedServoAngle()
                val newWrappedServoAngle =  servoWrapAngleOffset + unwrappedServoAngle

                // if the new angle is more than pi radians away from the current angle, add or subtract 2pi radians to the offset
                // this is handle the wrapping of the servo angle due to the servo gear ratio
                if (abs(newWrappedServoAngle - currentWrappedServoAngle) > Math.PI)
                    servoWrapAngleOffset += 2 * Math.PI * -sign(newWrappedServoAngle - currentWrappedServoAngle)
                currentWrappedServoAngle = servoWrapAngleOffset + unwrappedServoAngle

                val gearRatio = 24.0/60.0
                val currentModuleAngle = currentWrappedServoAngle * gearRatio
                // run PIDController to control the turnMotor Power until the measured angle is the desired angle (swerveModuleState.angle)
                val turnMotorPower = turnPIDController.calculate(currentModuleAngle, swerveModuleState.angle.radians)
                turnMotor.set(turnMotorPower)
                if(!compareWithTolerance(turnMotorPower, 0.0))
                    delay(20)
                else
                    delay(100)
            }
        }
    }

    private fun compareWithTolerance(double1: Double, double2: Double, tolerance :Double = 1e-9 ): Boolean {
        return abs(double1 - double2) < tolerance
    }
    fun setSwerveModuleState(swerveModuleState : SwerveModuleState) {
        this.swerveModuleState = swerveModuleState // the coroutine will use this to calculate the turnMotor Power
        driveMotor.set(swerveModuleState.speedMetersPerSecond)
    }
    private fun getUnwrappedServoAngle(): Double {
        return angleAnalogInput.voltage/angleAnalogInput.maxVoltage * 2 * Math.PI - servoAngleOffset
    }
    private fun zeroServoAngle() {
        servoAngleOffset = getUnwrappedServoAngle()
    }
    fun init() {
        zeroServoAngle()
        setSwerveModuleState(SwerveModuleState(0.0, Rotation2d(0.0)))
    }
}
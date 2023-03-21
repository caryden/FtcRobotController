package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.AnalogInput
import kotlinx.coroutines.*
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

class LeapfrogTurnServo(private val turnMotor: CRServo, private val angleAnalogInput: AnalogInput, private val dispatcher: CoroutineDispatcher = Dispatchers.Default) {
    private val turnPIDController = PIDController(-0.01, 0.0, 0.0)

    private var servoWrapAngleOffset = 0.0
    private var currentWrappedServoAngle = 0.0

    private var currentModuleAngle = 0.0
    @Volatile
    private var targetModuleAngle = 0.0

    private var initialized = false
    private var initialModuleAngle = 0.0
    private val gearRatio = 24.0/60.0

    private var scope = CoroutineScope(dispatcher)

    var moduleAngle : Double
        get() = currentModuleAngle
        set(value) {
            targetModuleAngle = value
        }

    fun initialize() {
        initialModuleAngle = currentWrappedServoAngle * gearRatio
        initialized = true
    }
    fun startControlLoop() {
        if(!scope.isActive) {
            scope.launch {
                while (isActive) {
                    runTurnMotorPID()
                    delay(20)
                }
                turnMotor.set(0.0)
            }
        }
    }
    fun stopControlLoop() {
        if(scope.isActive) {
            scope.cancel()
        }
    }
    private fun runTurnMotorPID()  {
        val unwrappedServoAngle = getUnwrappedServoAngle()
        val newWrappedServoAngle =  servoWrapAngleOffset + unwrappedServoAngle

        // if the new angle is more than pi radians away from the current angle, add or subtract 2pi radians to the offset
        // this is handle the wrapping of the servo angle due to the servo gear ratio
        if (abs(newWrappedServoAngle - currentWrappedServoAngle) > Math.PI)
            servoWrapAngleOffset += 2 * Math.PI * -sign(newWrappedServoAngle - currentWrappedServoAngle)
        currentWrappedServoAngle = servoWrapAngleOffset + unwrappedServoAngle

        currentModuleAngle = (currentWrappedServoAngle * gearRatio - initialModuleAngle) % (2 * Math.PI)

        // calculate the difference between the current angle and the target angle and normalize it to be between -180 and 180
        val angleDifference = (currentModuleAngle - targetModuleAngle + Math.PI) % (2 * Math.PI) - Math.PI
        val turnMotorPower = max(-1.0, min(1.0, turnPIDController.calculate(angleDifference, 0.0)))
    }
    private fun getUnwrappedServoAngle(): Double {
        return angleAnalogInput.voltage/angleAnalogInput.maxVoltage * 2 * Math.PI
    }
}

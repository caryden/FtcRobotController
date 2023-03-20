package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.AnalogInput
import kotlinx.coroutines.*
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

class LeapfrogTurnServo(private val turnMotor: CRServo, private val angleAnalogInput: AnalogInput, private val dispatcher: CoroutineDispatcher = Dispatchers.Default) {
    private val turnPIDController = PIDController(5.0, 0.0, 0.0)

    private var servoWrapAngleOffset = 0.0
    private var servoAngleOffset = 0.0
    private var currentWrappedServoAngle = 0.0

    private var targetModuleAngle = 0.0
    private var currentModuleAngle = 0.0
    private var scope = CoroutineScope(dispatcher)

    var moduleAngle : Double
        get() = currentModuleAngle
        set(value) {
            targetModuleAngle = value
        }

    fun initialize() {
        zeroServoAngle()
    }
    fun startControlLoop() {
        scope.launch {
            while (isActive) {
                runTurnMotorPID()
                delay(20)
            }
            turnMotor.set(0.0)
        }
    }
    fun stopControlLoop() {
       scope.cancel()
    }
    private fun runTurnMotorPID()  {
        val unwrappedServoAngle = getUnwrappedServoAngle()
        val newWrappedServoAngle =  servoWrapAngleOffset + unwrappedServoAngle
        val telemetry = FtcDashboard.getInstance().telemetry

        // if the new angle is more than pi radians away from the current angle, add or subtract 2pi radians to the offset
        // this is handle the wrapping of the servo angle due to the servo gear ratio
        if (abs(newWrappedServoAngle - currentWrappedServoAngle) > Math.PI)
            servoWrapAngleOffset += 2 * Math.PI * -sign(newWrappedServoAngle - currentWrappedServoAngle)
        currentWrappedServoAngle = servoWrapAngleOffset + unwrappedServoAngle

        val gearRatio = 24.0/60.0
        currentModuleAngle = currentWrappedServoAngle * gearRatio

        // run PIDController to control the turnMotor Power until the measured angle is the desired angle (swerveModuleState.angle)
        // note that we are using the currentModuleAngle as the measured angle, and swerveModuleState.angle as the desired angle
        val turnMotorPower = max(-1.0, min(1.0, turnPIDController.calculate(currentModuleAngle, targetModuleAngle)))
        turnMotor.set(turnMotorPower)

        telemetry.addData("unwrappedServoAngle", unwrappedServoAngle)
        telemetry.addData("currentWrappedServoAngle", currentWrappedServoAngle)
        telemetry.addData("currentModuleAngle", currentModuleAngle)
        telemetry.addData("targetModuleAngle", targetModuleAngle)
        telemetry.update()
    }
    private fun getUnwrappedServoAngle(): Double {
        return angleAnalogInput.voltage/angleAnalogInput.maxVoltage * 2 * Math.PI - servoAngleOffset
    }
    private fun zeroServoAngle() {
        servoAngleOffset = 0.0
        servoAngleOffset = getUnwrappedServoAngle()
    }
}

//fun Double.compareWithTolerance(other: Double, tolerance :Double = 1e-9 ): Boolean {
//    return abs(this - other) < tolerance
//}
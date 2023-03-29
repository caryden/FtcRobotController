package org.firstinspires.ftc.teamcode.tests.experiments

import com.qualcomm.robotcore.hardware.AnalogInput
import org.junit.Test
import kotlin.math.abs
import kotlin.math.sign

class WrappingLogic {
    private var servoWrapAngleOffset = 0.0
    private var currentWrappedServoAngle = 0.0

    private var currentModuleAngle = 0.0
    private var targetModuleAngle = 0.0

    private var initialized = false
    private var initialModuleAngle = 0.0
    private val gearRatio = 24.0/60.0

    fun initialize() {
        servoWrapAngleOffset = 0.0
        currentModuleAngle = 0.118
        initialModuleAngle = currentModuleAngle

        updateCurrentModuleAngle()
        initialized = true
    }

    private fun updateCurrentModuleAngle() {
        val unwrappedServoAngle = getUnwrappedServoAngle(6.76)
        val newWrappedServoAngle =  servoWrapAngleOffset + unwrappedServoAngle

        // if the new angle is more than pi radians away from the current angle, add or subtract 2pi radians to the offset
        // this is handle the wrapping of the servo angle due to the servo gear ratio
        if (abs(newWrappedServoAngle - currentWrappedServoAngle) > Math.PI)
            servoWrapAngleOffset += 2 * Math.PI * -sign(newWrappedServoAngle - currentWrappedServoAngle)

        currentWrappedServoAngle = servoWrapAngleOffset + unwrappedServoAngle
        currentModuleAngle = (currentWrappedServoAngle * gearRatio - initialModuleAngle) % (2 * Math.PI)
    }
    private fun getUnwrappedServoAngle(angle: Double): Double {
        return Math.toRadians(angle)
    }

    @Test
    fun test_initial_angle_setting(){
        initialize()
        print(currentModuleAngle)
    }

}
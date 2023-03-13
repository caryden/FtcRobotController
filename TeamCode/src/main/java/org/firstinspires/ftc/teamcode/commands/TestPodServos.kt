package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.util.function.DoubleSupplier

class TestPodServos(private val allServos : MotorGroup,
                    private val forward : DoubleSupplier,
                    private val fLA :AnalogInput, private val fRA :AnalogInput, private val bLA :AnalogInput, private val bRA :AnalogInput,
                    private val telemetry: Telemetry ): CommandBase() {
    override fun execute() {
        allServos.set(forward.asDouble)

        telemetry.addData("frontLeftServoAngle", fLA.voltage/fLA.maxVoltage * 360.0 )
        telemetry.addData("frontRightServoAngle", fRA.voltage/fRA.maxVoltage * 360.0 )
        telemetry.addData("backLeftServoAngle", bLA.voltage/bLA.maxVoltage * 360.0 )
        telemetry.addData("backRightServoAngle", bRA.voltage/bRA.maxVoltage * 360.0 )
        telemetry.update()
    }
}
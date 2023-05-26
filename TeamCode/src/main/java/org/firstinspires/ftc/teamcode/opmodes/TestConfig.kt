package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.subsystems.DummySubsystem

@TeleOp
class TestConfig() : LinearOpMode() {

    override fun runOpMode() {
        var index = 0
        val t = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // let's get the drive motors
        val frontLeftDrive = MotorEx(hardwareMap, "frontLeftDrive", Motor.GoBILDA.BARE)
        val frontRRightDrive = MotorEx(hardwareMap, "frontRightDrive", Motor.GoBILDA.BARE)
        val backLeftDrive = MotorEx(hardwareMap, "backLeftDrive", Motor.GoBILDA.BARE)
        val backRightDrive = MotorEx(hardwareMap, "backRightDrive", Motor.GoBILDA.BARE)

        val motors = listOf(frontLeftDrive, frontRRightDrive, backLeftDrive, backRightDrive)

        val frontLeftCRServo = CRServo(hardwareMap, "frontLeftServo")
        val frontRightCRServo = CRServo(hardwareMap, "frontRightServo")
        val backLeftCRServo = CRServo(hardwareMap, "backLeftServo")
        val backRightCRServo = CRServo(hardwareMap, "backRightServo")

        val servos = listOf(frontLeftCRServo, frontRightCRServo, backLeftCRServo, backRightCRServo)

        val frontLeftAnalogInput = hardwareMap.get(AnalogInput::class.java, "frontLeftServoAngle")
        val frontRightAnalogInput = hardwareMap.get(AnalogInput::class.java, "frontRightServoAngle")
        val backLeftAnalogInput = hardwareMap.get(AnalogInput::class.java,"backLeftServoAngle")
        val backRightAnalogInput = hardwareMap.get(AnalogInput::class.java,"backRightServoAngle")

        var inputs = listOf<AnalogInput>(frontLeftAnalogInput, frontRightAnalogInput, backLeftAnalogInput, backRightAnalogInput)

        val driverOp = GamepadEx(gamepad1)

        val incrementButton = GamepadButton(driverOp, GamepadKeys.Button.DPAD_UP)
        val decrementButton = GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN)



        waitForStart()
        while(opModeIsActive() && !isStopRequested){
            driverOp.readButtons()

            if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                index++
                motors.forEach { it.velocity = 0.0 }
                servos.forEach { it.set(0.0) }
                motors[index % 4].velocity = 5.0
                servos[index % 4].set(1.0)
            } else if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                index--
                motors.forEach { it.velocity = 0.0 }
                servos.forEach { it.set(0.0) }
                motors[index % 4].velocity = 2.0
                servos[index % 4].set(1.0)
            }

            t.addData("Index", index % 4)
            t.addData("Voltage", inputs[index%4].voltage)
            t.update()
        }
    }

}
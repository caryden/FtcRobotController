package org.firstinspires.ftc.teamcode.opmodes

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.TestDrive
import org.firstinspires.ftc.teamcode.subsystems.LeapfrogDriveBase
import java.util.function.DoubleSupplier


@TeleOp(name = "Swerve Initial Test", group = "none")
public class LeapfrogInitialTestOpmode() : CommandOpMode() {

    override fun initialize() {

        // let's get the drive motors
        var fL = MotorEx(hardwareMap, "frontLeftDrive", Motor.GoBILDA.BARE)
        var fR = MotorEx(hardwareMap, "frontRightDrive", Motor.GoBILDA.BARE)
        var bL = MotorEx(hardwareMap, "backLeftDrive", Motor.GoBILDA.BARE)
        var bR = MotorEx(hardwareMap, "backRightDrive", Motor.GoBILDA.BARE)

        val leapfrogDrive = LeapfrogDriveBase(fL, fR, bL, bR);
        val driverOp = GamepadEx(gamepad1);
        val command = TestDrive(leapfrogDrive, DoubleSupplier { driverOp.leftY });

        schedule(command);
    }


}
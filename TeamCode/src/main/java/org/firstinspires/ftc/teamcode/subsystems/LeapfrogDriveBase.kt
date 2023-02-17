package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.hardware.motors.MotorGroup

class LeapfrogDriveBase(private val frontLeftDrive : MotorEx,
                        private val frontRightDrive : MotorEx,
                        private val backLeftDrive : MotorEx,
                        private val backRightDrive : MotorEx
) : SubsystemBase() {
    private val motors = MotorGroup(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

    init {
        motors.setRunMode(Motor.RunMode.RawPower);
    }
    fun drive(forward : Double) {
        motors.set(forward);
    }
    fun stop() {
        motors.set(0.0);
    }
}
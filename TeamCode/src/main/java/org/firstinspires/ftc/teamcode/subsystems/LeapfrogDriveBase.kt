package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.qualcomm.robotcore.hardware.AnalogInput

class LeapfrogDriveBase(private val frontLeftDrive : MotorEx,
                        private val frontRightDrive : MotorEx,
                        private val backLeftDrive : MotorEx,
                        private val backRightDrive : MotorEx,
                        private val frontLeftServo : CRServo,
                        private val frontRightServo : CRServo,
                        private val backLeftServo : CRServo,
                        private val backRightServo : CRServo,
                        private val frontLeftServoAngle : AnalogInput,
                        private val frontRightServoAngle : AnalogInput,
                        private val backLeftServoAngle : AnalogInput,
                        private val backRightServoAngle : AnalogInput) : SubsystemBase() {

    // The locations of the swerve modules relative to the robot center (measurements in meters, taken from OnShape)
    private val frontLeftLocation = Translation2d(0.132665, 0.132665)
    private val frontRightLocation = Translation2d(0.132665, -0.132665)
    private val backLeftLocation = Translation2d(-0.132665, 0.132665)
    private val backRightLocation = Translation2d(-0.132665, -0.132665)

    // The kinematics object that converts between chassis speeds and module states
    private val kinematics = SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation)
    private val frontLeftSwerveModule = SwerveModule(frontLeftDrive, frontLeftServo, frontLeftServoAngle)
    private val frontRightSwerveModule = SwerveModule(frontRightDrive, frontRightServo, frontRightServoAngle)
    private val backLeftSwerveModule = SwerveModule(backLeftDrive, backLeftServo, backLeftServoAngle)
    private val backRightSwerveModule = SwerveModule(backRightDrive, backRightServo, backRightServoAngle)


    fun drive(forward : Double) {

    }
    fun stop() {
        frontLeftSwerveModule.setSwerveModuleState(SwerveModuleState(0.0, Rotation2d()))
    }
    fun init() {
        // this will assume that all modules are point forward (+x) and will assign this the 0 degree position
        frontLeftSwerveModule.init()
        frontRightSwerveModule.init()
        backLeftSwerveModule.init()
        backRightSwerveModule.init()
    }
}
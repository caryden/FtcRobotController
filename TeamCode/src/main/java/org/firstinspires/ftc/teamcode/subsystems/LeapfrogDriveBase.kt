package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
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

    // The swerve modules
    private val frontLeftSwerveModule = LeapfrogSwerveModule(frontLeftDrive, frontLeftServo, frontLeftServoAngle)
    private val frontRightSwerveModule = LeapfrogSwerveModule(frontRightDrive, frontRightServo, frontRightServoAngle)
    private val backLeftSwerveModule = LeapfrogSwerveModule(backLeftDrive, backLeftServo, backLeftServoAngle)
    private val backRightSwerveModule = LeapfrogSwerveModule(backRightDrive, backRightServo, backRightServoAngle)

    fun drive(chassisSpeeds: ChassisSpeeds) {
        // Convert the desired chassis speeds into module states
        val moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds)

        // Set the module states
        frontLeftSwerveModule.moduleState = moduleStates[0]
        frontRightSwerveModule.moduleState = moduleStates[1]
        backLeftSwerveModule.moduleState = moduleStates[2]
        backRightSwerveModule.moduleState = moduleStates[3]
    }
    fun init() {
        // this will assume that all modules are point forward (+x) and will assign this the 0 degree position
        frontLeftSwerveModule.initialize().startControlLoop()
        frontRightSwerveModule.initialize().startControlLoop()
        backLeftSwerveModule.initialize().startControlLoop()
        backRightSwerveModule.initialize().startControlLoop()
    }
}
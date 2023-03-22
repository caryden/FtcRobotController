package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import com.qualcomm.robotcore.hardware.AnalogInput

class SwerveDriveBase(frontLeftSwerveModule : SwerveModule,
                      frontRightSwerveModule : SwerveModule,
                      backLeftSwerveModule : SwerveModule,
                      backRightSwerveModule: SwerveModule) : SubsystemBase() {
    constructor(frontLeftDrive : MotorEx, frontRightDrive : MotorEx, backLeftDrive : MotorEx, backRightDrive : MotorEx,
                frontLeftServo : CRServo, frontRightServo : CRServo, backLeftServo : CRServo, backRightServo : CRServo,
                frontLeftServoAngle : AnalogInput, frontRightServoAngle : AnalogInput,
                backLeftServoAngle : AnalogInput, backRightServoAngle : AnalogInput)
            : this(SwerveModule(frontLeftDrive, frontLeftServo, frontLeftServoAngle),
                   SwerveModule(frontRightDrive, frontRightServo, frontRightServoAngle),
                   SwerveModule(backLeftDrive, backLeftServo, backLeftServoAngle),
                   SwerveModule(backRightDrive, backRightServo, backRightServoAngle))

    // The locations of the swerve modules relative to the robot center (measurements in meters, taken from OnShape)
    // TODO: these might need to be moved to SwerveDriveConfiguration.kt
    private val frontLeftLocation = Translation2d(0.132665, 0.132665)
    private val frontRightLocation = Translation2d(0.132665, -0.132665)
    private val backLeftLocation = Translation2d(-0.132665, 0.132665)
    private val backRightLocation = Translation2d(-0.132665, -0.132665)

    // The kinematics object that converts between chassis speeds and module states
    private val kinematics = SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation)
    private val swerveModules = arrayOf(frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule)

    init {
        // invert the left drive motors
        frontLeftSwerveModule.driveMotor.inverted = true
        backLeftSwerveModule.driveMotor.inverted = true
    }

    fun drive(chassisSpeeds: ChassisSpeeds) {
        // Convert the desired chassis speeds into module states
        val moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds)
        swerveModules.forEachIndexed { index, swerveModule -> swerveModule.moduleState = moduleStates[index] }
    }

    fun initialize() {
        // this will assume that all modules are point forward (+x) and will assign this the 0 degree position
        // to be called in the OpMode initialize() method
        swerveModules.forEach { it.initialize() }
     }
}
package org.firstinspires.ftc.teamcode.subsystems

import DashboardUtils
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveConfiguration.*
import org.firstinspires.ftc.teamcode.utils.SwerveUtils


class SwerveDriveBase(
    frontLeftSwerveModule: SwerveModule,
    frontRightSwerveModule: SwerveModule,
    backLeftSwerveModule: SwerveModule,
    backRightSwerveModule: SwerveModule,
    private val gyroAngleProvider: () -> Rotation2d
) : SubsystemBase() {
    constructor(
        frontLeftDrive: MotorEx,
        frontRightDrive: MotorEx,
        backLeftDrive: MotorEx,
        backRightDrive: MotorEx,
        frontLeftServo: CRServo,
        frontRightServo: CRServo,
        backLeftServo: CRServo,
        backRightServo: CRServo,
        frontLeftServoAngle: AnalogInput,
        frontRightServoAngle: AnalogInput,
        backLeftServoAngle: AnalogInput,
        backRightServoAngle: AnalogInput,
        gyroAngleProvider: () -> Rotation2d
    )
            : this(
        SwerveModule(frontLeftDrive, frontLeftServo, frontLeftServoAngle, frontLeftKp, frontLeftKi, frontLeftKd),
        SwerveModule(frontRightDrive, frontRightServo, frontRightServoAngle, frontRightKp, frontRightKi, frontRightKd),
        SwerveModule(backLeftDrive, backLeftServo, backLeftServoAngle, backLeftKp, backLeftKi, backLeftKd),
        SwerveModule(backRightDrive, backRightServo, backRightServoAngle, backRightKp, backRightKi, backRightKd),
        gyroAngleProvider
    )

    // The locations of the swerve modules relative to the robot center (measurements in meters, taken from OnShape)
    // TODO: these might need to be moved to SwerveDriveConfiguration.kt

    private val dashboard = FtcDashboard.getInstance()

    // The kinematics object that converts between chassis speeds and module states
    private val kinematics = SwerveDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation
    )
    private val swerveModules = arrayOf(
        frontLeftSwerveModule,
        frontRightSwerveModule,
        backLeftSwerveModule,
        backRightSwerveModule
    )
    private val odometry =
        com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry(
            kinematics,
            gyroAngleProvider()
        )
    private var moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds(0.0, 0.0, 0.0))
    private var externalHeading: Double = 0.0
    init {
        register()

        // invert the right drive motors
        frontRightSwerveModule.driveMotor.inverted = true
        backRightSwerveModule.driveMotor.inverted = true
    }

    override fun periodic() {
        // get the current time in seconds
        val currentTime = System.currentTimeMillis() / 1000.0
        odometry.updateWithTime(
            currentTime,
            gyroAngleProvider(),
            moduleStates[0],
            moduleStates[1],
            moduleStates[2],
            moduleStates[3]
        )

        dashboard.telemetry.addData("FL Target Velocity", moduleStates[0].speedMetersPerSecond)
        dashboard.telemetry.addData("FL Actual Velocity", swerveModules[0].driveMotor.velocity)
        dashboard.telemetry.addData("FR Target Velocity", moduleStates[1].speedMetersPerSecond)
        dashboard.telemetry.addData("FR Actual Velocity", swerveModules[1].driveMotor.velocity)
        dashboard.telemetry.addData("BL Target Velocity", moduleStates[2].speedMetersPerSecond)
        dashboard.telemetry.addData("BL Actual Velocity", swerveModules[2].driveMotor.velocity)
        dashboard.telemetry.addData("BR Target Velocity", moduleStates[3].speedMetersPerSecond)
        dashboard.telemetry.addData("BR Actual Velocity", swerveModules[3].driveMotor.velocity)

        dashboard.telemetry.update()
        DashboardUtils.drawServoModules(swerveModules, dashboard)
    }

    fun drive(chassisSpeeds: ChassisSpeeds) {
        // Convert the desired chassis speeds into module states
        moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds)
        swerveModules.forEachIndexed { index, swerveModule ->
            swerveModule.moduleState =
                SwerveUtils.optimize(moduleStates[index], swerveModule.moduleState.angle)
        }
    }

    fun setDriveSignal(signal: DriveSignal){

    }

    fun initialize(initialPose: Pose2d) {
        // this will assume that all modules are point forward (+x) and will assign this the 0 degree position
        // to be called in the OpMode initialize() method
        swerveModules.forEach { it.initialize() }
        odometry.resetPosition(initialPose, gyroAngleProvider())
    }

    // TODO: Finish implementing roadrunner stuff
    class SwerveLocalizer @JvmOverloads constructor(
        private val drive: SwerveDriveBase,
        private val useExternalHeading:Boolean = true,
    ) : Localizer {
        private var _poseEstimate = com.acmerobotics.roadrunner.geometry.Pose2d()
        override var poseEstimate: com.acmerobotics.roadrunner.geometry.Pose2d
            get() = _poseEstimate
            set(value) {
                lastModuleStates = emptyList()
                lastExtHeading = Double.NaN
                if (useExternalHeading) drive.externalHeading = value.heading
                _poseEstimate = value
            }
        override var poseVelocity: com.acmerobotics.roadrunner.geometry.Pose2d? = null
            private set
        private var lastModuleStates = emptyList<Double>()
        private var lastExtHeading = Double.NaN

        override fun update() {
            TODO("Not yet implemented")
            // Find the delta in module states now vs last loop
            // Feed that data to kinematics, find change in position
            // Add that change in position to old position to get new estimate
        }

    }


}
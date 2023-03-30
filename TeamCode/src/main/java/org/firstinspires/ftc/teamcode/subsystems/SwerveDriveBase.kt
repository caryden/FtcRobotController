package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
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
        SwerveModule(frontLeftDrive, frontLeftServo, frontLeftServoAngle),
        SwerveModule(frontRightDrive, frontRightServo, frontRightServoAngle),
        SwerveModule(backLeftDrive, backLeftServo, backLeftServoAngle),
        SwerveModule(backRightDrive, backRightServo, backRightServoAngle),
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
    private val swerveOdometry =
        com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry(
            kinematics,
            gyroAngleProvider()
        )
    private var moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds(0.0, 0.0, 0.0))

    init {
        register()

        // invert the right drive motors
        frontRightSwerveModule.driveMotor.inverted = true
        backRightSwerveModule.driveMotor.inverted = true
    }

    override fun periodic() {
        // get the current time in seconds
        val currentTime = System.currentTimeMillis() / 1000.0
        swerveOdometry.updateWithTime(
            currentTime,
            gyroAngleProvider(),
            moduleStates[0],
            moduleStates[1],
            moduleStates[2],
            moduleStates[3]
        )

//        dashboard.telemetry.addData("FL Angle", swerveModules[0].turnMotor.moduleAngle)
//        dashboard.telemetry.addData(
//            "FL Theoretical Angle",
//            swerveModules[0].moduleState.angle.radians
//        )
//
//        dashboard.telemetry.addData("FR Angle", swerveModules[1].turnMotor.moduleAngle)
//        dashboard.telemetry.addData(
//            "FL Theoretical Angle",
//            swerveModules[1].moduleState.angle.radians
//        )
//
//        dashboard.telemetry.addData("BL Angle", swerveModules[2].turnMotor.moduleAngle)
//        dashboard.telemetry.addData(
//            "BL Theoretical Angle",
//            swerveModules[2].moduleState.angle.radians
//        )
//
//        dashboard.telemetry.addData("BR Angle", swerveModules[3].turnMotor.moduleAngle)
//        dashboard.telemetry.addData(
//            "BR Theoretical Angle",
//            swerveModules[3].moduleState.angle.radians
//        )

        dashboard.telemetry.addData("FL Target Velocity", moduleStates[0].speedMetersPerSecond)
        dashboard.telemetry.addData("FL Actual Velocity", swerveModules[0].driveMotor.velocity)
        dashboard.telemetry.addData("FR Target Velocity", moduleStates[1].speedMetersPerSecond)
        dashboard.telemetry.addData("FR Actual Velocity", swerveModules[1].driveMotor.velocity)
        dashboard.telemetry.addData("BL Target Velocity", moduleStates[2].speedMetersPerSecond)
        dashboard.telemetry.addData("BL Actual Velocity", swerveModules[2].driveMotor.velocity)
        dashboard.telemetry.addData("BR Target Velocity", moduleStates[3].speedMetersPerSecond)
        dashboard.telemetry.addData("BR Actual Velocity", swerveModules[3].driveMotor.velocity)

        dashboard.telemetry.update()

        drawServoModules()
    }

    fun drive(chassisSpeeds: ChassisSpeeds) {
        // Convert the desired chassis speeds into module states
        moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds)
        swerveModules.forEachIndexed { index, swerveModule ->
            swerveModule.moduleState =
                SwerveUtils.optimize(moduleStates[index], swerveModule.moduleState.angle)
        }
    }

    fun initialize(initialPose: Pose2d) {
        // this will assume that all modules are point forward (+x) and will assign this the 0 degree position
        // to be called in the OpMode initialize() method
        swerveModules.forEach { it.initialize() }
        swerveOdometry.resetPosition(initialPose, gyroAngleProvider())
    }

    val x0 = 0.0
    val y0 = 0.0
    val xOffset = 30.0
    val yOffset = 30.0
    val radius = 10.0

    fun drawServoModules() {
        val packet = TelemetryPacket()
        packet
            .fieldOverlay()
            // frontLeft
            .setStroke("green")
            .strokeCircle(x0 + xOffset, y0 + yOffset, radius)
            .strokeLine(
                x0 + xOffset, y0 + yOffset,
                getXCoord(x0 + xOffset, swerveModules[0].turnMotor.moduleAngle),
                getYCoord(y0 + yOffset, swerveModules[0].turnMotor.moduleAngle)
            )

            // frontRight
            .setStroke("blue")
            .strokeCircle(x0 + xOffset, y0 - yOffset, radius)
            .strokeLine(
                x0 + xOffset, y0 - yOffset,
                getXCoord(x0 + xOffset, swerveModules[1].turnMotor.moduleAngle),
                getYCoord(y0 - yOffset, swerveModules[1].turnMotor.moduleAngle)
            )

            // backLeft
            .setStroke("orange")
            .strokeCircle(x0 - xOffset, y0 + yOffset, radius)
            .strokeLine(
                x0 - xOffset, y0 + yOffset,
                getXCoord(x0 - xOffset, swerveModules[2].turnMotor.moduleAngle),
                getYCoord(y0 + yOffset, swerveModules[2].turnMotor.moduleAngle)
            )

            // backRight
            .setStroke("purple")
            .strokeCircle(x0 - xOffset, y0 - yOffset, radius)
            .strokeLine(
                x0 - xOffset, y0 - yOffset,
                getXCoord(x0 - xOffset, swerveModules[3].turnMotor.moduleAngle),
                getYCoord(y0 - yOffset, swerveModules[3].turnMotor.moduleAngle)
            )

        dashboard.sendTelemetryPacket(packet)
    }

    private fun getXCoord(x: Double, angle: Double): Double {
        return x + radius * kotlin.math.cos(angle)
    }

    private fun getYCoord(y: Double, angle: Double): Double {
        return y + radius * kotlin.math.sin(angle)
    }


}
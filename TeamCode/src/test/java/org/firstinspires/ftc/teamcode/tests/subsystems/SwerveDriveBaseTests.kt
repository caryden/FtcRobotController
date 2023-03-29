package org.firstinspires.ftc.teamcode.tests.subsystems

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveConfiguration
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule
import org.firstinspires.ftc.teamcode.subsystems.SwerveModuleDriveMotor
import org.junit.Test
import org.mockito.kotlin.*

class SwerveDriveBaseTests {
    val swerveModuleDriveMotor = mock<SwerveModuleDriveMotor>() {
        on { inverted } doAnswer { b -> false }
    }
    val frontLeftSwerveModule = mock<SwerveModule>() {
        on { driveMotor } doReturn (swerveModuleDriveMotor)
    }
    val frontRightSwerveModule = mock<SwerveModule>()
    val backLeftSwerveModule = mock<SwerveModule>() {
        on { driveMotor } doReturn (swerveModuleDriveMotor)
    }
    val backRightSwerveModule = mock<SwerveModule>()
    val swerveModules = arrayOf(
        frontLeftSwerveModule,
        frontRightSwerveModule,
        backLeftSwerveModule,
        backRightSwerveModule
    )

    @Test
    fun drive_should_set_module_states() {
        val swerveDriveBase = SwerveDriveBase(
            frontLeftSwerveModule,
            frontRightSwerveModule,
            backLeftSwerveModule,
            backRightSwerveModule
        ) { Rotation2d(0.0) }
        val chassisSpeeds = ChassisSpeeds(0.1, 0.2, 0.3)
        swerveDriveBase.drive(chassisSpeeds)
        swerveModules.forEach { verify(it).moduleState = any() }
    }

    @Test
    fun initialize_should_initialize_swervemodules() {
        val swerveDriveBase = SwerveDriveBase(
            frontLeftSwerveModule,
            frontRightSwerveModule,
            backLeftSwerveModule,
            backRightSwerveModule
        ) { Rotation2d(0.0) }
        swerveDriveBase.initialize(Pose2d(0.0, 0.0, Rotation2d(0.0)))
        swerveModules.forEach { verify(it).initialize() }
    }

    @Test
    fun kinematicsTest() {
        val kinematics = SwerveDriveKinematics(
            SwerveDriveConfiguration.frontLeftLocation,
            SwerveDriveConfiguration.frontRightLocation,
            SwerveDriveConfiguration.backLeftLocation,
            SwerveDriveConfiguration.backRightLocation
        )
        val straightForwardSpeeds = ChassisSpeeds(1.0, 1.0, 0.0)
        val moduleStates = kinematics.toSwerveModuleStates(straightForwardSpeeds)

        val swerveDriveBase = SwerveDriveBase(
            frontLeftSwerveModule,
            frontRightSwerveModule,
            backLeftSwerveModule,
            backRightSwerveModule
        ) { Rotation2d(0.0) }
        swerveDriveBase.initialize(Pose2d(0.0, 0.0, Rotation2d(0.0)))

        swerveDriveBase.drive(straightForwardSpeeds)
        print(frontLeftSwerveModule.turnMotor.moduleAngle)



    }


}
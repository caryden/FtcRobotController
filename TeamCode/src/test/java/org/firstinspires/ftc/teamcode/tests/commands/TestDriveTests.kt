package org.firstinspires.ftc.teamcode.tests.commands

import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import com.qualcomm.robotcore.hardware.AnalogInput
import junit.framework.TestCase.assertEquals
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.commands.TestDrive
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveConfiguration
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule
import org.junit.Test
import org.mockito.kotlin.argThat
import org.mockito.kotlin.mock
import org.mockito.kotlin.verify
import java.util.function.DoubleSupplier

class TestDriveTests {

    @Test
    fun execute() {
        val forwardValue = 0.5
        val chassisSpeeds = ChassisSpeeds(forwardValue, 0.0, 0.0)

        val leapfrogDriveBase = mock<SwerveDriveBase>()

        val testDrive =
            TestDrive(leapfrogDriveBase, DoubleSupplier { forwardValue }, mock<Telemetry>())
        testDrive.execute()
        verify(leapfrogDriveBase).drive(argThat { this.vxMetersPerSecond == chassisSpeeds.vxMetersPerSecond })
    }

    @Test
    fun testThatMotorsActuallySpin() {
        val forwardValue = 0.5
        val chassisSpeeds = ChassisSpeeds(forwardValue, 0.0, 0.0)

        val frontLDrive: MotorEx = mock<MotorEx>()
        val frontLServo: CRServo = mock<CRServo>()
        val frontLServoAngle: AnalogInput = mock<AnalogInput>()

        val frontLModule: SwerveModule = SwerveModule(frontLDrive, frontLServo, frontLServoAngle)
        val frontRModule: SwerveModule = mock<SwerveModule>()
        val backRModule: SwerveModule = mock<SwerveModule>()
        val backLModule: SwerveModule = mock<SwerveModule>()

        val drive = SwerveDriveBase(frontLModule, frontRModule, backLModule, backRModule) {
            Rotation2d(
                0.0
            )
        }

        val testDrive = TestDrive(drive, { forwardValue }, mock<Telemetry>())
        testDrive.execute()
        assertEquals(forwardValue, frontLDrive.velocity)
    }

    @Test
    fun testOmegaAngles() {

        val chassisSpeeds = ChassisSpeeds(0.0, 0.0, 6.0)
        val kinematics = SwerveDriveKinematics(
            SwerveDriveConfiguration.frontLeftLocation,
            SwerveDriveConfiguration.frontRightLocation,
            SwerveDriveConfiguration.backLeftLocation,
            SwerveDriveConfiguration.backRightLocation
        )
        val moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds)
        moduleStates.forEach { println(it) }


    }
}
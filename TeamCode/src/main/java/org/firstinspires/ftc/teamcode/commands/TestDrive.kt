package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase
import java.util.function.DoubleSupplier

class TestDrive(private val leapfrog : SwerveDriveBase, private val forward : DoubleSupplier, private val telemetry: Telemetry) : CommandBase() {
    init {
        addRequirements(leapfrog)
        telemetry.addData("initialized", " ")

    }
    override fun initialize() {
          leapfrog.initialize(Pose2d(0.0, 0.0, Rotation2d(0.0)))
    }
    override fun execute() {
        // just drive forward/back +/- X-direction
        val chassisSpeeds = ChassisSpeeds(forward.asDouble, 0.0, 0.0)
        leapfrog.drive(chassisSpeeds)

        telemetry.addData("Chassis Speeds", chassisSpeeds.toString())
        telemetry.update()
    }
    override fun isFinished(): Boolean {
        return false
    }
}
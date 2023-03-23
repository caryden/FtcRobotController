package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase

class SwerveDrive(private val leapfrog: SwerveDriveBase,
                  private val initialPose : Pose2d,
                  private val chassisSpeedsProvider : () -> ChassisSpeeds) : CommandBase() {
    init {
        addRequirements(leapfrog)
    }
    override fun initialize() {
        leapfrog.initialize(initialPose)
    }
    override fun execute() {
        leapfrog.drive(chassisSpeedsProvider())
    }
}
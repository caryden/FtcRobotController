package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase
import java.util.function.DoubleSupplier

class TestDrive(private val leapfrog : SwerveDriveBase, private val forward : DoubleSupplier) : CommandBase() {
    init {
        addRequirements(leapfrog);
    }
    override fun initialize() {
          leapfrog.initialize()
    }
    override fun execute() {
        // just drive forward/back +/- X-direction
        val chassisSpeeds = ChassisSpeeds(forward.asDouble, 0.0, 0.0)
        leapfrog.drive(chassisSpeeds)
    }
}
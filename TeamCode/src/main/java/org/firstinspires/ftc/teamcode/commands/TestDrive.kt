package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.LeapfrogDriveBase
import java.util.function.DoubleSupplier

class TestDrive(private val leapfrog : LeapfrogDriveBase, private val forward : DoubleSupplier) : CommandBase() {
    init {
        addRequirements(leapfrog);
    }
    override fun execute() {
        leapfrog.drive(forward.asDouble)
    }
}
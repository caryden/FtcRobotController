package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.LeapfrogDriveBase

class FieldRelativeDrive(private val leapfrogDriveBase: LeapfrogDriveBase) : CommandBase() {
    init {
        addRequirements(leapfrogDriveBase)
    }
    override fun execute() {

    }
}
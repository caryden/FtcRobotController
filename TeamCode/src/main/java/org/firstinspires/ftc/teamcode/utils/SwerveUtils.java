package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;

public class SwerveUtils {

    // Completely stolen from WPILib
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        Rotation2d delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SwerveDriveConfiguration {
    public static double wheelRevsPerMotorRev = 16.0 / 60.0 * 24.0 / 36.0 * 14.0 / 28.0;
    public static double wheelDiameter = 72.0 / 1000.0; // 72mm in meters
    public static double wheelCircumference = wheelDiameter * Math.PI;  // meters/wheel rev
    public static double getWheelMetersPerTick(double driveMotorCPR) {
        return wheelCircumference * wheelRevsPerMotorRev / driveMotorCPR;
    }
    public static double maxMotorRevs = 5000.0/60.0; // 5000 RPM in revs/second
    public static double maxAngularSpeed = 2 * Math.PI; // radians/second - 1 rev/second
    public static double maxWheelSpeed = maxMotorRevs * wheelRevsPerMotorRev * wheelCircumference; // meters/second
}

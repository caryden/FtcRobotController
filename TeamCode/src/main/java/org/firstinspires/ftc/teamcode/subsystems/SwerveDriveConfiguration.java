package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

/// Configuration for the swerve drive
/// This is a class so that we can use the @Config annotation to make it easy to change
/// the values in the dashboard.
///
/// Note that this is a java (not kotlin) class because I couldn't get the @Config annotation
/// to work in kotlin.
@Config
public class SwerveDriveConfiguration {
    public static double wheelRevsPerMotorRev = 16.0 / 60.0 * 24.0 / 36.0 * 14.0 / 28.0;
    public static double wheelDiameter = 72.0 / 1000.0; // 72mm in meters
    public static double wheelCircumference = wheelDiameter * Math.PI;  // meters/wheel rev
    public static double getWheelMetersPerTick(double driveMotorCPR) {
        return wheelCircumference * wheelRevsPerMotorRev / driveMotorCPR;
    }
    public static Translation2d frontLeftLocation = new Translation2d(0.132665, 0.132665);
    public static Translation2d frontRightLocation = new Translation2d(0.132665, -0.132665);
    public static Translation2d backLeftLocation = new Translation2d(-0.132665, 0.132665);
    public static Translation2d backRightLocation = new Translation2d(-0.132665, -0.132665);
    public static double maxMotorRevs = 5000.0/60.0; // 5000 RPM in revs/second
    public static double axonMaxPlusAt6VMaxRotationalSpeed = (60.0/360.0) * 2 * Math.PI / 0.115;  // spec is 0.115 secs/60 degrees converted to rad/sec
    public static double maxAngularSpeed = axonMaxPlusAt6VMaxRotationalSpeed;
    public static double maxWheelSpeed = maxMotorRevs * wheelRevsPerMotorRev * wheelCircumference; // meters/second

    public static double frontRightKp = 0.003;
    public static double frontRightKi = 0.0;
    public static double frontRightKd = 0.0;

    public static double backRightKp = 0.007;
    public static double backRightKi = 0.0;
    public static double backRightKd = 0.0;

    public static double frontLeftKp = 0.003;
    public static double frontLeftKi = 0.0;
    public static double frontLeftKd = 0.0;

    public static double backLeftKp = 0.003;
    public static double backLeftKi = 0.0;
    public static double backLeftKd = 0.0;

    public static double turnKp = 0.6;
}

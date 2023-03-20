package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.qualcomm.robotcore.hardware.AnalogInput

class LeapfrogSwerveModule(private val driveMotor: MotorEx, private val turnServo: LeapfrogTurnServo) {
   constructor(driveMotor: MotorEx, turnMotor: CRServo, angleAnalogInput: AnalogInput) : this(driveMotor, LeapfrogTurnServo(turnMotor, angleAnalogInput))
    init {
        driveMotor.setRunMode(Motor.RunMode.VelocityControl)
        initialize()
    }

    private var swerveModuleState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d(0.0))
    private val wheelRevsPerMotorRev = 16.0 / 60.0 * 24.0 / 36.0 * 14.0 / 28.0
    private val wheelDiameter = 72.0 / 1000.0 // 72mm in meters
    private val wheelCircumference = wheelDiameter * Math.PI  // meters/wheel rev
    private val wheelMetersPerTick = wheelCircumference * wheelRevsPerMotorRev / driveMotor.cpr

    var invertedDriveMotor : Boolean
        get() = driveMotor.inverted
        set(value) { driveMotor.inverted = value}

    /// The angle of the turn servo in radians
    val turnServoAngle : Double
        get() = turnServo.moduleAngle

    /// The velocity at the wheel in meters/second
    val wheelVelocity : Double
        get() = driveMotor.velocity * wheelMetersPerTick

    var moduleState : SwerveModuleState
        get() = swerveModuleState
        set(value) {
            this.swerveModuleState = value
            driveMotor.velocity = swerveModuleState.speedMetersPerSecond / wheelMetersPerTick
            turnServo.moduleAngle = swerveModuleState.angle.radians
        }

    fun initialize() : LeapfrogSwerveModule {
        turnServo.initialize()
        return this // allows chaining
    }
    fun startControlLoop() : LeapfrogSwerveModule {
        turnServo.startControlLoop()
        return this // allows chaining
    }

}
package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.qualcomm.robotcore.hardware.AnalogInput

class SwerveModule(private val driveMotor: MotorEx, private val turnServo: LeapfrogTurnServo) {
   constructor(driveMotor: MotorEx, turnMotor: CRServo, angleAnalogInput: AnalogInput) : this(driveMotor, LeapfrogTurnServo(turnMotor, angleAnalogInput))
    init {
        driveMotor.setRunMode(Motor.RunMode.VelocityControl)
        initialize()
    }

    private var swerveModuleState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d(0.0))
    private val wheelRevsPerMotorRev = SwerveDriveConfiguration.wheelRevsPerMotorRev
    private val wheelDiameter = SwerveDriveConfiguration.wheelDiameter
    private val wheelCircumference = SwerveDriveConfiguration.wheelCircumference
    private val wheelMetersPerTick = SwerveDriveConfiguration.getWheelMetersPerTick(driveMotor.cpr)

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
            updateMotors()
        }
    private var _isDriveMotorEnabled = true
    var isDriveMotorEnabled : Boolean
        get() = _isDriveMotorEnabled
        set(value) {
            _isDriveMotorEnabled = value
            updateMotors()
        }
    private var _isTurnServoEnabled = true
    var isTurnServoEnabled : Boolean
        get() = _isTurnServoEnabled
        set(value) {
            _isTurnServoEnabled = value
            updateMotors()
        }
    fun initialize() {
        turnServo.initialize()
    }
    fun startControlLoop() {
        turnServo.startControlLoop()
    }
    fun stopControlLoop() {
        turnServo.stopControlLoop()
    }
    private fun updateMotors() {
        if(_isDriveMotorEnabled)
            driveMotor.velocity = swerveModuleState.speedMetersPerSecond / wheelMetersPerTick
        else
            driveMotor.velocity = 0.0

        if(_isTurnServoEnabled) {
            turnServo.moduleAngle = swerveModuleState.angle.radians
            turnServo.startControlLoop()
        }
        else {
            turnServo.stopControlLoop()
        }

        FtcDashboard.getInstance().telemetry.addData("driveMotor.velocity:",driveMotor.velocity)
        FtcDashboard.getInstance().telemetry.addData("turnServo.moduleAngle:",turnServo.moduleAngle)
        FtcDashboard.getInstance().telemetry.update()
    }

}
package org.firstinspires.ftc.teamcode.roadrunner.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.SwerveKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase

class SwerveLocalizer @JvmOverloads constructor(
    private val drive: SwerveDriveBase,
    private val useExternalHeading: Boolean = true
) : Localizer {
    private var _poseEstimate = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            lastWheelPositions = emptyList()
            lastExtHeading = Double.NaN
            if (useExternalHeading) drive.getExternalHeading() = value.heading
            _poseEstimate = value
        }
    override var poseVelocity: Pose2d? = null
        private set
    private var lastWheelPositions = emptyList<Double>()
    private var lastExtHeading = Double.NaN

    override fun update() {
        val wheelPositions = drive.getWheelPositions()
        val moduleOrientations = drive.getModuleOrientations()
        val extHeading = if (useExternalHeading) drive.externalHeading else Double.NaN
        if (lastWheelPositions.isNotEmpty()) {
            val wheelDeltas = wheelPositions
                .zip(lastWheelPositions)
                .map { it.first - it.second }
            val robotPoseDelta = SwerveKinematics.wheelToRobotVelocities(
                wheelDeltas,
                moduleOrientations,
                drive.wheelBase,
                drive.trackWidth
            )
            val finalHeadingDelta = if (useExternalHeading) {
                Angle.normDelta(extHeading - lastExtHeading)
            } else {
                robotPoseDelta.heading
            }
            _poseEstimate = Kinematics.relativeOdometryUpdate(
                _poseEstimate,
                Pose2d(robotPoseDelta.vec(), finalHeadingDelta)
            )
        }

        val wheelVelocities = drive.getWheelVelocities()
        val extHeadingVel = drive.getExternalHeadingVelocity()
        if (wheelVelocities != null) {
            poseVelocity = SwerveKinematics.wheelToRobotVelocities(
                wheelVelocities,
                moduleOrientations,
                drive.wheelBase,
                drive.trackWidth
            )
            if (useExternalHeading && extHeadingVel != null) {
                poseVelocity = Pose2d(poseVelocity!!.vec(), extHeadingVel)
            }
        }

        lastWheelPositions = wheelPositions
        lastExtHeading = extHeading
    }
}
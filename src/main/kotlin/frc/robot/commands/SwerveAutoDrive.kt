package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import frc.lib.SwerveDriveBase
import frc.lib.SwerveDriveConstants
import frc.lib.rotation2dFromDeg
import frc.robot.Constants
import java.util.function.DoubleSupplier
import kotlin.math.abs

/*
SwerveAutoDrive, command for swerve driving which fuses setpoint following and remote control
allows the user to take manual control of the joysticks to make adjustments while also sending the robot to the setpoint
 */
class SwerveAutoDrive(
    private val setpoint: Pose2d,
    private val tolerance: Pose2d,
    private val drive: SwerveDriveBase,
    private val translationX: DoubleSupplier,
    private val translationY: DoubleSupplier,
    private val rotation: DoubleSupplier,
): Command() {
    init{
        addRequirements(drive)
    }
    override fun execute() {

        /*collect speeds based on which controls are used*/
        val controlsX = MathUtil.applyDeadband(translationX.asDouble, Constants.OperatorConstants.STICK_DEADBAND)
        val controlsY = MathUtil.applyDeadband(translationY.asDouble, Constants.OperatorConstants.STICK_DEADBAND)
        val controlsRot = MathUtil.applyDeadband(rotation.asDouble, Constants.OperatorConstants.STICK_DEADBAND)

        val speeds = Pose2d(
            if (controlsX > 0.1)
                controlsX* SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND
            else
                SwerveDriveConstants.DrivetrainConsts.xPIDController.calculate(drive.getPose().x, TrapezoidProfile.State(setpoint.x, 0.0)) * SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND,
            if (controlsY > 0.1)
                controlsY* SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND
            else
                SwerveDriveConstants.DrivetrainConsts.yPIDController.calculate(drive.getPose().y, TrapezoidProfile.State(setpoint.y, 0.0)) * SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND,
            if (controlsRot > 0.1)
                (controlsRot * SwerveDriveConstants.DrivetrainConsts.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND).rotation2dFromDeg()
            else
                (SwerveDriveConstants.DrivetrainConsts.xPIDController.calculate(drive.getPose().y, TrapezoidProfile.State(setpoint.y, 0.0)) * SwerveDriveConstants.DrivetrainConsts.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND).rotation2dFromDeg(),

        )

        /* Drive */
        drive.setModuleStates(drive.constructModuleStatesFromChassisSpeeds(speeds.x,speeds.y,speeds.rotation.degrees,true))

    }

    override fun isFinished(): Boolean {
        return (
        abs(drive.getPose().x - setpoint.x) < tolerance.x &&
        abs(drive.getPose().y - setpoint.y) < tolerance.y &&
        abs(drive.getPose().rotation.degrees - setpoint.rotation.degrees) < tolerance.rotation.degrees
                )
    }

    override fun end(interrupted: Boolean) {
    }

}
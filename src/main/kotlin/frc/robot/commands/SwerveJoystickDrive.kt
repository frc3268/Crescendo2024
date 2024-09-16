package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.math.MathUtil
import frc.lib.SwerveDriveBase
import frc.lib.SwerveDriveConstants
import java.util.function.*

import frc.robot.Constants

class SwerveJoystickDrive(
    private val drive: SwerveDriveBase,
    private val translationX: DoubleSupplier,
    private val translationY: DoubleSupplier,
    private val rotation: DoubleSupplier,
    private val fieldOriented: BooleanSupplier
) : Command() {
    //multiply x and y values from controller by this
    var sideMultiplier = 1

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() { 
        /* Get Values, Deadband, Convert to speeds */
        val xSpeed: Double = MathUtil.applyDeadband(translationX.asDouble, Constants.OperatorConstants.STICK_DEADBAND)* SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND
        val ySpeed: Double = MathUtil.applyDeadband(translationY.asDouble, Constants.OperatorConstants.STICK_DEADBAND) * SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND
        val turnSpeed: Double = MathUtil.applyDeadband(rotation.asDouble, Constants.OperatorConstants.STICK_DEADBAND) * SwerveDriveConstants.DrivetrainConsts.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND

        /* Drive */
        drive.setModuleStates(drive.constructModuleStatesFromChassisSpeeds(xSpeed,ySpeed,turnSpeed,fieldOriented.asBoolean))
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}

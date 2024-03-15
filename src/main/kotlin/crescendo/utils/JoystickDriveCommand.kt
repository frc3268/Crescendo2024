package crescendo.utils

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.math.MathUtil
import crescendo.subsystems.swerve.SwerveSubsystem
import java.util.function.*

/**
 * Runs during teleop. Listens for joystick input and executes it until the command is stopped.
 */
class JoystickDriveCommand(
    private val drive: SwerveSubsystem,
    private val translationX: DoubleSupplier,
    private val translationY: DoubleSupplier,
    private val rotation: DoubleSupplier,
    private val fieldOriented: BooleanSupplier
) : Command() {
    companion object {
        const val STICK_DEADBAND = 0.1
    }

    init {
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() { 
        /* Get Values, Deadband, Convert to speeds */
        val xSpeed: Double = MathUtil.applyDeadband(translationX.asDouble, STICK_DEADBAND) * Constants.MAX_SPEED_METERS_PER_SECOND
        val ySpeed: Double = MathUtil.applyDeadband(translationY.asDouble, STICK_DEADBAND) * Constants.MAX_SPEED_METERS_PER_SECOND
        val turnSpeed: Double = MathUtil.applyDeadband(rotation.asDouble, STICK_DEADBAND) * Constants.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND

        /* Drive */
        drive.setModuleStates(
            drive.constructModuleStatesFromChassisSpeeds(
                xSpeed,
                ySpeed,
                turnSpeed,
                fieldOriented.asBoolean
            )
        )
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    // Returns true when the command should end.
    override fun isFinished() = false
}

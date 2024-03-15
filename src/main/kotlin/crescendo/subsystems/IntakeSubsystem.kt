package crescendo.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.*
import crescendo.utils.*

/**
 * Represents the entire intake module, consisting of both the intake motor and the intake arm motor.
 *
 * Most methods return [Command]s, and they are grouped into intake-only, arm-only, and hybrid commands. Intake-only
 * means the intake runs but the arm doesn't move; arm-only means the arm moves but the intake doesn't; hybrid means
 * both may move.
 */
class IntakeSubsystem: SubsystemBase() {
    companion object {
        const val INTAKE_SPEED = 0.3
        const val OUTTAKE_ADJUST_SPEED = -0.3
        const val OUTTAKE_SPEED = -0.9
        const val SHOOT_AMP_SPEED = -1.0

        const val UP_ANGLE = 0.05
        const val DOWN_ANGLE = 170.0
    }

    private val intakeMotor = Motor(9)
    private val armMotor = Motor(10)
    private val armPIDController = PIDController(0.2/170,0.0,0.0)

    val shuffleboardTab = Shuffleboard.getTab("intake")
    val intakeArmPositionEntry = shuffleboardTab.add("Intake arm encoder position", 0.0).entry
    val intakeVelocityEntry = shuffleboardTab.add("Intake encoder velocity", 0.0).entry

    // TODO replace with actual channel
    val limitSwitch = DigitalInput(0)

    init {
        shuffleboardTab.add("Arm down - TESTING", runOnce { armMotor.set(0.1) })
        shuffleboardTab.add("Arm up - TESTING", runOnce { armMotor.set(-0.1) })
        shuffleboardTab.add("Intake in - TESTING", runIntakeAtSpeed(INTAKE_SPEED))
        shuffleboardTab.add("Intake out - TESTING", runIntakeAtSpeed(OUTTAKE_SPEED))
        shuffleboardTab.add("Intake sequence - TESTING", intakeAndStopCommand())

        shuffleboardTab.add("Arm down - sequence", armDownCommand())
        shuffleboardTab.add("Arm up - sequence", armUpCommand())

        shuffleboardTab.add("STOP - TESTING", stopAllCommand())
        armMotor.encoder.positionConversionFactor = 360 / 75.0
        intakeMotor.encoder.velocityConversionFactor = 1.0 /1600.0
    }

    fun zeroArmEncoderCommand() =
        runOnce { armMotor.encoder.position = 0.0 }

    fun getArmPosition() =
        Rotation2d.fromDegrees(armMotor.encoder.position)

    /*----------------------------------------------------------------------------------------------------------------*/
    // INTAKE-ONLY COMMANDS
    /*----------------------------------------------------------------------------------------------------------------*/

    /**
     * Runs the intake gears at [speed].
     */
    fun runIntakeAtSpeed(speed: Double): Command =
        runOnce { intakeMotor.set(speed) }

    fun runIntakeCommand() =
        runIntakeAtSpeed(INTAKE_SPEED)

    fun runOuttakeCommand() =
        runIntakeAtSpeed(OUTTAKE_ADJUST_SPEED)

    fun stopIntake(): Command =
        runOnce { intakeMotor.stopMotor() }

    /*----------------------------------------------------------------------------------------------------------------*/
    // ARM-ONLY COMMANDS
    /*----------------------------------------------------------------------------------------------------------------*/

    fun armUpCommand(): Command =
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, UP_ANGLE -5.0)) }
            .until { getArmPosition().degrees < UP_ANGLE }
            .andThen(armStopCommand())

    fun armDownCommand(): Command =
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, DOWN_ANGLE +5.0)) }
            .until { getArmPosition().degrees > DOWN_ANGLE }
            .andThen(armStopCommand())

    fun armStopCommand(): Command =
        runOnce { armMotor.stopMotor() }

    /**
     * Sets the arm to the amp shoot or source intake angle, which are the same.
     */
    fun armToAmpAngleCommand(): Command =
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, 105.0)) }
            .until { getArmPosition().degrees > 100.0 }
            .andThen(armStopCommand())

    /*----------------------------------------------------------------------------------------------------------------*/
    // HYBRID COMMANDS
    /*----------------------------------------------------------------------------------------------------------------*/

    /**
     * Stops both the arm and intake gears immediately.
     */
    fun stopAllCommand(): Command =
        SequentialCommandGroup(
            stopIntake(),
            armStopCommand()
        )

    /**
    - Runs the intake motor at the speed necessary to intake a game piece
    - Waits until a game piece enters the intake
    - Waits until the game piece has been compressed to a sufficient level
    - Stops the intake motor

    This is done by checking the velocity of the intake motor, given that the motor will run slower when the motion of the wheels is inhibited by game pieces
     */
    fun intakeAndStopCommand(): Command =
        SequentialCommandGroup(
            run { intakeMotor.set(INTAKE_SPEED) }.withTimeout(1.0),
            run {}.until { intakeMotor.encoder.velocity < 1.0 },
            run {}.until { intakeMotor.encoder.velocity > 1.0 },
            run {}.withTimeout(0.2),
            stopIntake()
        )

    /**
     * Brings the arm up to the amp angle and shoots the note into the amp.
     */
    fun ampCommand() =
        SequentialCommandGroup(
            armToAmpAngleCommand(),
            runIntakeAtSpeed(SHOOT_AMP_SPEED),
            WaitCommand(2.0),
            stopIntake(),
            armUpCommand()
        )

    /**
     * Brings the arm up to the source intake angle and then intakes a note.
     */
    fun armUpAndIntakeCommand() =
        SequentialCommandGroup(
            armToAmpAngleCommand(),
            runIntakeAtSpeed(INTAKE_SPEED),
            WaitCommand(0.1),
            stopIntake(),
            armUpCommand()
        )

    fun takeInCommand() =
        SequentialCommandGroup(
            runIntakeAtSpeed(INTAKE_SPEED),
            armDownCommand(),
            WaitCommand(0.8),
        )

    fun takeOutCommand() =
        SequentialCommandGroup(
            armUpCommand(),
            runIntakeAtSpeed(OUTTAKE_SPEED)
        )

    override fun periodic() {
        // Stop arm guard in case it screws itself over

        if (getArmPosition().degrees >= 180.0) armStopCommand().schedule()
        else if (getArmPosition().degrees <= -10.0) armStopCommand().schedule()

        intakeArmPositionEntry.setDouble(armMotor.encoder.position)
        intakeVelocityEntry.setDouble(intakeMotor.encoder.velocity)
    }

    override fun simulationPeriodic() {
    }
}
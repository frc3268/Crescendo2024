package frc.subsystems

import com.revrobotics.*
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.*
import frc.utils.Motor

class IntakeSubsystem: SubsystemBase() {
    val intakeMotor = Motor(9)
    val armMotor = Motor(10)
    val armEncoder: RelativeEncoder = armMotor.encoder
    val intakeEncoder: RelativeEncoder = intakeMotor.encoder
    val armPIDController = PIDController(0.2/170,0.0,0.0)

    val shuffleboardTab = Shuffleboard.getTab("intake")
    val intakeArmPositionEntry = shuffleboardTab.add("Intake arm encoder position", 0.0).entry
    val intakeVelocityEntry = shuffleboardTab.add("Intake encoder velocity", 0.0).entry

    // TODO replace with actual channel
    val limitSwitch = DigitalInput(0)

    companion object {
        const val INTAKE_SPEED = 0.3
        const val OUTTAKE_ADJUST_SPEED = -0.3
        const val OUTTAKE_SPEED = -0.9
        const val SHOOT_AMP_SPEED = -1.0
        //based momento...

        const val UP_ANGLE = 0.05
        const val DOWN_ANGLE = 170.0
    }

    init {
        shuffleboardTab.add("Arm down - TESTING", runOnce{armMotor.set(0.1)})
        shuffleboardTab.add("Arm up - TESTING", runOnce{armMotor.set(-0.1)})
        shuffleboardTab.add("Intake in - TESTING", runIntakeAtSpeed(INTAKE_SPEED))
        shuffleboardTab.add("Intake out - TESTING", runIntakeAtSpeed(OUTTAKE_SPEED))
        shuffleboardTab.add("Intake sequence - TESTING", intakeAndStopCommand())

        shuffleboardTab.add("Arm down - sequence", armDownCommand())
        shuffleboardTab.add("Arm up - sequence", armUpCommand())

        shuffleboardTab.add("STOP - TESTING", stopAllCommand())
        armEncoder.positionConversionFactor =  360 / 75.0
        intakeEncoder.velocityConversionFactor = 1.0 /1600.0
    }

    fun stopIntake(): Command =
        runOnce { intakeMotor.stopMotor() }

    /*
    intakeAndStopCommand: Command
    runs the intake motor at the speed necessary to intake a game piece
    waits until a game piece enters the intake
    waits until the game piece has been compressed to a sufficient level
    stops the intake motor

    this is done by checking the velocity of the intake motor, given that the motor will run slower when the motion of the wheels is inhibited by game pieces
     */
    fun intakeAndStopCommand() =
        SequentialCommandGroup(
            run { intakeMotor.set(INTAKE_SPEED) }.withTimeout(1.0),
            run {}.until { intakeEncoder.velocity < 1.0 },
            run {}.until { intakeEncoder.velocity > 1.0 },
            run {}.withTimeout(0.2),
            stopIntake()
        )

    fun stopArmCommand() =
        runOnce { armMotor.stopMotor() }

    /**
     * Stops both the arm and intake gears immediately.
     */
    fun stopAllCommand() =
        SequentialCommandGroup(
            stopIntake(),
            stopArmCommand()
        )

    /**
     * Runs the intake gears at [speed].
     */
    fun runIntakeAtSpeed(speed: Double) =
        runOnce { intakeMotor.set(speed) }

    fun armUpCommand() =
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, UP_ANGLE -5.0)) }
            .until { getArmPosition().degrees < UP_ANGLE }
            .andThen(stopArmCommand())

    fun armDownCommand() =
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, DOWN_ANGLE +5.0)) }
            .until { getArmPosition().degrees > DOWN_ANGLE }
            .andThen(stopArmCommand())

    /**
     * Sets the arm to the amp shoot or source intake angle, which are the same.
     */
    fun armToAmpAngleCommand() =
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, 105.0)) }
            .until { getArmPosition().degrees > 100.0 }
            .andThen(stopArmCommand())

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

    fun runIntakeCommand() =
        runIntakeAtSpeed(INTAKE_SPEED)

    fun runOnceOuttakeCommand() =
        runIntakeAtSpeed(OUTTAKE_ADJUST_SPEED)

    fun zeroArmEncoderCommand() =
        runOnce { armEncoder.position = 0.0 }

    fun getArmPosition() =
        Rotation2d.fromDegrees(armEncoder.position)

    override fun periodic() {

        // Stop arm guard in case it screws itself over

        if (getArmPosition().degrees >= 180.0) stopArmCommand().schedule()
        else if (getArmPosition().degrees <= -10.0) stopArmCommand().schedule()


        intakeArmPositionEntry.setDouble(armEncoder.position)
        intakeVelocityEntry.setDouble(intakeEncoder.velocity)
    }

    override fun simulationPeriodic() {
    }
}
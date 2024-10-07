package frc.robot.subsystems.intake

import com.revrobotics.*
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.*
import frc.lib.Motor
import frc.lib.rotation2dFromDeg
import frc.robot.Constants
import org.littletonrobotics.junction.Logger


class IntakeSubsystem: SubsystemBase() {
    private val intakeMotor = Motor(9)
    private val armMotor = Motor(10)
    val armEncoder: RelativeEncoder = armMotor.encoder
    val intakeEncoder: RelativeEncoder = intakeMotor.encoder
    val armPIDController = PIDController(0.8/170,0.0,0.0)

    val troubleshootingTab = Shuffleboard.getTab(Constants.TROUBLESHOOTING_TAB)
    val intakeArmPositionEntry = troubleshootingTab.add("Arm pos", 0.0)
            .withPosition(2, 0)
            .entry
    val intakePositionEntry = troubleshootingTab.add("Intake pos", 0.0)
            .withPosition(2, 1)
            .entry
    val intakeVelocityEntry = troubleshootingTab.add("Intake velocity", 0.0)
            .withPosition(2, 2)
            .entry

    // TODO replace with actual channel
    val limitSwitch = DigitalInput(0)

    companion object {
        const val INTAKE_SPEED = 0.30
        const val OUTTAKE_ADJUST_SPEED = -0.3
        const val OUTTAKE_SPEED = -0.9
        const val SHOOT_AMP_SPEED = -1.0
        //based momento...

        const val UP_ANGLE = 5.0
        const val DOWN_ANGLE = 180.0
    }

    init {
        armMotor.controller.inverted = true
        armEncoder.positionConversionFactor = 360 / 112.5
        intakeEncoder.velocityConversionFactor = 1.0 / 1600
        intakeEncoder.positionConversionFactor = 1.0/12.0
    }

    fun stopIntake(): Command =
        runOnce { intakeMotor.stop() }

    /*
    intakeAndStopCommand: Command
    runs the intake motor at the speed necessary to intake a game piece
    waits until a game piece enters the intake
    waits until the game piece has been compressed to a sufficient level
    stops the intake motor

    this is done by checking the velocity of the intake motor, given that the motor will run slower when the motion of the wheels is inhibited by game pieces
     */
    fun intakeAndStopCommand(): Command =
        run{intakeMotor.setVoltage(INTAKE_SPEED * 12.0)}.withTimeout(1.0).andThen(
            run{}.until{intakeEncoder.velocity < 1.0}.andThen(runOnce{intakeEncoder.setPosition(0.0)}.andThen(
               run{}.until{intakeEncoder.position > 1.15}.andThen(stopIntake()))
            )
        )

    fun stopArm(): Command =
        runOnce { armMotor.stop() }

    /**
     * Stops both the arm and intake gears immediately.
     */
    fun stopAllCommand(): Command =
        SequentialCommandGroup(
            stopIntake(),
            stopArm()
        )

    /**
     * Runs the intake gears at [speed].
     */
    fun runIntakeAtSpeed(speed: Double): Command =
        runOnce { intakeMotor.setVoltage(speed * 12.0) }

    fun armUpCommand(): Command =
        run { armMotor.setPercentOutput(armPIDController.calculate(getArmPosition().degrees, UP_ANGLE-5.0)) }
            .until { getArmPosition().degrees < UP_ANGLE }
            .andThen(stopArm())

    fun armDownCommand(): Command =
        run { armMotor.setPercentOutput(armPIDController.calculate(getArmPosition().degrees, DOWN_ANGLE+5.0)) }
            .until { getArmPosition().degrees > DOWN_ANGLE }
            .andThen(stopArm())
    fun takeInCommand(): Command =
        SequentialCommandGroup(
            runIntakeAtSpeed(INTAKE_SPEED),
            armDownCommand(),
            WaitCommand(0.8),
        )

    fun takeOutCommand(): Command =
        SequentialCommandGroup(
            armUpCommand(),
            runIntakeAtSpeed(OUTTAKE_SPEED)
        )

    fun zeroArmEncoderCommand(): Command =
        runOnce { armEncoder.position = 0.0 }

    fun getArmPosition() : Rotation2d =
        armEncoder.position.rotation2dFromDeg()

    override fun periodic() {
        if (getArmPosition().degrees >= 190.0 || getArmPosition().degrees <= -5.0)
            stopArm().schedule()

        intakeArmPositionEntry.setDouble(armEncoder.position)
        intakeVelocityEntry.setDouble(intakeEncoder.velocity)
        intakePositionEntry.setDouble(intakeEncoder.position)
    }

    override fun simulationPeriodic() {
    }
}
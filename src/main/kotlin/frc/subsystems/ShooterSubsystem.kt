package frc.subsystems

import edu.wpi.first.wpilibj2.command.*
import frc.utils.Motor

class ShooterSubsystem: SubsystemBase() {
    val leftFlywheelMotor = Motor(11)
    val rightFlywheelMotor = Motor(12)

    companion object {
        const val SPEAKER_SPEED = -1.0
        const val AMP_SPEED = -0.5
        const val INTAKE_SPEED = 0.7
    }

    fun runAtSpeedCommand(speed: Double): Command =
        runOnce {
            leftFlywheelMotor.set(speed)
            rightFlywheelMotor.set(speed)
        }

    fun speakerCommand(): Command =
        runAtSpeedCommand(SPEAKER_SPEED)

    fun ampCommand(): Command =
        // used to be within another function which ran another function; they were combined and this instance had to be changed
        runAtSpeedCommand(AMP_SPEED)

    fun takeInCommand(): Command =
        run { runAtSpeedCommand(INTAKE_SPEED) }
            // TODO Adjust timeout
            .withTimeout(1.5)
            .andThen(stopCommand())

    // if there are issues stopping the shooter this might be the problem however all instances should be taken care of
    fun stopCommand() : Command = runOnce {
        leftFlywheelMotor.stopMotor()
        rightFlywheelMotor.stopMotor()
    }

    /** This method will be called once per scheduler run  */
    override fun periodic() {
    }

    /** This method will be called once per scheduler run during simulation  */
    override fun simulationPeriodic() {
    }
}
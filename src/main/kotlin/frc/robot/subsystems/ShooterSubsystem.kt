package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.*
import frc.lib.Motor

class ShooterSubsystem: SubsystemBase() {
    val leftFlywheelMotor = Motor(11)
    val rightFlywheelMotor = Motor(12)

    companion object {
        const val SPEAKER_SPEED = -0.8
        const val AMP_SPEED = -0.5
        const val INTAKE_SPEED = 0.7
    }

    fun runAtSpeedCommand(speed: Double): Command =
        runOnce {
            leftFlywheelMotor.setVoltage(speed * 12.0)
            rightFlywheelMotor.setVoltage(speed * 12.0)
        }

    fun takeInCommand(): Command =
        run { runAtSpeedCommand(INTAKE_SPEED) }
            // TODO Adjust shooter timeout
            .withTimeout(1.5)
            .andThen(stopCommand())

    // if there are issues stopping the shooter this might be the problem however all instances should be taken care of
    fun stopCommand() : Command = runOnce {
        leftFlywheelMotor.stop()
        rightFlywheelMotor.stop()
    }

    /** This method will be called once per scheduler run  */
    override fun periodic() {
    }

    /** This method will be called once per scheduler run during simulation  */
    override fun simulationPeriodic() {
    }
}
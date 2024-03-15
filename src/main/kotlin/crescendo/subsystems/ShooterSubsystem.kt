package crescendo.subsystems

import edu.wpi.first.wpilibj2.command.*
import crescendo.utils.Motor

class ShooterSubsystem: SubsystemBase() {
    companion object {
        const val SPEAKER_SPEED = -1.0
        const val AMP_SPEED = -0.5
        const val INTAKE_SPEED = 0.7
    }

    private val leftFlywheelMotor = Motor(11)
    private val rightFlywheelMotor = Motor(12)

    private fun runAtSpeedCommand(speed: Double): Command =
        runOnce {
            leftFlywheelMotor.set(speed)
            rightFlywheelMotor.set(speed)
        }

    fun speakerCommand(): Command =
        runAtSpeedCommand(SPEAKER_SPEED)

    fun ampCommand(): Command =
        runAtSpeedCommand(AMP_SPEED)

    fun takeInCommand(): Command =
        run { runAtSpeedCommand(INTAKE_SPEED) }
            .withTimeout(1.5)
            .andThen(stopCommand())

    fun stopCommand(): Command =
        runOnce {
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
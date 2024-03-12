package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.wpilibj2.command.*

class ShooterSubsystem: SubsystemBase() {
    val leftFlywheelMotor = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)
    val rightFlywheelMotor = CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless)

    fun runAtSpeedCommand(speed: Double): Command = runOnce {
        leftFlywheelMotor.set(speed)
        rightFlywheelMotor.set(speed)
    }

    fun speakerCommand(): Command =
        runAtSpeedCommand(-1.0)

    fun ampCommand(): Command =
        // used to be within another function which ran another function; they were combined and this instance had to be changed
        runAtSpeedCommand(-0.5)

    fun takeInCommand(): Command =
        run { runAtSpeedCommand(0.7) }
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
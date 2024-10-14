package frc.robot.subsystems.shooter

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface ShooterSubsystemIO {
    @AutoLog
    open class ShooterIOInputs {
        //IO goes here
    }

    //any mandatory props

    //any mandatory methods

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ShooterIOInputs) {}

    fun reset() {}
}

// FIXME: This should be automated
class IntakeIOInputsAutoLogged : ShooterSubsystemIO.ShooterIOInputs(), LoggableInputs {
    override fun toLog(table: LogTable) {
    }

    override fun fromLog(table: LogTable) {
    }
}

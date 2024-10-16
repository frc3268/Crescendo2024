package frc.robot.subsystems.climber

import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface LeftClimberSubsystemIO {
    @AutoLog
    open class LeftClimberIOInputs {
        //IO goes here
    }

    //any mandatory props

    //any mandatory methods

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: LeftClimberIOInputs) {}

    fun reset() {}
}

// FIXME: This should be automated
class LeftClimberIOInputsAutoLogged : LeftClimberSubsystemIO.LeftClimberIOInputs(), LoggableInputs {
    override fun toLog(table: LogTable?) {
    }

    override fun fromLog(table: LogTable?) {
    }
}
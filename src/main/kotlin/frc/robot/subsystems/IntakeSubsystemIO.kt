package frc.lib

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface IntakeSubsystemIO {
    @AutoLog
    open class IntakeIOInputs {
        var armAppliedVolts: Double = 0.0
        var armCurrentAmps: Double = 0.0
        var armPosition: Rotation2d = Rotation2d()
        var armVelocityRadPerSec: Double = 0.0

        var intakeAppliedVolts: Double = 0.0
        var intakeCurrentAmps: Double = 0.0
        var intakePosition: Rotation2d = Rotation2d()
        var intakeVelocityRadPerSec: Double = 0.0
    }

    val turnPIDController:PIDController

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: IntakeIOInputs) {}


    fun reset() {}
}

class IntakeIOInputsAutoLogged : IntakeSubsystemIO.IntakeIOInputs(), LoggableInputs {
    override fun toLog(table: LogTable) {

    }

    override fun fromLog(table: LogTable) {

    }
}

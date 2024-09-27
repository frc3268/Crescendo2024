package frc.robot.subsystems.intake

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface IntakeSubsystemIO {
    @AutoLog
    open class IntakeIOInputs {
        var armAppliedVolts: Double = 0.0
        var armCurrentAmps: DoubleArray = doubleArrayOf()
        var armPosition: Rotation2d = Rotation2d()
        var armVelocityRadPerSec: Double = 0.0

        var intakeAppliedVolts: Double = 0.0
        var intakeCurrentAmps: DoubleArray = doubleArrayOf()
        var intakePosition: Rotation2d = Rotation2d()
        var intakeVelocityRadPerSec: Double = 0.0
    }

    val turnPIDController: PIDController

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: IntakeIOInputs) {}
    
    /** Run the arm motor at the specified voltage.  */
    fun setArmVoltage(volts: Double) {}
    
    /** Run the intake motor at the specified voltage.  */
    fun setIntakeVoltage(volts: Double) {}
    
    /** Enable or disable brake mode on the arm motor.  */
    fun setArmBrakeMode(enable: Boolean) {}
    
    /** Enable or disable brake mode on the intake motor.  */
    fun setIntakeBrakeMode(enable: Boolean) {}
    
    fun reset() {}
}

// FIXME: This should be automated
class IntakeIOInputsAutoLogged : IntakeSubsystemIO.IntakeIOInputs(), LoggableInputs {
    override fun toLog(table: LogTable) {
        table.put("armAppliedVolts", armAppliedVolts)
        table.put("armCurrentAmps", armCurrentAmps)
        table.put("armPosition", armPosition)
        table.put("armVelocityRadPerSec", armVelocityRadPerSec)
        table.put("intakeAppliedVolts", intakeAppliedVolts)
        table.put("intakeCurrentAmps", intakeCurrentAmps)
        table.put("intakePosition", intakePosition)
        table.put("intakeVelocityRadPerSec", intakeVelocityRadPerSec)
    }

    override fun fromLog(table: LogTable) {
        table.get("armAppliedVolts", armAppliedVolts)
        table.get("armCurrentAmps", armCurrentAmps)
        table.get("armPosition", armPosition)
        table.get("armVelocityRadPerSec", armVelocityRadPerSec)
        table.get("intakeAppliedVolts", intakeAppliedVolts)
        table.get("intakeCurrentAmps", intakeCurrentAmps)
        table.get("intakePosition", intakePosition)
        table.get("intakeVelocityRadPerSec", intakeVelocityRadPerSec)
    }
}

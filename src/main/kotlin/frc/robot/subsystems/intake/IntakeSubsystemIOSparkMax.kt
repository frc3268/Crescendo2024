package frc.robot.subsystems.intake

import edu.wpi.first.math.controller.PIDController
import frc.robot.subsystems.intake.IntakeSubsystemIO.IntakeIOInputs

class IntakeSubsystemIOSparkMax(val moduleConstants: IntakeSubsystemConstants.ModuleConstants) : IntakeSubsystemIO {
    override val turnPIDController: PIDController = moduleConstants.PID_CONTROLLER
   
    override fun updateInputs(inputs: IntakeIOInputs) {}

    override fun reset() {}
}




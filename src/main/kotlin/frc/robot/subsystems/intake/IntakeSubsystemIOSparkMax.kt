package frc.robot.subsystems.intake

import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.AnalogEncoder
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.lib.*
import kotlin.math.IEEErem
import frc.robot.subsystems.intake.IntakeSubsystemIO.IntakeIOInputs


class IntakeSubsystemIOSparkMax(moduleConstants: IntakeSubsystemConstants.ModuleConstants) : IntakeSubsystemIO {
    override val turnPIDController: PIDController = moduleConstants.PID_CONTROLLER
    override fun updateInputs(inputs: IntakeIOInputs) {

    }

    override fun reset() {

    }
}

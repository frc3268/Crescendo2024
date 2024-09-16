package frc.robot.subsystems.intake

import frc.lib.SwerveModuleIO
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IntakeIO {
    @AutoLog
    //record inputs and outputs (ie power to the motors & encoder values)
    open class IntakeIOInputs{

    }

    //functions to do common things. see swervemoduleio class


}

class ModuleIOInputsAutoLogged : SwerveModuleIO.ModuleIOInputs(), LoggableInputs {
    //see swervemoduleio class
    override fun toLog(table: LogTable) {

    }

    override fun fromLog(table: LogTable) {

    }
}
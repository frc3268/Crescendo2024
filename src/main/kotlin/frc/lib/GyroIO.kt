package frc.lib

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface GyroIO {
    @AutoLog
    open class GyroIOInputs {
        var connected: Boolean = false
        var yawPosition: Rotation2d = Rotation2d()
        var odometryYawTimestamps: DoubleArray = doubleArrayOf()
        var odometryYawPositions: Array<Rotation2d> = arrayOf()
        var yawVelocityRadPerSec: Double = 0.0
    }

    fun updateInputs(inputs: GyroIOInputs) {}

    fun zeroYaw() {}
}

class GyroIOInputsAutoLogged : GyroIO.GyroIOInputs(), LoggableInputs {
    override fun toLog(table: LogTable) {
        table.put("connected", connected)
        table.put("yawPosition", yawPosition)
        table.put("odometryYawPositions", odometryYawPositions.map { it.radians }.toDoubleArray())
        table.put("odometryYawTimestamps", odometryYawTimestamps)
        table.put("yawVelocityRadPerSec", yawVelocityRadPerSec)
    }

    override fun fromLog(table: LogTable) {
        table.get("connected", connected)
        table.get("yawPosition", yawPosition)
        table.get("odometryYawPositions", odometryYawPositions.map { it.radians }.toDoubleArray())
        table.get("odometryYawTimestamps", odometryYawTimestamps)
        table.get("yawVelocityRadPerSec", yawVelocityRadPerSec)
    }
}

package frc.lib

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.SPI
import java.util.*
import kotlin.math.IEEErem


class GyroIOKauai : GyroIO {
    private val gyro = AHRS(SPI.Port.kMXP)
    private val yawPositionQueue: Queue<Double>? = null
    private val yawTimestampQueue: Queue<Double>? = null


    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.connected = gyro.isConnected
        inputs.yawPosition = -(gyro.rotation2d.degrees).IEEErem(360.0).rotation2dFromDeg()
        inputs.yawVelocityRadPerSec = gyro.rate.rotation2dFromDeg().radians
        //useless for now
        inputs.odometryYawTimestamps =
            yawTimestampQueue!!.stream().mapToDouble { value: Double? -> value!! }.toArray()
        inputs.odometryYawPositions =
            yawPositionQueue!!.stream()
                .map<Rotation2d> { value: Double? ->
                    Rotation2d.fromDegrees(
                        value!!
                    )
                }
                .toArray<Rotation2d> { arrayOf() }
        yawTimestampQueue.clear()
        yawPositionQueue.clear()


    }
}
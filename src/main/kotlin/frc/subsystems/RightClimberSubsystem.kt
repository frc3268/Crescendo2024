package frc.subsystems

import com.revrobotics.*
import edu.wpi.first.wpilibj2.command.*
import frc.utils.Motor

class RightClimberSubsystem: SubsystemBase(){
    val motor = Motor(15)
    val encoder: RelativeEncoder = motor.encoder

    init {
        motor.inverted = true
        encoder.positionConversionFactor = 1.0/224.0
        encoder.position = 0.0
    }

    /**
     * TODO make a func that converts meters to rotations and a function that converts rotations to meters
     * TODO using these functions rotate the motors on the arms accordingly while accounting for min and max heights
     */

    fun down() =
        run { motor.set(-0.5) }
            .until { encoder.position < 0.1 }
            .andThen(runOnce { motor.stopMotor() })

    fun up() =
        run { motor.set(0.5) }
            .until { encoder.position > 0.9 }
            .andThen(runOnce { motor.stopMotor() })

    fun reset() =
        runOnce { encoder.position = 0.0 }

    fun testup() =
        run { motor.set(0.2) }

    fun testdown() =
        run { motor.set(-0.2) }

    fun stop() =
        runOnce { motor.set(0.0) }

    override fun periodic() {
        if(encoder.position !in -0.1..1.1){
            stop().schedule()
        }
    }
}

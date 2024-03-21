package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.wpilibj2.command.*
import frc.lib.utils.*

class RightClimberSubsystem: SubsystemBase(){
    val motor = Motor(15)
    val encoder: RelativeEncoder = motor.encoder

    /* CONSTANTS */
    private val metersPerRotation: Double = 0.0
    private val minPositionMeters: Double = 0.0
    private val maxPositionMeters: Double = 0.34

    init {
        motor.inverted = true
        // TODO test if this is needed
        //rightEncoder.inverted = true
        encoder.positionConversionFactor = 1.0/224.0
        encoder.position = 0.0
    }


    fun down(): Command =
        run { motor.set(-0.5) }
            .until { encoder.position < 0.1 }
            .andThen(runOnce { motor.stopMotor() })

    fun up(): Command =
        run { motor.set(0.5) }
            .until { encoder.position > 0.9 }
            .andThen(runOnce { motor.stopMotor() })

    fun reset(): Command =
        runOnce { encoder.position = 0.0 }

    fun testup():Command =
        run { motor.set(0.2) }

    fun testdown():Command =
        run { motor.set(-0.2) }

    fun stop(): Command =
        runOnce { motor.set(0.0) }

    override fun periodic() {
        //System.out.println("Right climber: " + encoder.position)

        if(encoder.position !in -0.1..1.1){
            stop().schedule()
        }
    }
}

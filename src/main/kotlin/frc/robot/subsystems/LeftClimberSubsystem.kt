package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.wpilibj2.command.*
import frc.lib.utils.*

class LeftClimberSubsystem: SubsystemBase(){
    val motor = Motor(14)
    val encoder: RelativeEncoder = motor.encoder
    val limitSwitch = motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)

    init {
        motor.inverted = true
        // TODO test if this is needed
        //rightEncoder.inverted = true
        
        encoder.positionConversionFactor = 1.0 / 104
        encoder.position = 0.0
    }

    /**
     * TODO make a func that converts meters to rotations and a function that converts rotations to meters
     * TODO using these functions rotate the motors on the arms accordingly while accounting for min and max heights
     */

    fun down(): Command =
        run { motor.set(-0.3) }
            .until { encoder.position < 0.1 || limitSwitch.isPressed}
            .andThen(runOnce { motor.stopMotor() })

    fun up(): Command =
        run { motor.set(0.3) }
            .until { encoder.position > 0.9 }
            .andThen(runOnce { motor.stopMotor() })

    fun reset(): Command =
        runOnce { encoder.position = 0.0 }

    fun testup():Command =
            runOnce { motor.set(0.2) }

    fun testdown():Command =
            runOnce { motor.set(-0.2) }

    fun stop(): Command =
        runOnce { motor.set(0.0) }

    override fun periodic() {
        System.out.println("Left climber: " + encoder.position)
        System.out.println("Left limit switch: " + limitSwitch.isPressed)
        if(encoder.position !in -0.1..1.1){
            stop().schedule()
        }
    }
}

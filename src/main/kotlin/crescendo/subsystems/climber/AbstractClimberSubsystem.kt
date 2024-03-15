package crescendo.subsystems.climber

import edu.wpi.first.wpilibj2.command.*
import crescendo.utils.Motor

/**
 * This represents either one of the two climber arms. [LeftClimberSubsystem] and [RightClimberSubsystem] are derived
 * from here.
 *
 * TODO make a func that converts meters to rotations and a function that converts rotations to meters
 *
 * TODO using these functions rotate the motors on the arms accordingly while accounting for min and max heights
 */
abstract class AbstractClimberSubsystem(
    motorId: Int,
    positionConversionFactor: Double,
    val downSpeed: Double,
    val upSpeed: Double
): SubsystemBase() {
    companion object {
        const val TEST_UP_SPEED = 0.2
        const val TEST_DOWN_SPEED = -0.2
    }

    private val motor = Motor(motorId)

    init {
        motor.inverted = true
        motor.encoder.positionConversionFactor = positionConversionFactor
        motor.encoder.position = 0.0
    }

    fun down(): Command =
        run { motor.set(downSpeed) }
            .until { motor.encoder.position < 0.1 }
            .andThen(runOnce { motor.stopMotor() })

    fun up(): Command =
        run { motor.set(upSpeed) }
            .until { motor.encoder.position > 0.9 }
            .andThen(runOnce { motor.stopMotor() })

    fun reset(): Command =
        runOnce { motor.encoder.position = 0.0 }

    fun testUp(): Command =
        runOnce { motor.set(TEST_UP_SPEED) }

    fun testDown(): Command =
        runOnce { motor.set(TEST_DOWN_SPEED) }

    fun stop(): Command =
        runOnce { motor.set(0.0) }

    override fun periodic() {
        if(motor.encoder.position !in -0.1..1.1) {
            stop().schedule()
        }
    }
}

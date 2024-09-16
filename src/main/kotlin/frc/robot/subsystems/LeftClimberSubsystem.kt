package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.*
import frc.lib.Motor

class LeftClimberSubsystem: SubsystemBase(){
    val motor = Motor(14)
    val encoder: RelativeEncoder = motor.encoder
    val troubleShootingTab: ShuffleboardTab = Shuffleboard.getTab("Troubleshooting")
    val booleanBoxDangerMode: GenericEntry = troubleShootingTab.add("L DGo", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
        .withPosition(5, 0)
            .entry

    init {
        motor.controller.inverted = true
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
        run { motor.setPercentOutput(-1.0) }
            .until { encoder.position < 0.1 }
            .andThen(runOnce { motor.stop() })

    fun up(): Command =
        run { motor.setPercentOutput(1.0) }
            .until { encoder.position > 0.9 }
            .andThen(runOnce { motor.stop() })

    fun reset(): Command =
        runOnce { encoder.position = 0.0 }

    fun testup():Command =
        runOnce { motor.setPercentOutput(0.2) }

    fun testdown():Command =
        runOnce { motor.setPercentOutput(-0.2) }

    fun stop(): Command =
        runOnce { motor.setPercentOutput(0.0) }

    override fun periodic() {
        System.out.println("Left climber: " + encoder.position)
        if(encoder.position !in -0.1..2.0 && !booleanBoxDangerMode.getBoolean(false)){
            stop().schedule()
        }
    }
}

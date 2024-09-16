package frc.lib

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
class Motor(id: Int, val positionPidController:ProfiledPIDController = ProfiledPIDController(0.0,0.0,0.0, TrapezoidProfile.Constraints(0.0,0.0)), val velocityPIDController: PIDController = PIDController(0.0,0.0,0.0), rotationsPerUnit: Double = 1.0) {
    val controller = CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless)
    val encoder = controller.encoder

    init{
        encoder.positionConversionFactor = 1/rotationsPerUnit
    }

    fun setPercentOutput(percentOutput: Double){
        controller.set(percentOutput / 100)
    }
    fun setVoltage(voltage: Double){
        controller.setVoltage(voltage)
    }
    fun setPosition(position: Double){
        positionPidController.goal = TrapezoidProfile.State(position, 0.0)
        controller.set(positionPidController.calculate(getPositonMeasurement()))
    }
    fun setVelocity(velocity: Double){
        velocityPIDController.setpoint = velocity
        controller.set(velocityPIDController.calculate(getVelocityMeasurement()))
    }
    fun stop(){
        controller.stopMotor()
    }

    fun isAtVelocityGoal() = velocityPIDController.atSetpoint()
    fun isAtPositionGoal() = positionPidController.atGoal()

    open fun getPositonMeasurement() = encoder.position
    open fun getVelocityMeasurement() = encoder.velocity
}

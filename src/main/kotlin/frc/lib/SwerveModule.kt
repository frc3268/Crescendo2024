package frc.lib

import com.revrobotics.*
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.*
import frc.robot.Constants
import org.littletonrobotics.junction.Logger
import kotlin.math.*

/*
Props: drive motor, drive encoder, angle motor, angle encoder, absolute encoder
Get: Cancoder measurement, Module state(velocity) and position
Set: Module state
 */
class SwerveModule(val io: , val index:Int) {
    private val inputs = ModuleIOInputsAutoLogged()
    private val ShuffleboardTab = Shuffleboard.getTab("Swerve Module " + index)
    val setPointEntry: GenericEntry = ShuffleboardTab.add("Setpoint", 0.0).withWidget(BuiltInWidgets.kGyro).entry

    val angleEncoderEntry: GenericEntry = ShuffleboardTab.add("Angle Encoder (Relative)", 0.0).withWidget(BuiltInWidgets.kGyro).entry
    val absoluteEncoderEntry: GenericEntry = ShuffleboardTab.add("Angle Encoder (Absolute)", 0.0).withWidget(BuiltInWidgets.kGyro).entry

    val turnController: PIDController = io.turnPIDController

    init {
        turnController.enableContinuousInput(-180.0, 180.0)
    }

    fun update() {
        Logger.processInputs("Drive/module" + index.toString(), inputs)
        angleEncoderEntry.setDouble(getState().angle.degrees)
        absoluteEncoderEntry.setDouble(inputs.turnAbsolutePosition.degrees)
    }

    fun resetToAbsolute() {
        io.reset()
    }

    fun getState() = SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.turnPosition)
    fun getPosition() = SwerveModulePosition(inputs.drivePositionMeters, inputs.turnPosition)

    fun setDesiredState(desiredState: SwerveModuleState) {
        if (abs(desiredState.speedMetersPerSecond) < 0.01) {
            stop()
            return
        }
        val optimizedState = SwerveModuleState.optimize(desiredState, getState().angle)
        setPointEntry.setDouble(optimizedState.angle.degrees)
        io.setDriveVoltage((optimizedState.speedMetersPerSecond / SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND) * 12.0)
        io.setTurnVoltage(turnController.calculate(getState().angle.degrees, optimizedState.angle.degrees) * 12.0)
    }

    fun stop() {
        io.setDriveVoltage(0.0)
        io.setTurnVoltage(0.0)
    }


}
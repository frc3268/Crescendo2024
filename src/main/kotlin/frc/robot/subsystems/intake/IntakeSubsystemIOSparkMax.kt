package frc.robot.subsystems.intake

import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.AnalogEncoder
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.lib.SwerveModuleIO.ModuleIOInputs
import frc.robot.subsystems.intake.IntakeSubsystemConstants.IntakeConstants
import kotlin.math.IEEErem


class IntakeSubsystemIOSparkMax() : IntakeSubsystemIO {

    private val driveMotor = CANSparkMax(moduleConstants.DRIVE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val angleMotor = CANSparkMax(moduleConstants.ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    override val turnPIDController: PIDController = moduleConstants.PID_CONTROLLER

    private val driveEncoder: RelativeEncoder = driveMotor.encoder
    private val angleEncoder: RelativeEncoder = angleMotor.encoder

    private val absoluteEncoder = AnalogEncoder(moduleConstants.ENCODER_ID)

    // Gear ratios for SDS MK4i L2, adjust as necessary
    private val DRIVE_GEAR_RATIO: Double = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)
    private val TURN_GEAR_RATIO: Double = 150.0 / 7.0

    fun ModuleIOSparkMax(index: Int) {
        absoluteEncoder.distancePerRotation =
                SwerveDriveConstants.Encoder.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION
        absoluteEncoder.positionOffset = moduleConstants.ANGLE_OFFSET.degrees
        driveEncoder.positionConversionFactor =
                SwerveDriveConstants.DriveMotor.POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION
        driveEncoder.velocityConversionFactor =
                SwerveDriveConstants.DriveMotor.VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND
        angleEncoder.positionConversionFactor =
                SwerveDriveConstants.AngleMotor.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION

        driveMotor.inverted = moduleConstants.DRIVE_MOTOR_REVERSED
        angleMotor.inverted = moduleConstants.ANGLE_MOTOR_REVERSED

        driveMotor.setOpenLoopRampRate(SwerveDriveConstants.DrivetrainConsts.OPEN_LOOP_RAMP_RATE_SECONDS)
        angleMotor.setOpenLoopRampRate(SwerveDriveConstants.DrivetrainConsts.OPEN_LOOP_RAMP_RATE_SECONDS)

        driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 15)
        //todo: fix? below
        angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 15)
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        inputs.drivePositionMeters = -driveEncoder.position
        inputs.driveVelocityMetersPerSec = -driveEncoder.velocity
        inputs.driveAppliedVolts = driveMotor.appliedOutput * driveMotor.busVoltage
        inputs.driveCurrentAmps = doubleArrayOf(driveMotor.outputCurrent)

        inputs.turnAbsolutePosition = ((absoluteEncoder.absolutePosition * 360.0) + moduleConstants.ANGLE_OFFSET.degrees).rotation2dFromDeg()
        inputs.turnPosition = ((-inputs.turnAbsolutePosition.degrees).IEEErem(360.0).rotation2dFromDeg())
        inputs.turnVelocityRadPerSec = (
                Units.rotationsPerMinuteToRadiansPerSecond(angleEncoder.velocity)
                        / TURN_GEAR_RATIO)
        inputs.turnAppliedVolts = angleMotor.appliedOutput * angleMotor.busVoltage
        inputs.turnCurrentAmps = doubleArrayOf(angleMotor.outputCurrent)
    }

    // TODO: REDO THIS
    public fun setDriveVoltage(volts: Double) {
        driveMotor.setVoltage(volts)
    }

    public fun setTurnVoltage(volts: Double) {
        angleMotor.setVoltage(volts)
    }

    public fun setDriveBrakeMode(enable: Boolean) {
        driveMotor.setIdleMode(if (enable) IdleMode.kBrake else IdleMode.kCoast)
    }

    public fun setTurnBrakeMode(enable: Boolean) {
        angleMotor.setIdleMode(if (enable) IdleMode.kBrake else IdleMode.kCoast)
    }

    public fun reset() {
        driveEncoder.position = 0.0
        angleEncoder.position = ((absoluteEncoder.absolutePosition * 360.0) + moduleConstants.ANGLE_OFFSET.degrees)
    }
}
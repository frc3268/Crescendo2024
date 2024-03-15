package crescendo.subsystems.swerve

import com.revrobotics.*
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.AnalogEncoder
import edu.wpi.first.wpilibj.shuffleboard.*
import crescendo.utils.*
import kotlin.math.*

/**
 * Represents one of the four swerve modules.
 *
 * Swerve modules have two motors: a [driveMotor], which controls speed, and an [angleMotor], which controls direction.
 */
class SwerveModule(
    val MODULE_NUMBER: Int,
    val ANGLE_OFFSET: Rotation2d,
    val DRIVE_MOTOR_ID: Int,
    val ANGLE_MOTOR_ID: Int,
    val ENCODER_ID: Int,
    val DRIVE_MOTOR_REVERSED: Boolean,
    val ANGLE_MOTOR_REVERSED: Boolean,
    val PID_CONTROLLER: PIDController
) {
    companion object {
        const val WHEEL_DIAMETER_METERS = 0.1016
        const val OPEN_LOOP_RAMP_RATE_SECONDS = 0.25

        const val ANGLE_MOTOR_POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION = 360.0
        const val ENCODER_POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION = 360.0
        const val GEAR_RATIO = 8.14 / 1.0
        const val DRIVE_MOTOR_POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION =
            (WHEEL_DIAMETER_METERS * Math.PI) / GEAR_RATIO
        const val DRIVE_MOTOR_VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND =
            DRIVE_MOTOR_POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION / 60.0
    }

    private val driveMotor = CANSparkMax(DRIVE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val angleMotor = CANSparkMax(ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val absoluteEncoder = AnalogEncoder(ENCODER_ID)

    private val shuffleboardTab = Shuffleboard.getTab("Swerve Module $MODULE_NUMBER")
    val setPointEntry: GenericEntry = shuffleboardTab
        .add("Setpoint", 0.0)
        .withWidget(BuiltInWidgets.kGyro)
        .entry
    val angleEncoderEntry: GenericEntry = shuffleboardTab
        .add("Angle Encoder (Relative)", 0.0)
        .withWidget(BuiltInWidgets.kGyro)
        .entry
    val absoluteEncoderEntry: GenericEntry = shuffleboardTab
        .add("Angle Encoder (Absolute)", 0.0)
        .withWidget(BuiltInWidgets.kGyro)
        .entry

    private var turnController: PIDController = PID_CONTROLLER

    init {
        absoluteEncoder.distancePerRotation =
            ENCODER_POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION
        absoluteEncoder.positionOffset = ANGLE_OFFSET.degrees
        driveMotor.encoder.positionConversionFactor =
            DRIVE_MOTOR_POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION
        driveMotor.encoder.velocityConversionFactor =
            DRIVE_MOTOR_VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND
        angleMotor.encoder.positionConversionFactor =
            ANGLE_MOTOR_POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION

        driveMotor.inverted = DRIVE_MOTOR_REVERSED
        angleMotor.inverted = ANGLE_MOTOR_REVERSED

        driveMotor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE_SECONDS)
        angleMotor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE_SECONDS)

        turnController.enableContinuousInput(-180.0, 180.0)

        driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 15)
        //todo: fix? below
        angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 15)
    }

    fun updateShuffleboard(){
        angleEncoderEntry.setDouble(getState().angle.degrees)
        absoluteEncoderEntry.setDouble(getAbsoluteEncoderMeasurement().degrees)
    }

    fun resetToAbsolute() {
        driveMotor.encoder.position = 0.0
        angleMotor.encoder.position = getAbsoluteEncoderMeasurement().degrees
    }

    private fun getAbsoluteEncoderMeasurement() =
        Rotation2d.fromDegrees((absoluteEncoder.absolutePosition * 360.0) + ANGLE_OFFSET.degrees)

    fun getState() =
        SwerveModuleState(
            -driveMotor.encoder.velocity,
            Rotation2d.fromDegrees(-getAbsoluteEncoderMeasurement()
                .degrees
                .IEEErem(360.0))
        )

    fun getPosition() =
        SwerveModulePosition(
            -driveMotor.encoder.position,
            Rotation2d.fromDegrees(-getAbsoluteEncoderMeasurement()
                .degrees
                .IEEErem(360.0))
        )

    fun setDesiredState(desiredState: SwerveModuleState) {
        if (abs(desiredState.speedMetersPerSecond) < 0.01){
            stop()
            return
        }
        val optimizedState = SwerveModuleState.optimize(desiredState, getState().angle)
        setPointEntry.setDouble(optimizedState.angle.degrees)
        driveMotor.set(optimizedState.speedMetersPerSecond / Constants.MAX_SPEED_METERS_PER_SECOND)
        angleMotor.set(turnController.calculate(getState().angle.degrees, optimizedState.angle.degrees))
    }

    /**
     * Immediately stops this swerve module.
     */
    fun stop() {
        driveMotor.set(0.0)
        angleMotor.set(0.0)
    }
}
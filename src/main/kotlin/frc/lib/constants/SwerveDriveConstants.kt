/*
HEY!
If you're planning on using the Swerve Drive Base in this library on your own robot,
make sure to edit these constants based on your own needs! Info on this may appear here later,
but as of now [[https://github.com/Team364/BaseFalconSwerve]] is a great resource
for most constants used in this library.
 */
package frc.lib.constants

import com.revrobotics.CANSparkBase
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import frc.lib.constants.SwerveDriveConstants.DrivetrainConsts.WHEEL_DIAMETER_METERS

object SwerveDriveConstants {
    data class ModuleConstants(
        val MODULE_NUMBER: Int,
        val ANGLE_OFFSET: Rotation2d,
        val DRIVE_MOTOR_ID: Int,
        val ANGLE_MOTOR_ID: Int,
        val ENCODER_ID: Int,
        val DRIVE_MOTOR_REVERSED: Boolean,
        val ANGLE_MOTOR_REVERSED: Boolean,
        val PID_CONTROLLER:PIDController
    )

    object ModuleGains {
        /* Drive Motor Characterization Values */
        const val DRIVE_KS: Double = 0.0
        const val DRIVE_KV: Double = 0.0
        const val DRIVE_KA: Double = 0.0
    }

    object DriveMotor {
        const val GEAR_RATIO: Double = 8.14 / 1.0
        const val CONTINUOUS_CURRENT_LIMIT: Int = 80
        const val POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION: Double =
            (WHEEL_DIAMETER_METERS * Math.PI) / GEAR_RATIO
        const val VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND = POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION / 60.0
        val NEUTRAL_MODE: CANSparkBase.IdleMode = CANSparkBase.IdleMode.kBrake
        const val INVERT: Boolean = false
    }

    object AngleMotor {
        //for some reason 10:1 delivers the most accurate results
        private const val GEAR_RATIO: Double = -((150.0 / 7.0) / 1.0)
        const val POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION = 16.8

    }

    object Encoder {
        const val INVERT:Boolean = false
        const val POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION:Double = 360.0
    }

    object DrivetrainConsts {        /* Drivetrain Constants */
        val TRACK_WIDTH_METERS = Units.inchesToMeters(24.0)
        val WHEEL_BASE_METERS = Units.inchesToMeters(24.0)

        const val WHEEL_DIAMETER_METERS = 0.1016
        val WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI

        const val OPEN_LOOP_RAMP_RATE_SECONDS: Double = 0.25
        const val CLOSED_LOOP_RAMP_RATE_SECONDS: Double = 0.0

        /* Swerve Voltage Compensation */
        const val VOLTAGE_COMPENSATION: Double = 12.0

        /* Swerve Profiling Values */
        const val MAX_SPEED_METERS_PER_SECOND = 5.0
        const val MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND = 200.0
        const val MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0

        var turnController = PIDController(
            //tentative!
            0.01,
            0.0,
            0.0
        )
        val xPIDController = PIDController(1.5,1.0,0.8)
        val yPIDController = PIDController(1.5,1.0,0.8)
        val thetaPIDController = PIDController(2.0,0.3,0.0)

        //in the order they appear in modules list
        //assuming that 0,0 is the center of the robot, and (+,+) means (left, front)
        val kinematics =
            SwerveDriveKinematics(
                    Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0),
                Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),
                    Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),
                    Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0),


        )
    }
    val modules = listOf<ModuleConstants>(
        ModuleConstants(1, Rotation2d.fromDegrees(-253.36), 1, 2, 0, false, false, PIDController(0.012, 0.003, 0.0003)),
        ModuleConstants(2, Rotation2d.fromDegrees(-7.66), 3, 4, 1, false, false, PIDController(0.0012, 0.003, 0.0003)),
        ModuleConstants(3, Rotation2d.fromDegrees(-182.53), 5, 6, 2, false, false, PIDController(0.012, 0.003, 0.0003)),
        ModuleConstants(4, Rotation2d.fromDegrees(-115.76), 7, 8, 3, false, false, PIDController(0.012, 0.003, 0.0003))
    )
    val startCoordinates = mapOf(
        // Starting x values
        DriverStation.Alliance.Blue to -0.5,
        DriverStation.Alliance.Red to -13.2254
    )
        .mapValues { colorEntry ->
            // Starting y values
            listOf(2.57305, 4.6305, 7.181312)
                .map { Pose2d(colorEntry.value, it, Rotation2d.fromDegrees(0.0))}
        }
}
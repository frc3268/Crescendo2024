/*
HEY!
If you're planning on using the Swerve Drive Base in this library on your own robot,
make sure to edit these constants based on your own needs! Info on this may appear here later,
but as of now [[https://github.com/Team364/BaseFalconSwerve]] is a great resource
for most constants used in this library.
 */
package frc.lib

import com.revrobotics.CANSparkBase
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import frc.lib.SwerveDriveConstants.DrivetrainConsts.WHEEL_DIAMETER_METERS

object SwerveDriveConstants {
    data class ModuleConstants(
            val MODULE_NUMBER: Int,
            val ANGLE_OFFSET: Rotation2d,
            val DRIVE_MOTOR_ID: Int,
            val ANGLE_MOTOR_ID: Int,
            val ENCODER_ID: Int,
            val DRIVE_MOTOR_REVERSED: Boolean,
            val ANGLE_MOTOR_REVERSED: Boolean,
            val PID_CONTROLLER: PIDController
    )

    object DriveMotor {
        const val GEAR_RATIO: Double = 8.14 / 1.0
        const val POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION: Double =
                (WHEEL_DIAMETER_METERS * Math.PI) / GEAR_RATIO
        const val VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND = POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION / 60.0
    }

    object AngleMotor {
        //for some reason 10:1 delivers the most accurate results
        const val POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION = 16.8

    }

    object Encoder {
        const val INVERT: Boolean = false
        const val POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION: Double = 360.0
    }

    object DrivetrainConsts {        /* Drivetrain Constants */
        val TRACK_WIDTH_METERS = Units.inchesToMeters(24.0)
        val WHEEL_BASE_METERS = Units.inchesToMeters(24.0)

        const val WHEEL_DIAMETER_METERS = 0.1016

        const val OPEN_LOOP_RAMP_RATE_SECONDS: Double = 0.25

        /* Swerve Profiling Values */
        const val MAX_SPEED_METERS_PER_SECOND = 6.0
        const val MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND = 300.0
        const val MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0

        val xPIDController = ProfiledPIDController(1.5, 0.0, 0.0, TrapezoidProfile.Constraints(
            MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED))
        val yPIDController = ProfiledPIDController(1.5, 0.0, 0.0, TrapezoidProfile.Constraints(
            MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED))
        val thetaPIDController = PIDController(1.5, 0.0, 0.0)

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


    val modules = listOf(
            ModuleConstants(1, Rotation2d.fromDegrees(-253.36), 1, 2, 0, false, false, PIDController(0.009, 0.003, 0.0003)),
            ModuleConstants(2, Rotation2d.fromDegrees(-7.66), 3, 4, 1, false, false, PIDController(0.009, 0.003, 0.0003)),
            ModuleConstants(3, Rotation2d.fromDegrees(-182.53), 5, 6, 2, false, false, PIDController(0.009, 0.003, 0.0003)),
            ModuleConstants(4, Rotation2d.fromDegrees(-115.76), 7, 8, 3, false, false, PIDController(0.009, 0.003, 0.0003))
    )
}
/*
HEY!
If you're planning on using the Swerve Drive Base in this library on your own robot,
make sure to edit these constants based on your own needs! Info on this may appear here later,
but as of now [[https://github.com/Team364/BaseFalconSwerve]] is a great resource
for most constants used in this library.
 */
package frc.robot.subsystems.intake

import com.revrobotics.CANSparkBase
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import frc.lib.SwerveDriveConstants.DrivetrainConsts.WHEEL_DIAMETER_METERS

object IntakeSubsystemConstants {
    data class ModuleConstants(
            val ANGLE_OFFSET: Rotation2d,
            val ARM_MOTOR_ID: Int,
            val INTAKE_MOTOR_ID: Int,
            val ENCODER_ID: Int,
            val PID_CONTROLLER: PIDController
    )

    object ArmMotor {

        // No idea what the gear ratio is, this is just a copy of SwerveDriveConstants
        // TODO: this
        const val GEAR_RATIO: Double = 8.14 / 1.0
        const val POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION: Double =
                (WHEEL_DIAMETER_METERS * Math.PI) / GEAR_RATIO
        const val VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND = POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION / 60.0
    }

    object IntakeMotor {

        const val GEAR_RATIO: Double = 8.14 / 1.0
        const val POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION: Double =
                (WHEEL_DIAMETER_METERS * Math.PI) / ArmMotor.GEAR_RATIO
        const val VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND = POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION / 60.0

    }

    object Encoder {
        const val INVERT: Boolean = false
        const val POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION: Double = 360.0
    }

    object IntakeConsts {        /* intake Constants */
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
        // only one intake module
        ModuleConstants(Rotation2d(0.0,0.0), 1,1,1, PIDController(0.009,0.003,0.0003))
    )
}
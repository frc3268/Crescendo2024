package frc

import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. inside the companion object). Do not put anything functional in this class.
 *
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
object Constants {

    val TRACK_WIDTH_METERS = Units.inchesToMeters(24.0)
    val WHEEL_BASE_METERS = Units.inchesToMeters(24.0)

    /* Swerve Profiling Values */
    const val MAX_SPEED_METERS_PER_SECOND = 5.0
    const val MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND = 200.0

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

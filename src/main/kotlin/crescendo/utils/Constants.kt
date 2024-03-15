package crescendo.utils

import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.DriverStation

/**
 * This class is only for constants used across multiple files. Constants used only within a single class can be found
 * in that class's companion object, which is at the top of each class.
 */
object Constants {

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

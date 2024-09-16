package frc.robot

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
    object OperatorConstants {
        const val kDriverControllerPort = 0
        const val STICK_DEADBAND = 0.1
    }

    enum class States{REAL, SIM, REPLAY}

    val mode = States.REAL

    const val TROUBLESHOOTING_TAB = "Troubleshooting"
}

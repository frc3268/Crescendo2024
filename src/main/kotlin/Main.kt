@file:JvmName("Main")

import edu.wpi.first.wpilibj.RobotBase

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what you are doing, do not modify this file except to
 * change the parameter class to the startRobot call.
 */
object Main {
    /**
     * Main initialization function. Do not perform any initialization here.
     */
    @JvmStatic
    fun main(args: Array<String>) {
        RobotBase.startRobot { crescendo.Robot() }
    }
}

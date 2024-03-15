package crescendo

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

/**
 * This class contains everything about the robot. Robot logic and structure should be defined and changed in
 * [RobotLogic] since we don't want to initialize anything until the robot is physically ready. [robotLogic] is
 * initialized in [robotInit].
 *
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
class Robot: TimedRobot() {
    private var autonomousCommand: Command? = null
    private var robotLogic: RobotLogic? = null

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        // Perform all button bindings and put the autonomous mode chooser on the dashboard.
        robotLogic = RobotLogic()
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()
    }

    /** This function is called once each time the robot enters Disabled mode.  */
    override fun disabledInit() {}

    /** This function is called periodically when disabled.  */
    override fun disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your [RobotLogic] class.  */
    override fun autonomousInit() {
        // TODO is this needed? -- Weiju
        autonomousCommand = robotLogic?.autoChooser?.selected
        robotLogic?.swerveSubsystem?.zeroPoseToCameraPosition()
        /*
        TODO: this may need to be deleted
        robotLogic?.swerveSubsystem?.zeroPoseToFieldPositionCommand(
            SwerveDriveConstants.startCoordinates
                [DriverStation.getAlliance().get()]!!
                [DriverStation.getLocation().asInt - 1]
        )
        */


        // Schedule the autonomous command (example)
        autonomousCommand?.schedule()
    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {}

    /** This function is called once when teleop is enabled.  */
    override fun teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        autonomousCommand?.cancel()
        robotLogic?.teleopCommand?.schedule()
    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {}

    /** This function is called once when test mode is enabled.  */
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {}

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {}

    /** This function is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {}
}

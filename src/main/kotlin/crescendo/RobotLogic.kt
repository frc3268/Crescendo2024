package crescendo

import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import crescendo.subsystems.*
import crescendo.subsystems.climber.*
import crescendo.subsystems.swerve.SwerveSubsystem
import crescendo.utils.JoystickDriveCommand
import kotlin.math.*

/**
 * Contains all high-level robot declarations and logic. When instantiated, joystick controls and ShuffleBoard are set
 * up. This should only be instantiated by [Robot].
 *
 * # [Subsystem]s
 * The robot contains several subsystems like [swerveSubsystem], each of which represents a part of the robot. Most
 * subsystems instantiate [crescendo.utils.Motor]s or other classes from WPILIB that interface with physical objects on
 * the robot. Subsystems also include some functions for things you might want to do with that subsystem. For example,
 * [IntakeSubsystem] contains a member that represents the intake motor, and it has functions such as
 * `intakeAndStopCommand()` that allows you to perform sequences of actions with the intake motor that are commonly
 * useful.
 *
 * # [Command]s
 * All functions in this class return Commands, which are classes that define actions or sequences of actions. Within
 * Commands, e.g. [goto], we call the functions we defined in our Subsystems to actually execute them. Note that these
 * functions themselves do not do anything; they simply return an instance of a Command which has to actually be run
 * to do what it's defined to do.
 */
class RobotLogic {
    companion object {
        const val K_DRIVER_CONTROLLER_PORT = 0
    }

    private val generalTab = Shuffleboard.getTab("General")
    private val troubleshootingTab = Shuffleboard.getTab("Troubleshooting")

    val swerveSubsystem = SwerveSubsystem(Pose2d())
    val intakeSubsystem = IntakeSubsystem()
    val shooterSubsystem = ShooterSubsystem()
    val leftClimberSubsystem = LeftClimberSubsystem()
    val rightClimberSubsystem = RightClimberSubsystem()

    private val driverController = CommandXboxController(K_DRIVER_CONTROLLER_PORT)

    val autoChooser = SendableChooser<Command>()

    /**
     * In autonomous, the bot will run whatever command is selected in ShuffleBoard.
     */
    val autonomousCommand: Command
        get() = autoChooser.selected

    /**
     * In teleop, the robot runs [JoystickDriveCommand], which constantly listens for joystick inputs and executes
     * them.
     */
    val teleopCommand = JoystickDriveCommand(
        swerveSubsystem,
        { driverController.getRawAxis(1) },
        { -driverController.getRawAxis(0) },
        { -driverController.getRawAxis(4) },
        { true }
    )

    val ring1BooleanBox = generalTab.add("Collect ring 1?", false).withWidget(BuiltInWidgets.kBooleanBox)
    val ring2BooleanBox = generalTab.add("Collect ring 2?", false).withWidget(BuiltInWidgets.kBooleanBox)
    val ring3BooleanBox = generalTab.add("Collect ring 3?", false).withWidget(BuiltInWidgets.kBooleanBox)

    /**
     * Initializes the robot, sets up ShuffleBoard, and configures joystick bindings.
     */
    init {
        swerveSubsystem.defaultCommand = teleopCommand

        /* -----------------------------------------------------------------------------------------------------------*/
        // SHUFFLEBOARD
        /* -----------------------------------------------------------------------------------------------------------*/

        generalTab
            .add("Autonomous Mode", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1)

        autoChooser.setDefaultOption("Taxi", taxiAuto())
        autoChooser.addOption("Do nothing", WaitCommand(1.0))
        autoChooser.addOption("Shoot to speaker", shootSpeakerCommand())
        autoChooser.addOption("Shoot to speaker + taxi", shootSpeakerCommand().andThen(taxiAuto()))
        autoChooser.addOption("Shoot, Intake, Shoot", driveUpShootSpeakerAndReturnToRingsCommand())
        autoChooser.addOption("Shoot, Take rings and Shoot", collectStartingRingsAndShoot(1, arrayOf(ring1BooleanBox.entry, ring2BooleanBox.entry, ring3BooleanBox.entry)))

        //test individual commands
        autoChooser.addOption("test go to speaker (bottom)", goToSpeakerCommand(1))
        autoChooser.addOption("test go to speaker (middle)", goToSpeakerCommand(2))
        autoChooser.addOption("test go to speaker (top)", goToSpeakerCommand(3))
        autoChooser.addOption("test go to amp", goToAmpCommand())
        autoChooser.addOption("test go to source closerToBaseLine=false", goToSourceCommand(false))
        autoChooser.addOption("test go to source closerToBaseLine=false", goToSourceCommand(true))
        autoChooser.addOption("test go within speaker (obsolete?)", goWithinSpeakerCommand())
        autoChooser.addOption("test intake and up", intakeAndUpCommand())
        autoChooser.addOption("test intake from ground", intakeNoteCommand())
        autoChooser.addOption("climber up", climberUp())
        autoChooser.addOption("climber down", climberDown())
        autoChooser.addOption("climber stop", climberStop())
        autoChooser.addOption("test shootSpeakerCommand", shootSpeakerCommand())
        autoChooser.addOption("test Shooter only", testShooterCommand())
        autoChooser.addOption("test shoot amp", shootAmpCommand())
        autoChooser.addOption("test source intake", sourceIntakeCommand())

        generalTab.add("shoot speaker", shootSpeakerCommand()).withWidget(BuiltInWidgets.kCommand)
        generalTab.add("Ground intake", intakeAndUpCommand()).withWidget(BuiltInWidgets.kCommand)
        generalTab.add("Source Intake", sourceIntakeCommand())

        generalTab.add("CLIMBERS down", climberDown()).withWidget(BuiltInWidgets.kCommand)
        generalTab.add("CLIMBERS up", climberUp()).withWidget(BuiltInWidgets.kCommand)
        generalTab.add("CLIMBERS stop", climberStop()).withWidget(BuiltInWidgets.kCommand)

        // Troubleshooting tab holds manual controls for the climber and a reset for the arm encoder
        troubleshootingTab.add("CLIMBER L down", leftClimberSubsystem.testDown()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("CLIMBER L up", leftClimberSubsystem.testUp()).withWidget(BuiltInWidgets.kCommand)

        troubleshootingTab.add("CLIMBER R down", rightClimberSubsystem.testDown()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("CLIMBER R up", rightClimberSubsystem.testUp()).withWidget(BuiltInWidgets.kCommand)

        troubleshootingTab.add("Zero ARM ENCODER", intakeSubsystem.zeroArmEncoderCommand()).withWidget(BuiltInWidgets.kCommand)

        troubleshootingTab.add("CLIMBERS reset", climberStop()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("CLIMBERS stop", leftClimberSubsystem.stop().alongWith(rightClimberSubsystem.stop())).withWidget(BuiltInWidgets.kCommand)

        /* -----------------------------------------------------------------------------------------------------------*/
        // CONFIGURE JOYSTICK BINDINGS
        /* -----------------------------------------------------------------------------------------------------------*/

        // Schedule ExampleCommand when exampleCondition changes to true
        //Trigger { exampleSubsystem.exampleCondition() }.onTrue(ExampleCommand(exampleSubsystem))

        /*
        LT (Intake):
            runs intake
            (not arm!)
         */
        //driverController.leftTrigger().onTrue(intakeSubsystem.runIntakeCommand())
        //driverController.leftTrigger().onFalse(intakeSubsystem.stopIntake())

        /*
        RT (Shoot):
            1) Rev up shooter
            2) Run intake in reverse to feed it into shooter
            This assumes the arm is already up. If it's down, the note will be shot back onto the ground.
         */
        //driverController.rightTrigger().onTrue(testShooterCommand())
        driverController.rightTrigger().onTrue(shootSpeakerCommand())
        /*
        LB: Arm up
         */
        driverController.leftBumper().onTrue(intakeSubsystem.armDownCommand())

        /*
        RB: Arm Up
         */
        driverController.rightBumper().onTrue(intakeSubsystem.armUpCommand())

        /*
        Y (EMERGENCY STOP): Stop the intake gears, the arm, and the shooter.
        (The intention is to be able to prevent damage if the encoder is faulty and damaging any moving parts.)
         */
        driverController.y().onTrue(emergencyStopCommand())

        /*
        A runs outake
         */
        driverController.a().onTrue(intakeSubsystem.runOuttakeCommand())
        driverController.a().onFalse(intakeSubsystem.stopIntake())

        /*
        X does source intake
        arm up
        run shooter in reverse
        intake
        stop intake
        stop shooter
         */
        driverController.x().onTrue(sourceIntakeCommand())

        /*
        B does arm down, intake note, arm up
        */
        driverController.b().onTrue(intakeAndUpCommand())

        /*
        POV up and down bring arm up and down
         */
        //driverController.povUp().onTrue(intakeSubsystem.armUpCommand())
        //driverController.povDown().onTrue(intakeSubsystem.armDownCommand())
    }

    /**
     * Drives to [goalIfRed] if the robot is on the red team, otherwise [goalOtherwise].
     *
     * [goalOtherwise] is the default if there is no color.
     */
    fun goto(goalIfRed: Pose2d, goalOtherwise: Pose2d): Command {
        val color = DriverStation.getAlliance()
        return SequentialCommandGroup(
            swerveSubsystem.moveToPoseCommand(
                if (color.isPresent && color.get() == DriverStation.Alliance.Red)
                    goalIfRed
                else
                    goalOtherwise
            ),
            InstantCommand({ swerveSubsystem.stop() })
        )
    }

    fun taxiAuto() =
        goto(
            // should make the robot move around 2 meters as the starting zone is ~193 cm or 1.93m
            Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0.0)),
            Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0.0)),
        )

    fun goToSpeakerCommand(location: Int?): Command {
        var locationVar =
            if (location == null || location > 3 || location < 1)
                1
            else location
        // location maps to one of three points on the subwoofer
        // 1 -> closer to the source
        // 2 -> in the middle
        // 3 -> furthest from the source
        return when (location) {
            1 -> goto(
                Pose2d(16.0824625,4.96116891, Rotation2d.fromDegrees(-60.0)),
                Pose2d(0.958787,4.96116891, Rotation2d.fromDegrees(60.0))
            )
            2 -> goto(
                Pose2d(15.256, 5.547868, Rotation2d.fromDegrees(0.0)),
                Pose2d(1.6096, 5.547868, Rotation2d.fromDegrees(180.0))
            )
            3 -> goto(
                Pose2d(16.0824625,6.54503, Rotation2d.fromDegrees(60.0)),
                Pose2d(0.958787,6.54503, Rotation2d.fromDegrees(-60.0))
            )
            else -> WaitCommand(1.0)
        }
    }

    fun goToAmpCommand() =
        goto(
            Pose2d(14.929358, 8.2042, Rotation2d.fromDegrees(270.0)),
            Pose2d(1.84404, 8.2042, Rotation2d.fromDegrees(270.0))
        )

    // documentation: what is closerToBaseLine?
    fun goToSourceCommand(closerToBaseLine: Boolean) =
        goto(
            if (closerToBaseLine) Pose2d(0.356108, 0.883666, Rotation2d.fromDegrees(60.0))
            else Pose2d(1.461516, 0.245872, Rotation2d.fromDegrees(60.0)),

            if (closerToBaseLine) Pose2d(15.079472, 0.245872, Rotation2d.fromDegrees(120.0))
            else Pose2d(16.185134, 0.883666, Rotation2d.fromDegrees(120.0))
        )

    fun goToSourceAndIntakeCommand(closerToBaseLine: Boolean): Command =
        SequentialCommandGroup(
            goToSourceCommand(closerToBaseLine),
            sourceIntakeCommand()
        )

    fun goWithinSpeakerCommand(): Command {
        val color = DriverStation.getAlliance()
        //todo: fix x
        val to =
            if (color.isPresent && color.get() == DriverStation.Alliance.Red)
                Pose2d(14.579342, 5.547868, Rotation2d.fromDegrees(180.0))
            else
                Pose2d(0.0, 5.547868, Rotation2d.fromDegrees(0.0))
        //todo: make this real
        val pose = swerveSubsystem.getPose()
        val c = 1.0
        val theta = Rotation2d.fromDegrees(atan((pose.y - to.y) / (pose.x - to.x)))
        return SequentialCommandGroup(
            swerveSubsystem.moveToPoseCommand(
                Pose2d(
                    to.x + cos(theta.radians) * c,
                    to.y + sin(theta.radians) * c,
                    theta + pose.rotation)
            ),
            InstantCommand({ swerveSubsystem.stop() })
        )
    }

    fun driveUpAndShootSpeakerCommand(): Command =
        SequentialCommandGroup(
            goToSpeakerCommand(1),
            shootSpeakerCommand()
        )

    fun intakeAndUpCommand(): Command =
        SequentialCommandGroup(
            intakeSubsystem.armDownCommand(),
            intakeSubsystem.intakeAndStopCommand(),
            intakeSubsystem.armUpCommand(),
        )

    fun intakeNoteCommand(): Command =
        SequentialCommandGroup(
            intakeSubsystem.takeInCommand(),
            intakeSubsystem.stopIntake()
        )

    fun climberUp() =
        ParallelCommandGroup(
            leftClimberSubsystem.up(),
            rightClimberSubsystem.up()
        )

    fun climberDown() =
        ParallelCommandGroup(
            leftClimberSubsystem.down(),
            rightClimberSubsystem.down()
        )

    fun climberStop() =
        ParallelCommandGroup(
            leftClimberSubsystem.stop(),
            rightClimberSubsystem.stop()
        )

    fun shootSpeakerCommand(): Command =
        SequentialCommandGroup(
            shooterSubsystem.speakerCommand(),
            WaitCommand(1.0),
            intakeSubsystem.takeOutCommand(),
            WaitCommand(1.2),
            shooterSubsystem.stopCommand(),
            intakeSubsystem.stopIntake()
        )

    //Needed function to test controls that didn't involve intake
    fun testShooterCommand(): Command =
        SequentialCommandGroup(
            shooterSubsystem.speakerCommand(),
            WaitCommand(1.0),
            shooterSubsystem.stopCommand()
        )

    fun shootAmpCommand(): Command =
        SequentialCommandGroup(
            shooterSubsystem.ampCommand(),
            intakeSubsystem.takeOutCommand(),
            shooterSubsystem.stopCommand()
        )

    fun sourceIntakeCommand(): Command =
        SequentialCommandGroup(
            intakeSubsystem.armUpCommand(),
            shooterSubsystem.takeInCommand(),
            intakeSubsystem.runIntakeCommand(),
            WaitCommand(0.5),
            intakeSubsystem.stopIntake()
        )

    fun driveUpAndIntakeSourceCommand(closerToBaseLine: Boolean): Command =
        SequentialCommandGroup(
            goToSourceCommand(closerToBaseLine),//fix closer to baseline
            sourceIntakeCommand()
        )

    fun driveUpShootSpeakerAndReturnToRingsCommand(): Command =
        SequentialCommandGroup(
            goToSpeakerCommand(1),
            shootSpeakerCommand(),
            /** 8.2927 - 0.1524 - 0.3556 math for blue, 8.2927 + 0.1524 + 0.3556 math for red
            these should be correct but someone should check my math
            the y should be correct and the x was found by adding the width of the starting zone + the width of the distance from the
            starting zone to the ring, then depending on what team we are on the width of the ring + about half a foot is added or subtracted**/
            /** 8.2927 - 0.1524 - 0.3556 math for blue, 8.2927 + 0.1524 + 0.3556 math for red
            these should be correct but someone should check my math
            the y should be correct and the x was found by adding the width of the starting zone + the width of the distance from the
            starting zone to the ring, then depending on what team we are on the width of the ring + about half a foot is added or subtracted**/
            goto(Pose2d(8.8007, 0.752856, Rotation2d.fromDegrees(0.0)), Pose2d(7.7847, 0.752856, Rotation2d.fromDegrees(0.0))),
            intakeAndUpCommand(),
            goToSpeakerCommand(1),
            shootSpeakerCommand())

    // the Ideal is we test this first
    // the rings param asks for an array of GenericEntries, which are the shuffleboard booleanbox things
    // from those entries, you can get the boolean array mentioned in the next line
    // the array [True, False, True] will pick up ring A then C and not B
    fun collectStartingRingsAndShoot(location: Int, rings: Array<GenericEntry>): Command {
        val sequence: SequentialCommandGroup = SequentialCommandGroup()
        sequence.addCommands(goToSpeakerCommand(location))
        sequence.addCommands(shootSpeakerCommand())
        return Commands.runOnce({
            // the ring right along the middle
            if (rings[0].getBoolean(false)) {
                sequence.addCommands(
                    goto(
                        Pose2d(14.127, 4.105656, Rotation2d.fromDegrees(180.0)),
                        Pose2d(2.413, 4.105656, Rotation2d.fromDegrees(0.0))
                    )
                )
                sequence.addCommands(intakeAndUpCommand())
                sequence.addCommands(goToSpeakerCommand(location))
                sequence.addCommands(shootSpeakerCommand())
            }
            // ring above the first ring
            if (rings[1].getBoolean(false)) {
                sequence.addCommands(
                    goto(
                        Pose2d(14.127, 5.553456, Rotation2d.fromDegrees(180.0)),
                        Pose2d(2.413, 5.553456, Rotation2d.fromDegrees(0.0))
                    )
                )
                sequence.addCommands(intakeAndUpCommand())
                sequence.addCommands(goToSpeakerCommand(location))
                sequence.addCommands(shootSpeakerCommand())
            }
            // ring above the second ring
            if (rings[2].getBoolean(false)) {
                // I kept the equations here in case we need to adjust for whatever reason the 0.1778 is half the width of the ring in meters, I haven't adjusted for how far away from the ring we have to be
                // Red Equation : Center of Ring + half the ring's size + a foot (for intake) Blue Equation : Center of Ring + half the ring's size + a foot (intake)
                sequence.addCommands(
                    goto(
                        Pose2d(13.6444 + 0.1778 + 0.3048, 7.001256, Rotation2d.fromDegrees(180.0)),
                        Pose2d(2.8956 - 0.1778 - 0.3048, 7.001256, Rotation2d.fromDegrees(0.0))
                    )
                )
                sequence.addCommands(intakeAndUpCommand())
                sequence.addCommands(goToSpeakerCommand(location))
                sequence.addCommands(shootSpeakerCommand())
            }
        }).andThen(
            sequence,
            // will go to the bottom ring if Red and the top ring if Blue
            goto(Pose2d(8.2927 + 0.1778 + 0.3048, 0.752856, Rotation2d.fromDegrees(180.0)), Pose2d(8.2927 - 0.1778 - 0.3048, 7.457144, Rotation2d.fromDegrees(0.0)))
        )
    }

    fun emergencyStopCommand(): Command =
        SequentialCommandGroup(
            shooterSubsystem.stopCommand(),
            intakeSubsystem.stopAllCommand()
        )
}

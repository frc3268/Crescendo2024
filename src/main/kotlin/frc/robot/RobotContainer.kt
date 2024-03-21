package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.shuffleboard.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.lib.basics.SwerveDriveBase
import frc.robot.commands.*
import frc.robot.subsystems.*

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    private val GeneralTab = Shuffleboard.getTab("General")
    private val TroubleshootingTab = Shuffleboard.getTab("Troubleshooting")

    val driveSubsystem = SwerveDriveBase(Pose2d())
    val intakeSubsystem = IntakeSubsystem()
    val shooterSubsystem = ShooterSubsystem()
    val leftClimberSubsystem = LeftClimberSubsystem()
    val rightClimberSubsystem = RightClimberSubsystem()

    private val driverController = CommandXboxController(Constants.OperatorConstants.kDriverControllerPort)

    val autochooser = SendableChooser<Command>()

    val teleopCommand = SwerveJoystickDrive(
        driveSubsystem,
        { driverController.getRawAxis(1) },
        { -driverController.getRawAxis(0) },
        { -driverController.getRawAxis(4) },
        { true }
    )

    val ring1BooleanBox = GeneralTab.add("Collect ring 1?", false).withWidget(BuiltInWidgets.kBooleanBox).entry
    val ring2BooleanBox = GeneralTab.add("Collect ring 2?", false).withWidget(BuiltInWidgets.kBooleanBox).entry
    val ring3BooleanBox = GeneralTab.add("Collect ring 3?", false).withWidget(BuiltInWidgets.kBooleanBox).entry
    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        driveSubsystem.defaultCommand = teleopCommand

        GeneralTab
            .add("Autonomous Mode", autochooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1)


        autochooser.setDefaultOption("Taxi", Autos.taxiAuto(driveSubsystem))
        autochooser.addOption("Do nothing", WaitCommand(1.0))
        autochooser.addOption("Shoot to speaker", Autos.shootSpeakerCommand(intakeSubsystem, shooterSubsystem))
        autochooser.addOption("Shoot to speaker + taxi", Autos.shootSpeakerCommand(intakeSubsystem, shooterSubsystem).andThen(Autos.taxiAuto(driveSubsystem)))
        autochooser.addOption("Shoot, Intake, Shoot", Autos.driveUpShootSpeakerAndReturnToRingsCommand(driveSubsystem, intakeSubsystem, shooterSubsystem))
        autochooser.addOption("Shoot, Take rings and Shoot", Autos.collectStartingRingsAndShoot(driveSubsystem, intakeSubsystem, shooterSubsystem, 1, arrayOf(ring1BooleanBox.value, ring2BooleanBox.value, ring3BooleanBox.value)))
        //autochooser.addOption("test Shooter only", Autos.testShooterCommand(shooterSubsystem))

        GeneralTab.add("shoot speaker", Autos.shootSpeakerCommand(intakeSubsystem, shooterSubsystem)).withWidget(BuiltInWidgets.kCommand)
        GeneralTab.add("Ground intake", Autos.intakeAndUpCommand(intakeSubsystem)).withWidget(BuiltInWidgets.kCommand)
        GeneralTab.add("Source Intake", Autos.sourceIntakeCommand(shooterSubsystem, intakeSubsystem))

        GeneralTab.add("CLIMBERS down", Autos.climberDown(leftClimberSubsystem, rightClimberSubsystem)).withWidget(BuiltInWidgets.kCommand)
        GeneralTab.add("CLIMBERS up", Autos.climberUp(leftClimberSubsystem, rightClimberSubsystem)).withWidget(BuiltInWidgets.kCommand)
        GeneralTab.add("CLIMBERS stop", Autos.climberStop(leftClimberSubsystem, rightClimberSubsystem)).withWidget(BuiltInWidgets.kCommand)

        // Troubleshooting tab holds manual controls for the climber and a reset for the arm encoder
        TroubleshootingTab.add("CLIMBER L down", leftClimberSubsystem.testdown()).withWidget(BuiltInWidgets.kCommand)
        TroubleshootingTab.add("CLIMBER L up", leftClimberSubsystem.testup()).withWidget(BuiltInWidgets.kCommand)

        TroubleshootingTab.add("CLIMBER R down", rightClimberSubsystem.testdown()).withWidget(BuiltInWidgets.kCommand)
        TroubleshootingTab.add("CLIMBER R up", rightClimberSubsystem.testup()).withWidget(BuiltInWidgets.kCommand)

        TroubleshootingTab.add("Zero ARM ENCODER", intakeSubsystem.zeroArmEncoderCommand()).withWidget(BuiltInWidgets.kCommand)

        TroubleshootingTab.add("CLIMBERS reset", Autos.climberStop(leftClimberSubsystem, rightClimberSubsystem)).withWidget(BuiltInWidgets.kCommand)
        TroubleshootingTab.add("CLIMBERS stop", leftClimberSubsystem.stop().alongWith(rightClimberSubsystem.stop())).withWidget(BuiltInWidgets.kCommand)


        // Configure the trigger bindings
        configureBindings()
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger#Trigger(java.util.function.BooleanSupplier)] constructor with an arbitrary
     * predicate, or via the named factories in [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for
     * [CommandXboxController]/[edu.wpi.first.wpilibj2.command.button.CommandPS4Controller] controllers
     * or [edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
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
        //driverController.rightTrigger().onTrue(Autos.testShooterCommand(shooterSubsystem))
        driverController.rightTrigger().onTrue(Autos.shootSpeakerCommand(intakeSubsystem, shooterSubsystem))
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
        driverController.y().onTrue(Autos.emergencyStopCommand(shooterSubsystem, intakeSubsystem))

        /*
        A runs outake
         */
        driverController.a().onTrue(intakeSubsystem.runOnceOuttake())
        driverController.a().onFalse(intakeSubsystem.stopIntake())

        /*
        X does source intake
        arm up
        run shooter in reverse
        intake
        stop intake
        stop shooter
         */
        driverController.x().onTrue(Autos.sourceIntakeCommand(shooterSubsystem, intakeSubsystem))

       /*
       B does arm down, intake note, arm up
        */
        driverController.b().onTrue(Autos.intakeAndUpCommand(intakeSubsystem))

        /*
        POV up and down bring arm up and down
         */
        //driverController.povUp().onTrue(intakeSubsystem.armUpCommand())
        //driverController.povDown().onTrue(intakeSubsystem.armDownCommand())
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() {
            // wait 3 seconds...
            return autochooser.selected
        }
}

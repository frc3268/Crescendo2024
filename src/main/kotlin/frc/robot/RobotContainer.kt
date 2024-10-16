package frc.robot

import edu.wpi.first.math.geometry.Pose2d

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.shuffleboard.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.lib.*
import frc.robot.commands.*
import frc.robot.subsystems.climber.LeftClimberSubsystem
import frc.robot.subsystems.climber.RightClimberSubsystem
import frc.robot.subsystems.intake.*
import frc.robot.subsystems.shooter.ShooterSubsystem
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.decodeFromStream
import java.io.File

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    private val GeneralTab = Shuffleboard.getTab("General")
    private val TroubleshootingTab = Shuffleboard.getTab(Constants.TROUBLESHOOTING_TAB)

    val intakeSubsystem = IntakeSubsystem()
    val shooterSubsystem = ShooterSubsystem()
    val leftClimberSubsystem = LeftClimberSubsystem()
    val rightClimberSubsystem = RightClimberSubsystem()
    val driveSubsystem = SwerveDriveBase(Pose2d())

    private val driverController = CommandXboxController(Constants.OperatorConstants.kDriverControllerPort)

    val autochooser = SendableChooser<Command>()
    val leftBumperChooser = SendableChooser<Command>()
    val xButtonChooser = SendableChooser<Command>()


    val teleopCommand = SwerveJoystickDrive(
        driveSubsystem,
        { driverController.getRawAxis(1) },
        { -driverController.getRawAxis(0) },
        { -driverController.getRawAxis(4) },
        { true }
    )

    val autos: MutableMap<String, Command> = mutableMapOf(
        "gotoSpeaker" to goToSpeakerCloser(),
        "gotoSpeakerCenter" to goto(FieldPositions.speakerCenter),
        "gotoSpeakerRight" to goto(FieldPositions.speakerRight),
        "gotoSpeakerLeft" to goto(FieldPositions.speakerLeft),
        "gotoAmp" to goto(FieldPositions.amp),
        "goToSourceCloserToBaseline" to goto(FieldPositions.sourceBaseline),
        "goToSourceFurtherFromBaseline" to goto(FieldPositions.sourceNotBaseline),
        "goToRing" to WaitCommand(1.0),
        //todo: other rings
    )

    fun goto(goal: FieldLocation): Command {
        val color = DriverStation.getAlliance()
        val to =
            if (color.isPresent && color.get() == DriverStation.Alliance.Red)
                goal.red
            else
                goal.blue
        return SwerveAutoDrive(
            to,
            Pose2d(0.1, 0.1, 10.0.rotation2dFromDeg()),
            driveSubsystem,
            { driverController.getRawAxis(1) },
            { -driverController.getRawAxis(0) },
            { -driverController.getRawAxis(4) },
        )
    }

    fun goToSpeakerCloser(): Command {
        var startingPose = Pose2d()
        return runOnce({
            startingPose = driveSubsystem.getPose()
        }, driveSubsystem).andThen(
            //red
            goto(
                FieldPositions.closest(
                    startingPose,
                    listOf(
                        FieldPositions.speakerRight,
                        FieldPositions.speakerCenter,
                        FieldPositions.speakerLeft
                    )
                )
            )
        )
    }


    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    //on true
    //while true
    //toggle
    init {
        AutoCommand(
            SequentialCommandGroup(
                intakeSubsystem.armDownCommand(),
                intakeSubsystem.takeInCommand(),
                intakeSubsystem.stopIntake(),
                intakeSubsystem.armUpCommand(),
            ),
            autos,
            name = "Intake and up",
            binding = driverController.leftTrigger(),
        )

        AutoCommand(
            shooterSubsystem.runAtSpeedCommand(0.8).andThen(shooterSubsystem.stopCommand()),
            autos,
            name = "Rev Shooters",
            binding = driverController.rightBumper(),
            triggerType = TriggerType.TOGGLE
        )

        AutoCommand(
            intakeSubsystem.takeOutCommand(),
            autos,
            name = "Outtake Note",
            binding = driverController.rightTrigger(),
            triggerType = TriggerType.WHILE_TRUE
        )

        AutoCommand(
            ParallelCommandGroup(
                leftClimberSubsystem.up(),
                rightClimberSubsystem.up()
            ),
            name = "Climbers Up",
            shuffleboardTab = GeneralTab
        )

        AutoCommand(
            ParallelCommandGroup(
                leftClimberSubsystem.down(),
                rightClimberSubsystem.down()
            ),
            name = "Climbers Down",
            shuffleboardTab = GeneralTab
        )

        AutoCommand(
            ParallelCommandGroup(
                leftClimberSubsystem.stop(),
                rightClimberSubsystem.stop()
            ),
            name = "Climbers Stop",
            shuffleboardTab = TroubleshootingTab
        )

        AutoCommand(
            SequentialCommandGroup(
                shooterSubsystem.stopCommand(),
                intakeSubsystem.stopAllCommand()
            ),
            name = "Emergency STOP",
            shuffleboardTab = GeneralTab,
            binding = driverController.y()
        )

        driveSubsystem.defaultCommand = teleopCommand

        GeneralTab
            .add("Autonomous Mode", autochooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1)
        GeneralTab
            .add("LB Command", leftBumperChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(2, 0)
            .withSize(2, 1)

        // Troubleshooting tab holds manual controls for the climber and a reset for the arm encoder
        TroubleshootingTab.add("CLIMBER L down", leftClimberSubsystem.testdown()).withWidget(BuiltInWidgets.kCommand)
        TroubleshootingTab.add("CLIMBER L up", leftClimberSubsystem.testup()).withWidget(BuiltInWidgets.kCommand)

        TroubleshootingTab.add("CLIMBER R down", rightClimberSubsystem.testdown()).withWidget(BuiltInWidgets.kCommand)
        TroubleshootingTab.add("CLIMBER R up", rightClimberSubsystem.testup()).withWidget(BuiltInWidgets.kCommand)

        TroubleshootingTab.add("Zero ARM ENCODER", intakeSubsystem.zeroArmEncoderCommand()).withWidget(BuiltInWidgets.kCommand)


        for (file:File in File(Filesystem.getDeployDirectory().toString() + "/paths").listFiles()?.filter { it.isFile }!!){
            autochooser.addOption(file.name,Json.decodeFromStream<AutoSequence>(
               file.inputStream()
            ).toCommandGroup(autos))
        }

        for (file:File in File(Filesystem.getDeployDirectory().toString() + "/buttons").listFiles()?.filter { it.isFile }!!){
            GeneralTab.add(file.name,Json.decodeFromStream<AutoSequence>(
                file.inputStream()
            ).toCommandGroup(autos)).withWidget(BuiltInWidgets.kCommand)
        }
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() {
            return autochooser.selected
        }
}

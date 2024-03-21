package frc.robot.commands

import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import frc.lib.basics.SwerveDriveBase
import frc.lib.utils.*
import frc.robot.subsystems.*
import java.util.function.BooleanSupplier
import kotlin.math.*

class Autos private constructor() {
    init {
        throw UnsupportedOperationException("This is a utility class!")
    }

    companion object {


        /** Example static factory for an autonomous command.  */
        fun exampleAuto(subsystem: ExampleSubsystem): Command {
            return Commands.sequence(subsystem.exampleMethodCommand(), ExampleCommand(subsystem))
        }

        /**
         * Drives to [goalIfRed] if the robot is on the red team, otherwise [goalOtherwise].
         *
         * [goalOtherwise] is the default if there is no color.
         */
        fun goto(drive: SwerveDriveBase, goalIfRed: Pose2d, goalOtherwise: Pose2d): Command {
            val color = DriverStation.getAlliance()
            val to =
                    if (color.isPresent && color.get() == DriverStation.Alliance.Red)
                        goalIfRed
                    else
                        goalOtherwise
            return SequentialCommandGroup(
                    drive.moveToPoseCommand(to),
                    InstantCommand({ drive.stop() })
            )
        }

        fun taxiAuto(drive: SwerveDriveBase): Command =
                goto(
                        drive,
                        // should make the robot move around 2 meters as the starting zone is ~193 cm or 1.93m
                        Pose2d(1.0, 0.0, 0.0.rotation2dFromDeg()),
                        Pose2d(1.0, 0.0, 0.0.rotation2dFromDeg()),
                )

        fun goToSpeakerCommand(drive: SwerveDriveBase, location:Int?): Command {
            var locationVar = location
            if (locationVar == null || locationVar > 3 || locationVar < 1) {
                locationVar = 1
            }
            // location maps to one of three points on the subwoofer
            // 1 -> closer to the source
            // 2 -> in the middle
            // 3 -> furthest from the source
            when (location) {
                1 -> return goto(
                        drive,
                        Pose2d(16.0824625,4.96116891,-60.0.rotation2dFromDeg()),
                        Pose2d(0.958787,4.96116891,60.0.rotation2dFromDeg())
                )
                2 -> return goto(
                    drive,
                    Pose2d(15.256, 5.547868, 0.0.rotation2dFromDeg()),
                    Pose2d(1.6096, 5.547868, 180.0.rotation2dFromDeg())
                )
                3 -> return goto(
                        drive,
                        Pose2d(16.0824625,6.54503,60.0.rotation2dFromDeg()),
                        Pose2d(0.958787,6.54503,-60.0.rotation2dFromDeg())
                )
            }



            return WaitCommand(1.0)
        }

        fun goToAmpCommand(drive: SwerveDriveBase): Command =
                goto(
                        drive,
                        Pose2d(14.929358, 8.2042, 270.0.rotation2dFromDeg()),
                        Pose2d(1.84404, 8.2042, 270.0.rotation2dFromDeg())
                )

        // documentation: what is closerToBaseLine?
        fun goToSourceCommand(drive: SwerveDriveBase, closerToBaseLine: Boolean): Command =
                goto(
                        drive,

                        if (closerToBaseLine) Pose2d(0.356108, 0.883666, 60.0.rotation2dFromDeg())
                        else Pose2d(1.461516, 0.245872, 60.0.rotation2dFromDeg()),

                        if (closerToBaseLine) Pose2d(15.079472, 0.245872, 120.0.rotation2dFromDeg())
                        else Pose2d(16.185134, 0.883666, 120.0.rotation2dFromDeg())
                )

        fun goToSourceAndIntakeCommand(drive: SwerveDriveBase, closerToBaseLine: Boolean, shooter: ShooterSubsystem, intake: IntakeSubsystem): Command =
                SequentialCommandGroup(
                        goToSourceCommand(drive, closerToBaseLine),
                        sourceIntakeCommand(shooter, intake)
                )

        fun goWithinSpeakerCommand(drive: SwerveDriveBase): Command {
            val color = DriverStation.getAlliance()
            //todo: fix x
            val to =
                    if (color.isPresent && color.get() == DriverStation.Alliance.Red)
                        Pose2d(14.579342, 5.547868, 180.0.rotation2dFromDeg())
                    else
                        Pose2d(0.0, 5.547868, 0.0.rotation2dFromDeg())
            //todo: make this real
            val pose = drive.getPose()
            val c = 1.0
            val theta = atan((pose.y - to.y) / (pose.x - to.x)).rotation2dFromDeg()
            return SequentialCommandGroup(drive.moveToPoseCommand(Pose2d(to.x + cos(theta.radians) * c, to.y + sin(theta.radians) * c, theta + pose.rotation)),
                    InstantCommand({ drive.stop() }))

        }

        fun driveUpAndShootSpeakerCommand(drive: SwerveDriveBase, intake: IntakeSubsystem, shooter: ShooterSubsystem): Command =
                SequentialCommandGroup(
                        goToSpeakerCommand(drive, 1),
                        shootSpeakerCommand(intake, shooter)
                )

        fun intakeAndUpCommand(intake: IntakeSubsystem): Command =
                SequentialCommandGroup(
                        intake.armDownCommand(),
                        intake.intakeAndStopCommand(),
                        intake.armUpCommand(),
                )

        fun intakeNoteCommand(intake: IntakeSubsystem): Command =
                SequentialCommandGroup(
                        intake.takeInCommand(),
                        intake.stopIntake()
                )

        fun climberUp(left: LeftClimberSubsystem, right: RightClimberSubsystem): ParallelCommandGroup =
                ParallelCommandGroup(
                        left.up(),
                        right.up()
                )

        fun climberDown(left: LeftClimberSubsystem, right: RightClimberSubsystem): ParallelCommandGroup =
                ParallelCommandGroup(
                        left.down(),
                        right.down()
                )

        fun climberStop(left: LeftClimberSubsystem, right: RightClimberSubsystem): ParallelCommandGroup =
                ParallelCommandGroup(
                        left.stop(),
                        right.stop()
                )

        fun shootSpeakerCommand(intake: IntakeSubsystem, shooter: ShooterSubsystem): Command =
            SequentialCommandGroup(
                shooter.speakerCommand(),
                WaitCommand(1.0),
                intake.takeOutCommand(),
                WaitCommand(1.2),
                shooter.stopCommand(),
                intake.stopIntake()
            )

        //Needed function to test controls that didn't involve intake
        fun testShooterCommand(shooter: ShooterSubsystem):Command =
            SequentialCommandGroup(
                shooter.speakerCommand(),
                WaitCommand(1.0),
                shooter.stopCommand()
            )

        fun shootAmpCommand(intake: IntakeSubsystem, shooter: ShooterSubsystem): Command =
                SequentialCommandGroup(
                        shooter.ampCommand(),
                        intake.takeOutCommand(),
                        shooter.stopCommand()
                )

        fun sourceIntakeCommand(shooter: ShooterSubsystem, intake: IntakeSubsystem): Command =
                SequentialCommandGroup(
                        intake.armUpCommand(),
                        shooter.takeInCommand(),
                        intake.runIntakeCommand(),
                        WaitCommand(0.5),
                        intake.stopIntake()
                )

        fun driveUpAndIntakeSourceCommand(drive: SwerveDriveBase, shooter: ShooterSubsystem, closerToBaseLine: Boolean, intake: IntakeSubsystem): Command =
                SequentialCommandGroup(
                        goToSourceCommand(drive, closerToBaseLine),//fix closer to baseline
                        sourceIntakeCommand(shooter, intake)
                )

        fun driveUpShootSpeakerAndReturnToRingsCommand(drive: SwerveDriveBase, intake: IntakeSubsystem, shooter: ShooterSubsystem): Command =
                SequentialCommandGroup(
                        goToSpeakerCommand(drive, 1),
                        shootSpeakerCommand(intake, shooter),
                        /** 8.2927 - 0.1524 - 0.3556 math for blue, 8.2927 + 0.1524 + 0.3556 math for red
                        these should be correct but someone should check my math
                        the y should be correct and the x was found by adding the width of the starting zone + the width of the distance from the
                        starting zone to the ring, then depending on what team we are on the width of the ring + about half a foot is added or subtracted**/
                        goto(drive, Pose2d(8.8007, 0.752856, 0.0.rotation2dFromDeg()), Pose2d(7.7847, 0.752856, 0.0.rotation2dFromDeg())),
                        intakeAndUpCommand(intake),
                        goToSpeakerCommand(drive, 1),
                        shootSpeakerCommand(intake, shooter))

        // the Ideal is we test this first
        // the rings param asks for an array of GenericEntries, which are the shuffleboard booleanbox things
        // from those entries, you can get the boolean array mentioned in the next line
        // the array [True, False, True] will pick up ring A then C and not B
        fun collectStartingRingsAndShoot(drive: SwerveDriveBase, intake: IntakeSubsystem, shooter: ShooterSubsystem, location: Int, rings:Array<GenericEntry>): Command {
            val sequence: SequentialCommandGroup = SequentialCommandGroup()
            sequence.addCommands(goToSpeakerCommand(drive, location))
            sequence.addCommands(shootSpeakerCommand(intake, shooter))
            return runOnce({
                // the ring right along the middle
                if (rings[0].getBoolean(false)) {
                    sequence.addCommands(goto(drive, Pose2d(14.127, 4.105656, 180.0.rotation2dFromDeg()), Pose2d(2.413, 4.105656, 0.0.rotation2dFromDeg())))
                    sequence.addCommands(intakeAndUpCommand(intake))
                    sequence.addCommands(goToSpeakerCommand(drive, location))
                    sequence.addCommands(shootSpeakerCommand(intake, shooter))
                }
                // ring above the first ring
                if (rings[1].getBoolean(false)) {
                    sequence.addCommands(goto(drive, Pose2d(14.127, 5.553456, 180.0.rotation2dFromDeg()), Pose2d(2.413, 5.553456, 0.0.rotation2dFromDeg())))
                    sequence.addCommands(intakeAndUpCommand(intake))
                    sequence.addCommands(goToSpeakerCommand(drive, location))
                    sequence.addCommands(shootSpeakerCommand(intake, shooter))
                }
                // ring above the second ring
                if (rings[2].getBoolean(false)) {
                    // I kept the equations here in case we need to adjust for whatever reason the 0.1778 is half the width of the ring in meters, I haven't adjusted for how far away from the ring we have to be
                    // Red Equation : Center of Ring + half the ring's size + a foot (for intake) Blue Equation : Center of Ring + half the ring's size + a foot (intake)
                    sequence.addCommands(goto(drive, Pose2d(13.6444 + 0.1778 + 0.3048, 7.001256, 180.0.rotation2dFromDeg()), Pose2d(2.8956 - 0.1778 - 0.3048, 7.001256, 0.0.rotation2dFromDeg())))
                    sequence.addCommands(intakeAndUpCommand(intake))
                    sequence.addCommands(goToSpeakerCommand(drive, location))
                    sequence.addCommands(shootSpeakerCommand(intake, shooter))
                }
            }, drive, intake, shooter).andThen(
                sequence,
                // will go to the bottom ring if Red and the top ring if Blue
                goto(drive, Pose2d(8.2927 + 0.1778 + 0.3048, 0.752856, 180.0.rotation2dFromDeg()), Pose2d(8.2927 - 0.1778 - 0.3048, 7.457144, 0.0.rotation2dFromDeg()))
            )
        }
        
        fun emergencyStopCommand(shooter: ShooterSubsystem, intake: IntakeSubsystem): Command =
            SequentialCommandGroup(
                shooter.stopCommand(),
                intake.stopAllCommand()
            )
    }
}



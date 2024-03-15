package frc.subsystems.swerve

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.*
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.*
import frc.Constants
import frc.subsystems.Camera
import org.photonvision.EstimatedRobotPose
import java.util.*
import kotlin.math.*

class SwerveDriveBase(startingPose: Pose2d) : SubsystemBase() {
    companion object {
        val xPIDController = PIDController(1.5,1.0,0.8)
        val yPIDController = PIDController(1.5,1.0,0.8)
        val thetaPIDController = PIDController(2.0,0.3,0.0)

        //in the order they appear in modules list
        //assuming that 0,0 is the center of the robot, and (+,+) means (left, front)
        val kinematics =
            SwerveDriveKinematics(
                Translation2d(-Constants.WHEEL_BASE_METERS / 2.0, -Constants.TRACK_WIDTH_METERS / 2.0),
                Translation2d(-Constants.WHEEL_BASE_METERS / 2.0, Constants.TRACK_WIDTH_METERS / 2.0),
                Translation2d(Constants.WHEEL_BASE_METERS / 2.0, Constants.TRACK_WIDTH_METERS / 2.0),
                Translation2d(Constants.WHEEL_BASE_METERS / 2.0, -Constants.TRACK_WIDTH_METERS / 2.0),
            )
    }

    val field = Field2d()
    private val ShuffleboardTab = Shuffleboard.getTab("Drivetrain")

    var poseEstimator: SwerveDrivePoseEstimator
    private val modules =
        listOf(
            SwerveModule(1, Rotation2d.fromDegrees(-253.36), 1, 2, 0, false, false, PIDController(0.012, 0.003, 0.0003)),
            SwerveModule(2, Rotation2d.fromDegrees(-7.66), 3, 4, 1, false, false, PIDController(0.0012, 0.003, 0.0003)),
            SwerveModule(3, Rotation2d.fromDegrees(-182.53), 5, 6, 2, false, false, PIDController(0.012, 0.003, 0.0003)),
            SwerveModule(4, Rotation2d.fromDegrees(-115.76), 7, 8, 3, false, false, PIDController(0.012, 0.003, 0.0003))
        )
    private val gyro = AHRS(SPI.Port.kMXP)

    private var joystickControlledEntry: GenericEntry = ShuffleboardTab
        .add("Joystick Control", true)
        .withWidget("Toggle Button")
        .withProperties(mapOf("colorWhenTrue" to "green", "colorWhenFalse" to "maroon"))
        .getEntry()

    private var poseXEntry = ShuffleboardTab.add("Pose X", 0.0).entry
    private var poseYEntry = ShuffleboardTab.add("Pose Y", 0.0).entry
    private var seesAprilTag = ShuffleboardTab.add("Sees April Tag?", false).withWidget(BuiltInWidgets.kBooleanBox).entry

    private var yawOffset:Double = 0.0

    private val camera = Camera("hawkeye", "")

    init {
        thetaPIDController.enableContinuousInput(360.0, 0.0)
        zeroYaw()
        //https://github.com/Team364/BaseFalconSwerve/issues/8#issuecomment-1384799539
        Timer.delay(1.0)
        resetModulesToAbsolute()
        ShuffleboardTab.add("Stop", stopCommand()).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("Zero Heading", zeroHeadingCommand()).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("Dig In", digInCommand()).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("Robot Heading", gyro).withWidget(BuiltInWidgets.kGyro)




        ShuffleboardTab.add(field).withWidget(BuiltInWidgets.kField)

        poseEstimator = SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), startingPose, VecBuilder.fill(0.1, 0.1, 0.1),  VecBuilder.fill(0.5, 0.5, 0.5))
    }

    override fun periodic() {
        //estimate robot pose based on what the encoders say
        poseEstimator.update(getYaw(), getModulePositions())
        //estimate robot pose based on what the camera sees
        seesAprilTag.setBoolean(camera.captureFrame().hasTargets())
        val visionEst: Optional<EstimatedRobotPose>? = camera.getEstimatedPose()
        visionEst?.ifPresent { est ->

           poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, camera.getEstimationStdDevs(est.estimatedPose.toPose2d()))
        }
        //update module tabs on shuffleboard
        for (mod in modules){
            mod.updateShuffleboard()
        }
        //update drivetrain tab on shuffleboard
        field.robotPose = getPose()
        poseXEntry.setDouble(getPose().x)
        poseYEntry.setDouble(getPose().y)


    }

    override fun simulationPeriodic() {
        for ((x, state) in constructModuleStatesFromChassisSpeeds(0.0, 0.0,0.1, true).withIndex()){
            modules[x].setPointEntry.setDouble(state.angle.degrees)
        }
    }

    fun zeroYaw() {
        gyro.reset()
    }

    private fun resetModulesToAbsolute(){
        for (mod in modules){
            mod.resetToAbsolute()
        }
    }
    
    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED_METERS_PER_SECOND)

        for (mod in modules) {
            mod.setDesiredState(desiredStates[mod.MODULE_NUMBER - 1])
        }
    }

    fun constructModuleStatesFromChassisSpeeds(xSpeedMetersPerSecond:Double, ySpeedMetersPerSecond:Double, turningSpeedDegreesPerSecond:Double, fieldOriented:Boolean) : Array<SwerveModuleState> =
        kinematics.toSwerveModuleStates (
            if (fieldOriented)
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedMetersPerSecond,
                    ySpeedMetersPerSecond,
                    Rotation2d.fromDegrees(turningSpeedDegreesPerSecond).radians,
                    getYaw()
                )
            else
                ChassisSpeeds(
                    xSpeedMetersPerSecond,
                    ySpeedMetersPerSecond,
                    Rotation2d.fromDegrees(turningSpeedDegreesPerSecond).radians
                )
        )
    fun stop(){
        for(mod in modules){
            mod.stop()
        }
    }

    fun stopCommand() =
        run { stop() }
            .until { joystickControlledEntry.getBoolean(true) }
            .beforeStarting (runOnce { joystickControlledEntry.setBoolean(false) } )


    fun digInCommand() =
        runOnce {
            joystickControlledEntry.setBoolean(false)
            //because driving can be field-oriented, we need to take an angle perprendicular to the direction of the wheels, so that motion stops if we are being pushed
            }.andThen(
            run{
                setModuleStates(
                    arrayOf(SwerveModuleState(0.01, Rotation2d.fromDegrees(45.0)),
                            SwerveModuleState(0.01, Rotation2d.fromDegrees(-45.0)),
                            SwerveModuleState(0.01, Rotation2d.fromDegrees(45.0)),
                            SwerveModuleState(0.01, Rotation2d.fromDegrees(-45.0))
                ))
            }.until { joystickControlledEntry.getBoolean(true) })

    //reset yaw on gyro so that wherever the gyro is pointing is the new forward(0) value
    fun zeroHeadingCommand() =
        runOnce { zeroYaw() }

    //move robot to pose given in endpose argument
    fun moveToPoseCommand(endPose: Pose2d) =
        run {
            setModuleStates(
                constructModuleStatesFromChassisSpeeds(
                -xPIDController.calculate(getPose().x, endPose.x),
                -yPIDController.calculate(getPose().y,  endPose.y),
                -thetaPIDController.calculate(getYaw().degrees, endPose.rotation.degrees),
                true
            ))
        }
            .until {
                abs(getPose().translation.getDistance(endPose.translation)) < 0.02
                    && abs(getYaw().degrees - endPose.rotation.degrees) < 1.5
            }

    //getters
    fun getYaw() = Rotation2d.fromDegrees(
        -(gyro.rotation2d.degrees + yawOffset).IEEErem(360.0)
    )
    fun getPitch() = Rotation2d.fromDegrees(gyro.pitch.toDouble())
    fun getPose() = Pose2d(poseEstimator.estimatedPosition.x, poseEstimator.estimatedPosition.y, poseEstimator.estimatedPosition.rotation)
    fun getModuleStates(): Array<SwerveModuleState> = modules.map { it.getState() }.toTypedArray()
    fun getModulePositions(): Array<SwerveModulePosition> = modules.map { it.getPosition() }.toTypedArray()

    //sets poseEstimator's recorded position to a pose2d given in startingpose argument
    fun zeroPoseToFieldPosition(startingPose: Pose2d){
        yawOffset = startingPose.rotation.degrees
        resetModulesToAbsolute()
        poseEstimator.resetPosition(getYaw(), getModulePositions(), startingPose)
    }

    //does the same thing as previous but with the camera finding the starting pose
    fun zeroPoseToCameraPosition() =
        camera.getEstimatedPose()?.ifPresent {
            zeroPoseToFieldPosition(it.estimatedPose.toPose2d())
        }
}

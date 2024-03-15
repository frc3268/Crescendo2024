package crescendo.subsystems

import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.*
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.numbers.*
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.*
import org.photonvision.targeting.*
import java.io.IOException
import java.util.*

class CameraSubsystem(name: String, path: String): SubsystemBase() {
    private val limelight = PhotonCamera(name)
    private var frame = PhotonPipelineResult()
    private var poseEstimator: PhotonPoseEstimator? = null

    init {
        try {
            poseEstimator =
                PhotonPoseEstimator(
                    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    limelight,
                    Transform3d(
                        Translation3d(
                            Units.inchesToMeters(0.0),
                            Units.inchesToMeters(12.0),
                            Units.inchesToMeters(15.0),
                        ),
                        Rotation3d(0.0, 45.0, 0.0)
                    )
                )
        } catch (e: IOException) {
            DriverStation.reportError("AprilTag: Failed to Load", e.stackTrace)
            // !add some way to lock down AprilTag features after this
        }
    }

    override fun periodic() {
        captureFrame()
    }

    // Call periodically
    fun captureFrame(): PhotonPipelineResult =
        limelight.latestResult

    fun getAprilTagTarget(): PhotonTrackedTarget? {
        limelight.pipelineIndex = 1
        return if(frame.hasTargets()) frame.bestTarget else null
    }

    fun getReflectiveTapeTarget(): PhotonTrackedTarget?{
        limelight.pipelineIndex = 0
        return if(frame.hasTargets()) frame.bestTarget else null
    }

    fun getEstimatedPose(): Optional<EstimatedRobotPose>? =
        poseEstimator?.update()

    fun getEstimationStdDevs(estimatedPose: Pose2d): Matrix<N3, N1> {
        var estStdDevs = VecBuilder.fill(.7,.7,.9999999)

        val targets = captureFrame().getTargets()
            .mapNotNull { poseEstimator?.fieldTags?.getTagPose(it.fiducialId) }
            .filterNot { it.isEmpty }
        val numTags = targets.size
        val avgDist = (targets
            .sumOf { it.get().toPose2d().translation.getDistance(estimatedPose.translation) }
            / numTags)

        estStdDevs =
            if (numTags == 0) VecBuilder.fill(.7,.7,.9999999)
            // Decrease std devs if multiple targets are visible
            else if (numTags > 1) VecBuilder.fill(0.5, 0.5, 1.0)
            // Increase std devs based on (average) distance
            else if (avgDist > 4)
                VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
            else estStdDevs.times(1 + (avgDist * avgDist / 30))

        return estStdDevs
    }

    fun resetPose(pose: Pose2d) {
        poseEstimator ?: return
        poseEstimator?.setReferencePose(pose)
    }
}
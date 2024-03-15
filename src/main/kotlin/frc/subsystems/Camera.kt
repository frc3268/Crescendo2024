package frc.subsystems

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

class Camera(name:String, path:String): SubsystemBase(){
    val limelight = PhotonCamera(name)
    var frame = PhotonPipelineResult()
    var poseEstimator: PhotonPoseEstimator? = null

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
        // !add some way to lock down apriltage features after this
        }
    }

    override fun periodic() {
        captureFrame()
    }

    //call periodically
    fun captureFrame() : PhotonPipelineResult =
        limelight.latestResult

    fun getAprilTagTarget():PhotonTrackedTarget? {
        limelight.pipelineIndex = 1
        return if(frame.hasTargets()) frame.bestTarget else null
    }

    fun getReflectiveTapeTarget():PhotonTrackedTarget?{
        limelight.pipelineIndex = 0
        return if(frame.hasTargets()) frame.bestTarget else null
    }

    fun getEstimatedPose(): Optional<EstimatedRobotPose>? =
        poseEstimator?.update()

    //stolen from  photonvision(blatantly)
    fun getEstimationStdDevs(estimatedPose: Pose2d): Matrix<N3, N1> {
        //todo: expiriment with vecbuilder values(somehow)
        var estStdDevs =  VecBuilder.fill(.7,.7,.9999999)
        val targets = captureFrame().getTargets()
        var numTags = 0
        var avgDist = 0.0
        
        for (tgt in targets) {
            val tagPose = poseEstimator?.fieldTags?.getTagPose(tgt.fiducialId) ?: continue
            if (tagPose.isEmpty) continue
            numTags++
            avgDist +=
                tagPose.get().toPose2d().translation.getDistance(estimatedPose.translation)
        }
        if (numTags == 0) return estStdDevs
        avgDist /= numTags
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VecBuilder.fill(0.5, 0.5, 1.0)
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30))

        return estStdDevs
    }


    fun resetPose(pose: Pose2d) {
        poseEstimator ?: return
        poseEstimator?.setReferencePose(pose)
    }
}
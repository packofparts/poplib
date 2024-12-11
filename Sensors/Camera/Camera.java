package POPLib.Sensors.Camera;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public class Camera {
    private final PhotonCamera camera;
    private final CameraConfig config;
    private PhotonPoseEstimator poseEstimator;

    public Camera(CameraConfig config) {
        this.config = config;
        camera = new PhotonCamera(config.cameraName);
        AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        poseEstimator = new PhotonPoseEstimator(aprilTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, config.cameraToRobot);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    }

    public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d currPose) {
        if (!camera.isConnected()) {
            return Optional.empty();
        }
        poseEstimator.setReferencePose(currPose);
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult change : camera.getAllUnreadResults()) {
            visionEst = poseEstimator.update(change);
        }
        if (visionEst.isPresent() && isValidPose(visionEst.get())) {
            return visionEst;
        }
        return Optional.empty();
    } 

    private boolean isValidPose(EstimatedRobotPose pose) {
        List<PhotonTrackedTarget> targets = pose.targetsUsed;
        if (targets.size() == 1) {
            return targets.get(0).getPoseAmbiguity() >= config.poseAmbiguityThreshold;
        }
        return true;
    }
}
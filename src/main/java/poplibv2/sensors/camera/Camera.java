package poplibv2.sensors.camera;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;



public class Camera {
    private final PhotonCamera camera;
    private final CameraConfig config;
    private PhotonPoseEstimator poseEstimator;
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
    private Matrix<N3, N1> currStdDevs = null;
    private AprilTagFieldLayout layout;

    /**
     * Creates a new Photon Vision Camera
     * @param config
     */
    public Camera(CameraConfig config) {
        this.config = config;
        camera = new PhotonCamera(config.cameraName);
        layout = AprilTagFieldLayout.loadField(config.aprilTagField);
        poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, config.cameraToRobot);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    }

    /**
     * returns the distance between the best April tag (tag with the lowest reprojection error) and the focal lens of the camera.
     * Can be combined with the cameraToRobot mapping that you gave in CameraConfig to make an AprilTag to robot vector.
     * @return The Pose2d as an Optional as the camera might not see a valid april tag. Always use the .isPresent() method before the .get() method.
     */
    public Optional<Pose2d> relativeDistanceFromCameraToAprilTag() {
        if (!camera.isConnected()) {
            DriverStation.reportError("Camera named: " + config.cameraName + " is not connected!!!!!!!!", false);
            // the above code should save to the log file that you can view in the DS Log Viewer
            return Optional.empty();
        }
        ArrayList<PhotonTrackedTarget> poses = new ArrayList<>();
        
        Optional<PhotonTrackedTarget> ret1 = Optional.empty();
        Optional<Pose2d> ret = Optional.empty();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (result.hasTargets()) {
                List<PhotonTrackedTarget> target = result.getTargets();
                for (var i : target) {
                    poses.add(i);
                }
                // Optional<Pose3d> pose = layout.getTagPose(target.getFiducialId());
                // if (pose.isPresent()) {
                //     poses.add(new Pose2d(
                //         new Translation2d(target.getBestCameraToTarget().getX(), target.getBestCameraToTarget().getY()), 
                //         Rotation2d.fromRadians(pose.get().getRotation().getZ())));
                // }
            }
        }

        for (var pose : poses) {
            if (ret1.isEmpty() || dist(ret1.get().getBestCameraToTarget()) > dist(pose.getBestCameraToTarget())) {
                ret1 = Optional.of(pose);
            }
        }

        if (ret1.isPresent()) {
        Optional<Pose3d> pose = layout.getTagPose(ret1.get().getFiducialId());
                if (pose.isPresent()) {
                    ret = Optional.of(new Pose2d(
                        new Translation2d(ret1.get().getBestCameraToTarget().getX(), ret1.get().getBestCameraToTarget().getY()), 
                        Rotation2d.fromRadians(pose.get().getRotation().getZ())));
                }}
        return ret;
    } 

    /**
     * The mathematical distance formula that you learn in algebra.
     */
    private double dist(Transform3d pose) {
        return Math.sqrt(Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2));
    }

    /**
     * Returns the estimated position of the robot based on april tag readings from the camera and the current pose of the robot.
     * Meant to be used in SwerveDrivePoseEstimator.addVisionMeasurement()
     * @param currPose the current position of the robot
     * @return The new position of the robot as a EstimatedRobotPose wrapped in an Optional as the camera might not have seen anything. Always use .isPresent() before .get()
     */
    public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d currPose) {
        if (!camera.isConnected()) {
            DriverStation.reportError("Camera named: " + config.cameraName + " is not connected!!!!!!!!", false);
            // the above code should save to the log file that you can view in the DS Log Viewer
            return Optional.empty();
        }
        poseEstimator.setReferencePose(currPose);
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult change : camera.getAllUnreadResults()) {
            visionEst = poseEstimator.update(change);
            updateStdDevs(visionEst, change.getTargets());
        }
        if (visionEst.isPresent() && isValidPose(visionEst.get())) {
            return visionEst;
        }
        return Optional.empty();
    }

    /**
     * returns the current calculuated standard deviations for the getEstimatedPose calculation.
     * Meant to be used in SwerveDrivePoseEstimator.addVisionMeasurement()
     * @return the stdDevs in X, Y, Z format.
     */
    public Matrix<N3, N1> getVisionStdDevs() {
        return currStdDevs;
    }

    /**
     * Updates the standard deviations of the April Tag Pose.
     * @param visionEst The current estimated pose of the robot.
     * @param targets The targets (april tags to go through)
     */
    private void updateStdDevs(Optional<EstimatedRobotPose> visionEst, List<PhotonTrackedTarget> targets) {
        if (visionEst.isEmpty()) {
            currStdDevs = singleTagStdDevs;
        } else {
            Matrix<N3, N1> stdDevs = singleTagStdDevs;
            int numTags = 0;
            double avg = 0;
            for (PhotonTrackedTarget target : targets) {
                if (config.stdDevStategy == StdDevStategy.DISTANCE) {
                    Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
                    if (tagPose.isPresent()) {
                        numTags++;
                        avg += tagPose.get().toPose2d().getTranslation().getDistance(
                        visionEst.get().estimatedPose.toPose2d().getTranslation());
                    }
                } else if (config.stdDevStategy == StdDevStategy.AMBIGUITY) {
                    avg += target.getPoseAmbiguity();
                    numTags++;
                }
            }
            if (numTags == 0) {
                currStdDevs = singleTagStdDevs;
            } else {
                avg /= numTags;
                if (numTags > 1) {
                    stdDevs = VecBuilder.fill(0.5, 0.5, 1);
                }
                if (config.stdDevStategy == StdDevStategy.DISTANCE) {
                    if (numTags == 1 && avg > 4) {
                        stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);  // tag too unreliable
                    } else {
                        stdDevs = stdDevs.times(1 + (avg * avg / 30));
                    }
                } else if (config.stdDevStategy == StdDevStategy.AMBIGUITY) {
                    if (avg > 0.4) {    // ur cooked lil bro
                        stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    } else {
                        stdDevs = stdDevs.times(1 + (avg * avg * 30));
                    }
                }
                currStdDevs = stdDevs;
            }
        }
    }

    /**
     * Checks if a pose is valid or not based on the thresholds provided in CameraConfig 
     * @param pose The pose to check
     * @return Whether or not it is valid.
     */
    private boolean isValidPose(EstimatedRobotPose pose) {
        List<PhotonTrackedTarget> targets = pose.targetsUsed;
        double max_dist = 0;
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                max_dist = Math.max(max_dist, tagPose.get().toPose2d().getTranslation().
                getDistance(pose.estimatedPose.toPose2d().getTranslation()));
            }
        }
        if (max_dist >= config.poseDistanceThreshold) {
            return false;
        }
        if (targets.size() == 1) {
            return targets.get(0).getPoseAmbiguity() <= config.poseAmbiguityThreshold;
        }
        return true;
    }
}
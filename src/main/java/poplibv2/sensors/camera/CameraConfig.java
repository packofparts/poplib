package poplibv2.sensors.camera;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraConfig {
    public final String cameraName;
    public final Transform3d cameraToRobot;
    public final double poseAmbiguityThreshold;
    public final double poseDistanceThreshold;
    public final StdDevStategy stdDevStategy;
    public final AprilTagFields aprilTagField;

    /**
     * Creates a new CameraConfig that should be passed into a Camera Object. 
     * This will be used when creating a new PhotonVision Camera
     * @param cameraName The camera name, as set in the PhotonVision Dashboard (Note: this is the CAMERA name, not the name of the computer running photonvision)
     * @param cameraToRobot A 3d vector detailing the difference in positions between the center of the robot (where the gyro is) and the focal lens of the camera. See the WPILIB robot coordinate system on how to do this
     * @param poseAmbiguityThreshold The maximum allowed ambiguity for a detected april tag pose to still be valid
     * @param poseDistanceThreshold The maximum allowed distance for a detected april tag pose to still be valid
     * @param stdDevStategy The Standard Deviation calculuated stategy to be used when calculating the variable standard deviations of an april tag pose.
     * @param thisYearsField The AprilTagFields object that details the april tag layout for this years field.
     */
    public CameraConfig(String cameraName, Transform3d cameraToRobot, double poseAmbiguityThreshold, 
                        double poseDistanceThreshold, StdDevStategy stdDevStategy, AprilTagFields thisYearsField) {
        this.cameraName = cameraName;
        this.cameraToRobot = cameraToRobot;
        this.poseAmbiguityThreshold = poseAmbiguityThreshold; 
        this.poseDistanceThreshold = poseDistanceThreshold;
        this.stdDevStategy = stdDevStategy;
        this.aprilTagField = thisYearsField;
    }
}

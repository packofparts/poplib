package POPLib.Sensors.Camera;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraConfig {
    public final String cameraName;
    public final Transform3d cameraToRobot;
    public final double poseAmbiguityThreshold;
    public final StdDevStategy stdDevStategy;

    public CameraConfig(String cameraName, Transform3d cameraToRobot, double poseAmbiguityThreshold, StdDevStategy stdDevStategy) {
        this.cameraName = cameraName;
        this.cameraToRobot = cameraToRobot;
        this.poseAmbiguityThreshold = poseAmbiguityThreshold; 
        this.stdDevStategy = stdDevStategy;
    }
}

enum StdDevStategy {
    DISTANCE,    // recommended strategy
    AMBIGUITY
}

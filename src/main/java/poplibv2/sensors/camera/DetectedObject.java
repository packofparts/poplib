package poplibv2.sensors.camera;

public class DetectedObject {
    public final double xAngleOffset;
    public final double yAngleOffset;
    public final double area;
    public final String objectType;

    /**
     * A container for storing objects detected by a Camera. This is a public container, please access the values directly
     * @param xAngleOffset The pixel offset between the object and the center of the camera in the x direction
     * @param yAngleOffset The pixel offset between the object and the center of the camera in the x direction
     * @param area What percent of the camera screen the object takes up
     * @param objectType The detected type of the object
     */
    public DetectedObject(double xAngleOffset, double yAngleOffset, double area, String objectType) {
        this.xAngleOffset = xAngleOffset;
        this.yAngleOffset = yAngleOffset;
        this.area = area;
        this.objectType = objectType;
    }
}
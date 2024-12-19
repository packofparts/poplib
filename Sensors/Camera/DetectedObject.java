package POPLib.Sensors.Camera;

import edu.wpi.first.wpilibj.DriverStation;

public class DetectedObject {
    private final double xAngleOffset;
    private final double yAngleOffset;
    private final double area;
    private final boolean valid;

    public DetectedObject(double xAngleOffset, double yAngleOffset, double area, boolean valid) {
        this.xAngleOffset = xAngleOffset;
        this.yAngleOffset = yAngleOffset;
        this.area = area;
        this.valid = valid;
    }

    public boolean isValidDetection() {
        return valid;
    }

    public double getXAngleOffset() {
        if (!valid) {
            complain();
        }
        return xAngleOffset;
    }

    public double getYAngleOffset() {
        if (!valid) {
            complain();
        }
        return yAngleOffset : 0.0;
    }

    public double getAreaOfObject() {
        if (!valid) {
            complain();
        }
        return area : 0.0;
    }

    private void complain() {
        DriverStation.reportError("Querying infomation from an invalid Limelight DetectedObject!!!!! \n
        Please remember to check if the detection is valid by calling isValidDetection()", false);
    }
}
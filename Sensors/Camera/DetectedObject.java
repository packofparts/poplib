package POPLib.Sensors.Camera;

import edu.wpi.first.wpilibj.DriverStation;

public class DetectedObject {
    private final double xAngleOffset;
    private final double yAngleOffset;
    private final double area;
    private final String objectType;
    private final boolean valid;

    public DetectedObject(double xAngleOffset, double yAngleOffset, double area, String objectType, boolean valid) {
        this.xAngleOffset = xAngleOffset;
        this.yAngleOffset = yAngleOffset;
        this.area = area;
        this.objectType = objectType;
        this.valid = valid;
    }

    public boolean isValidDetection() {
        return valid;
    }

    public double getXAngleOffset() {
        checkIfValid();
        return xAngleOffset;
    }

    public double getYAngleOffset() {
        checkIfValid();
        return yAngleOffset;
    }

    public double getAreaOfObject() {
        checkIfValid();
        return area;
    }

    public String getObjectType() {
        checkIfValid();
        return objectType;
    }

    private void checkIfValid() {
        if (!valid) {
            DriverStation.reportError("Querying infomation from an invalid Limelight DetectedObject!!!!! \n
            Please remember to check if the detection is valid by calling isValidDetection()", false);
        }
    }
}
package POPLib.Sensors.Camera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable networkTable;

    public Limelight(String limelightName) {
        networkTable = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    public DetectedObject getLastestDetection() {
        return new DetectedObject(
            networkTable.getEntry("tx").getDouble(0.0),
            networkTable.getEntry("ty").getDouble(0.0),
            networkTable.getEntry("ta").getDouble(0.0),
            networkTable.getEntry("tv").getInteger(0) == 1 ? true : false
        );
    }
}
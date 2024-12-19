package POPLib.Sensors.Camera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable networkTable;
    private LimelightConfig config.

    public Limelight(LimelightConfig config) {
        this.config = config;
        networkTable = NetworkTableInstance.getDefault().getTable(config.limelightName);
        networkTable.getEntry("ledMode").setNumber(config.ledState);
        networkTable.getEntry("pipeline").setNumber(config.pipeline);

    }

    public DetectedObject getLastestDetection() {
        double area = networkTable.getEntry("ta").getDouble(0.0);
        return new DetectedObject(
            networkTable.getEntry("tx").getDouble(0.0),
            networkTable.getEntry("ty").getDouble(0.0), area,
            networkTable.getEntry("tclass").getString(""),
            (networkTable.getEntry("tv").getInteger(0) == 1 && area > minValidArea) ? true : false
        );
    }
}
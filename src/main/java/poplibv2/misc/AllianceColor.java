package poplibv2.misc;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AllianceColor {
    private boolean isRedAlliance;
    private NetworkTable table;

    public static AllianceColor instance; 

    /**
     * Gets the AllianceColor object.
     * @return
     */
    public static AllianceColor getInstance() {
        if (instance == null) {
            instance = new AllianceColor();
        }
        return instance;
    }

    /**
     * Creates a new AllianceColor object.
     */
    private AllianceColor() {
        table = NetworkTableInstance.getDefault().getTable("FMSInfo");
        isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);
    }

    /**
     * Updates the current alliance color. Should be called in Teleop Init, or after the match has started.
     */
    public void updateAllianceColor() {
        isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);
    }

    /**
     * Returns if we are on Red Alliance.
     */
    public boolean isRed() {
        return isRedAlliance;
    }
 
    /**
     * Returns if we are on Blue Alliance.
     */
    public boolean isBlue() {
        return !isRedAlliance;
    }
}

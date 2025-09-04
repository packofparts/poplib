package poplibv2.misc;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for a tunable number. This class gets a value from smart dashboard in tuning mode, 
 * it returns the default value if tuning mode is not on or the value is not in dashboard.
 */
public class TunableNumber {
    private static final String tableKey = "TunableNumbers";
    private final String key;
    private double defaultValue;
    private double lastHasChangedValue = defaultValue;
    private final boolean tuningMode;

    /**
     * Creates a new TunableNumber
     * @param dashboardKey the key to use when displaying this on smart dashboard
     */
    public TunableNumber(String dashboardKey, double defaultValue, boolean tuningMode) {
        this.key = tableKey + "/" + dashboardKey;
        this.tuningMode = tuningMode;
        set(defaultValue);
    }

    /**
     * Get the default value for the number
     * @return The default value
     */
    public double getDefault() {
        return defaultValue;
    }

    /**
     * Sets a new value for the TunableNumber
     * @param value The default value
     */
    public void set(double value) {
        this.defaultValue = value;
        if (tuningMode) {
            // This makes sure the data is on NetworkTables but will not change it
            SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
        } else if (SmartDashboard.containsKey(key) && SmartDashboard.isPersistent(key)) {
            SmartDashboard.clearPersistent(key);
        }
    }

    /**
     * Get the current value of the TunableNumber
     * @return The current value
     */
    public double get() {
        return tuningMode ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
    }

    /**
     * Checks whether the number has changed since the last check
     * @return true if the number has changed since the last time this method was called, false otherwise
     */
    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue != lastHasChangedValue) {
            lastHasChangedValue = currentValue;
            return true;
        }
        return false;
    }
}

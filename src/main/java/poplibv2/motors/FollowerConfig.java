package poplibv2.motors;

public class FollowerConfig {
    int canID;
    boolean inverted;

    /**
     * Creates a new Follower Config
     * @param CANID The CAN Id of the FOLLOWER Motor
     * @param inverted Whether or not the motor should do the exact same (false) or exact opposite (true) of what this motor is doing
     */
    public FollowerConfig(int canID, boolean inverted) {
        this.canID = canID;
        this.inverted = inverted;
    }
}
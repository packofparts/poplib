package poplibv2.sensors.gyro;

public class PigeonConfig {
    public int id;
    public boolean inversion;
    public String canBusName;

    /**
     * A Config for the Pigeon/gyro
     * @param id the CAN ID
     * @param inversion if it should be inverted bc electrical place it backward again
     * @param canBusName the name of the can bus loop it is on
     */
    public PigeonConfig(int id, boolean inversion, String canBusName) {
        this.id = id;
        this.inversion = inversion;
        this.canBusName = canBusName;
    }
}

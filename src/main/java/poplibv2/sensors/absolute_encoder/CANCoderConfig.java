package poplibv2.sensors.absolute_encoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class CANCoderConfig {
    public final int id;
    public final String CANBus;
    public final Rotation2d offset;
    public final boolean inversion;

    public CANCoderConfig(int id, String CANBus, Rotation2d offset, boolean inversion) {
        this.id = id;
        this.CANBus = CANBus;
        this.offset = offset;
        this.inversion = inversion;
    }

}

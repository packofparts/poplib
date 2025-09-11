package poplibv2.sensors.absolute_encoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class CANCoderConfig {
    public final int id;
    public final String CANBus;
    public final Rotation2d offset;
    public final boolean inversion;

    /**
     * Creates a new Config for a CANCoder.
     * @param id The CAN Id of the CANCoder.
     * @param CANBus The CANBus of the CANCoder
     * @param offset The offset of the CANCoder, used to align your swerve wheels.
     * @param inversion Whether or not the encoder should be inverted.
     */
    public CANCoderConfig(int id, String CANBus, Rotation2d offset, boolean inversion) {
        this.id = id;
        this.CANBus = CANBus;
        this.offset = offset;
        this.inversion = inversion;
    }

}

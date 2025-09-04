package poplibv2.sensors.absolute_encoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import poplibv2.misc.CanIdRegistry;

public class CANCoder {
    private CANcoder encoder;

    /**
     * Creates a new CANCoder (absolute encoder) using the CANCoderConfig
     * @param config the config to be used
     */
    public CANCoder(CANCoderConfig config) {
        encoder = new CANcoder(config.id, config.CANBus);
        CanIdRegistry.getRegistry().registerCanId(config.id);

        CANcoderConfiguration sensorConfig = new CANcoderConfiguration();
        sensorConfig.MagnetSensor.SensorDirection = SensorDirectionValue.valueOf(config.inversion ? 1 : 0);
        sensorConfig.MagnetSensor.MagnetOffset = config.offset.getRotations();

        encoder.getConfigurator().apply(sensorConfig);
    }

    /**
     * Gets the position the CANCoder is displaying
     * @return The position as a Rotation2d in rotations.
     */
    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue().in(Units.Rotations));
    }
}

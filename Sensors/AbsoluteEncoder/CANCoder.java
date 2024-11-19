package POPLib.Sensors.AbsoluteEncoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class CANCoder extends AbsoluteEncoder {
    private CANcoder encoder;

    public CANCoder(AbsoluteEncoderConfig config) {
        super(config);

        this.encoder = new CANcoder(config.id);

        CANcoderConfiguration sensorConfig = new CANcoderConfiguration();
        sensorConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        sensorConfig.MagnetSensor.SensorDirection = SensorDirectionValue.valueOf(config.inversion ? 1 : 0);
        sensorConfig.MagnetSensor.MagnetOffset = config.offset.getRotations();

        encoder.getConfigurator().apply(sensorConfig);
    }

    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue());
    }
}

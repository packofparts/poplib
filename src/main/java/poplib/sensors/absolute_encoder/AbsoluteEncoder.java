package poplib.sensors.absolute_encoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract class AbsoluteEncoder {
    protected final AbsoluteEncoderConfig config;
    
    public AbsoluteEncoder(AbsoluteEncoderConfig config) {
        this.config = config;
    }

    public abstract Rotation2d getPosition();

    public double getDegreeNormalizedPosition() {
        return MathUtil.inputModulus(getPosition().getDegrees(), -180, 180);
    }
}

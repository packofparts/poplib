package POPLib.Sensors.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class AbsoluteEncoder {
    public abstract Rotation2d getPosition();
}

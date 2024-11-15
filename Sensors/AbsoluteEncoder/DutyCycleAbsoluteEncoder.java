package POPLib.Sensors.AbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DutyCycleAbsoluteEncoder extends AbsoluteEncoder{
    private DutyCycleEncoder encoder;
    private double offset;
    private int inversion;

    public DutyCycleAbsoluteEncoder(int id, double offset, boolean inversion) {
        encoder = new DutyCycleEncoder(id);
        this.offset = offset;

        if (inversion) {
            this.inversion = -1;
        } else {
            this.inversion = 1;
        }
    }

    public double getDegreePosition() {
        return encoder.get() * 360 * inversion - offset;
    }

    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromDegrees(getDegreePosition());
    }

    public double getDegreeNormalizedPosition() {
        return MathUtil.inputModulus(getDegreePosition(), -180, 180);
    }
}
